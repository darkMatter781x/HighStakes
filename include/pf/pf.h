#pragma once

#include <array>
#include <cfloat>
#include <memory>
#include <numeric>
#include "pros/rtos.hpp"
#include "pf/resamplers/resampler.h"
#include "sensors/sensor.h"

/**
 * @brief Abstract object for Particle Filter algorithm
 *
 * @tparam P Particle type. Must be addable by P, and multipliable by float.
 * @tparam N Number of particles.
 */
template <typename P, size_t N> class ParticleFilter {
  private:
    /**
     * @brief Ensures only one operation occurs on particle filter at a time.
     */
    pros::Mutex m_mutex;

    std::unique_ptr<std::array<P, N>> m_particles;
    /** @brief Weights corresponding to the particles. */
    std::unique_ptr<std::array<float, N>> m_weights;

    /**
     * @brief Used to allocate memory once for resampling. Is not shared
     * between calls to the particle filter.
     */
    std::unique_ptr<std::array<size_t, N>> m_indices;

    /**
     * @brief Used to allocate memory once for resampling. Is not shared
     * between calls to the particle filter.
     */
    std::unique_ptr<std::array<P, N>> m_newParticles;
  public:
    using Resampler = resamplers::Resampler<N>;
    using Sensor = sensors::Sensor<P, N>;

    /** @brief Moves particles and incorporates noise. */
    struct Predictor {
        /**
         * @brief Modifies particles param according to system model and
         * incorporates noise.
         */
        virtual void predict(std::array<P, N>& particles) = 0;
    };

    /**
     * @brief Utility for normalizing weights
     * @time O(N)
     */
    static void normalizeWeights(std::array<float, N>& weights) {
      float sum = std::reduce(weights.begin(), weights.end());
      // TODO: Investigate if this is necessary/correct
      // Avoid round of to 0
      sum += FLT_MIN * N;
      for (float& weight : weights) {
        weight += FLT_MIN;
        weight /= sum;
      }
    }

    /** Moves particles and incorporates noise. */
    void predict(Predictor& predictor) {
      m_mutex.take();
      predictor.predict(*m_particles);
      m_mutex.give();
    }

    /** Updates weights, and resamples if needed. */
    void update(std::vector<std::unique_ptr<Sensor>>& sensors,
                Resampler& resampler) {
      m_mutex.take();
      // Update weights
      for (std::unique_ptr<Sensor>& sensor : sensors)
        sensor->update(*m_particles, *m_weights);
      // Normalize weights
      normalizeWeights(*m_weights);

      // TODO: Remove the need for m_indices and m_newParticles
      // (Some resamplers may not need them)

      // If should resample, then resample
      if (resampler.shouldResample(*m_weights)) {
        // printf("[%ims] Resampling!\n", pros::millis());
        // Get indices of particles to keep
        m_indices = resampler.resample(*m_weights, std::move(m_indices));
        // Perform the actual resampling
        for (size_t i = 0; i < N; i++)
          (*m_newParticles)[i] = (*m_particles)[(*m_indices)[i]];
        m_particles.swap(m_newParticles);
        // Reset weights
        std::fill(m_weights->begin(), m_weights->end(), 1.f / N);
      }
      m_mutex.give();
    };

    /**
     * @brief Computes the weighted average of the particles, aka the state
     * estimate.
     * @time O(N)
     */
    P getMean() {
      m_mutex.take();
      P mean;
      for (size_t i = 0; i < N; i++)
        mean += (*m_particles)[i] * (*m_weights)[i];
      m_mutex.give();
      return mean;
    }

    struct Data {
        std::unique_ptr<std::array<P, N>> particles;
        std::unique_ptr<std::array<float, N>> weights;
    };

    /**
     * @brief Lets the particles and weights be modified by a function.
     * @param func Function to given ability to modify the particles.
     */
    void modify(std::function<Data(Data)> func) {
      m_mutex.take();
      Data data {
          .particles = std::move(m_particles),
          .weights = std::move(m_weights),
      };
      Data newData = func(std::move(data));
      m_particles = std::move(newData.particles);
      m_weights = std::move(newData.weights);
      m_mutex.give();
    }

    /** @brief Sets the PF's weights */
    void setWeights(const std::array<float, N>& weights) {
      m_mutex.take();
      m_weights = weights;
      m_mutex.give();
    }

    /** Create a particle filter. */
    ParticleFilter(std::unique_ptr<std::array<P, N>> particles)
      : m_particles(std::move(particles)),
        m_weights(std::make_unique<std::array<float, N>>()),
        m_indices(std::make_unique<std::array<size_t, N>>()),
        m_newParticles(std::make_unique<std::array<P, N>>()) {
      std::fill((*m_weights).begin(), (*m_weights).end(), 1. / N);
    }
};