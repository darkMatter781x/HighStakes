#pragma once

#include <array>
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

    std::array<P, N> m_particles;
    /** @brief Weights corresponding to the particles. */
    std::array<float, N> m_weights;

    /**
     * @brief Used to allocate memory once for resampling. Is not shared
     * between calls to the particle filter.
     */
    std::array<size_t, N> m_indices;

    /**
     * @brief Used to allocate memory once for resampling. Is not shared
     * between calls to the particle filter.
     */
    std::array<P, N> m_newParticles;
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
      float sum = 0;
      std::reduce(weights.begin(), weights.end(), sum);
      for (float& weight : weights) {
        // TODO: Investigate if this is necessary/correct
        // Avoid round of to 0
        // weight += 1e-300;
        weight /= sum;
      }
    }

    /** Moves particles and incorporates noise. */
    void predict(Predictor& predictor) {
      m_mutex.take();
      predictor.predict(m_particles);
      m_mutex.give();
    }

    /** Updates weights, and resamples if needed. */
    void update(std::vector<std::unique_ptr<Sensor>>& sensors,
                Resampler& resampler) {
      m_mutex.take();
      // Update weights
      for (std::unique_ptr<Sensor>& sensor : sensors)
        sensor->update(m_particles, m_weights);
      // Normalize weights
      normalizeWeights(m_weights);

      // If should resample, then resample
      if (resampler.shouldResample(m_weights)) {
        // Get indices of particles to keep
        resampler.resample(m_weights, m_indices);
        // Perform the actual resampling
        for (size_t i = 0; i < N; i++)
          m_newParticles[i] = m_particles[m_indices[i]];
        m_particles = m_newParticles;
        // Reset weights
        std::fill(m_weights.begin(), m_weights.end(), 1.f / N);
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
      for (size_t i = 0; i < N; i++) mean += m_particles[i] * m_weights[i];
      m_mutex.give();
      return mean;
    }

    /**
     * @brief Lets the particles be modified by a function.
     * @param func Function to given ability to modify the particles.
     */
    void modifyParticles(std::function<void(std::array<P, N>&)> func) {
      m_mutex.take();
      func(m_particles);
      m_mutex.give();
    }

    /** @brief Sets the PF's weights */
    void setWeights(const std::array<float, N>& weights) {
      m_mutex.take();
      m_weights = weights;
      m_mutex.give();
    }

    /** Create a particle filter. */
    ParticleFilter(std::array<P, N> particles,
                   std::array<float, N> weights = {1 / N, N})
      : m_particles(particles), m_weights(weights) {}
};