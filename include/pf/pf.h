#pragma once

#include <algorithm>
#include <array>
#include <numeric>
#include "pros/rtos.hpp"
#include "pf/resamplers/resampler.h"

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

    /** @brief Moves particles and incorporates noise. */
    struct Predictor {
        /**
         * @brief Modifies particles param according to system model and
         * incorporates noise.
         */
        virtual void predict(std::array<P, N>& particles) = 0;
    };

    /** @brief Updates weights with measurement. */
    struct Updater {
        /**
         * @brief Modifies weights param based on measurement.
         * Must normalize weights (see `ParticleFilter<N>::normalizeWeights()`).
         */
        virtual void update(const std::array<P, N>& particles,
                            std::array<float, N>& weights) = 0;
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
    void update(Updater& updater, Resampler& resampler) {
      m_mutex.take();
      // Update weights (should be normalized)
      updater.update(m_particles, m_weights);
      // If must resample, resample
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

    /** @brief Sets the PF's particles */
    void setParticles(const std::array<P, N>& particles) {
      m_mutex.take();
      m_particles = particles;
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