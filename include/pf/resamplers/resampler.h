#pragma once

#include <array>

namespace resamplers {

/**
 * @brief Resamples particles
 * @tparam N Number of particles/weights
 */
template <size_t N> struct Resampler {
    /** @returns Whether resampling should be performed. */
    virtual bool shouldResample(const std::array<float, N>& weights) {
      float sumOfSquares = 0;
      for (const float& weight : weights) { sumOfSquares += weight * weight; }
      /** Effective N - Approximates number of particles meaningfully
       * contributing to probability distribution  */
      const float effN = 1 / sumOfSquares;
      return effN < (N / 2.f);
    }

    /** @brief Modifies indices array to contain selected indices. */
    virtual void resample(const std::array<float, N>& weights,
                          std::array<size_t, N>& indices) = 0;
};
} // namespace resamplers