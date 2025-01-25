#pragma once

#include <memory>
#include "Eigen/Core"

namespace resamplers {

/**
 * @brief Resamples particles
 * @tparam N Number of particles/weights
 */
template <size_t N> struct Resampler {
    /** @returns Whether resampling should be performed. */
    virtual bool shouldResample(const Eigen::Array<float, N, 1>& weights) {
      float sumOfSquares = 0;
      for (const float& weight : weights) { sumOfSquares += weight * weight; }
      /** Effective N - Approximates number of particles meaningfully
       * contributing to probability distribution  */
      const float effN = 1 / sumOfSquares;
      return effN < (N / 2.f);
    }

    /** @brief Modifies indices array to contain selected indices. */
    virtual std::unique_ptr<Eigen::Array<size_t, N, 1>>
    resample(const Eigen::Array<float, N, 1>& weights,
             std::unique_ptr<Eigen::Array<size_t, N, 1>> indices) = 0;
};
} // namespace resamplers