#pragma once

#include "pf/resamplers/resampler.h"
#include <functional>

namespace resamplers {

// TODO: Test this
/** @brief Implements Systematic Resampling */
template <size_t N> class Systematic : public Resampler<N> {
  private:
    /** @brief Returns random number between 0 and 1 */
    std::function<float()> m_random;
  public:
    std::unique_ptr<Eigen::Array<size_t, N, 1>>
    resample(const Eigen::Array<float, N, 1>& weights,
             std::unique_ptr<Eigen::Array<size_t, N, 1>> indices) override {
      float offset = m_random() / N;

      /** Current sum. */
      float currSum = 0;
      /** Value we want to find. */
      float position = offset;
      /** Index of weights */
      size_t i_w = 0;
      /** Index of new indices */
      size_t i_i = 0;

      for (; i_w < N; i_w++) {
        // Update sum
        currSum += weights[i_w];

        // If current weight is at desired position
        while (currSum > position) {
          if (i_i >= N) {
            printf("Systematic: Index out of bounds: %d\n", i_i);
            printf("currSum: %f\n", currSum);
            printf("position: %f\n", position);
            break;
          }
          // Then add it to the indices
          (*indices)[i_i] = i_w;
          ++i_i;
          // And move to next position
          position += 1.f / N;
        }
      }

      return std::move(indices);
    };

    /** @param random Returns random number between 0 and 1 */
    Systematic(std::function<float()> randomFunc) : m_random(randomFunc) {}
};
}; // namespace resamplers