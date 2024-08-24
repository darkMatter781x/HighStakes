#pragma once

#include "Eigen/Dense"
#include "kalman/types.h"

namespace kalman {

namespace unscented_transform {

/**
 * @brief Provides the necessary operators for the unscented transform.
 * For example, if your state contains an angle, the default mean and diff
 * functions will not work.
 *
 * @tparam N The dimensionality of the state
 * @tparam SIGMA_N The number of sigma points
 */
template <size_t N, size_t SIGMA_N> struct Operators {
    /**
     * @brief Computes the weighted mean of the sigma points.
     */
    virtual Eigen::Matrix<float, N, 1>
    mean(const Eigen::Matrix<float, N, SIGMA_N>& sigma_points,
         const Eigen::Matrix<float, SIGMA_N, 1>& weights) {
      return sigma_points * weights;
    };

    /**
     * @brief Computes the difference between the two params.
     */
    virtual Eigen::Matrix<float, N, 1>
    diff(const Eigen::Matrix<float, N, 1>& lhs,
         const Eigen::Matrix<float, N, 1>& rhs) {
      return lhs - rhs;
    }
};

/**
 * @brief Transforms the sigma points into a state mean and covariance.
 *
 * @tparam N The dimensionality of the state
 * @tparam SIGMA_N The number of sigma points
 * @param sigmas The sigma points.
 * @param weights The weights for the sigma points.
 * @param ops The operators to use for the transform.
 * @return State<N>
 */
template <size_t N, size_t SIGMA_N> State<N>
transform(const Eigen::Matrix<float, N, SIGMA_N>& sigmas,
          const Weights<SIGMA_N>& weights,
          const Operators<N, SIGMA_N>& ops = Operators<N, SIGMA_N> {}) {
  State<N> state;
  state.mean = ops.mean(sigmas, weights.mean);
  for (size_t i = 0; i < SIGMA_N; i++) {
    Eigen::Matrix<float, N, 1> diff = ops.diff(sigmas.col(i), state.mean);
    state.covar += diff * weights.covar(i) * diff.transpose();
  }
  return state;
}
} // namespace unscented_transform

namespace UT = unscented_transform;
} // namespace kalman