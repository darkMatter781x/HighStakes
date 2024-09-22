#pragma once

#include "Eigen/Dense"

namespace kalman {

/**
 * @brief Represents a state mean and its covariance.
 * @tparam N The dimensionality of the state
 */
template <size_t N> struct State {
    Eigen::Matrix<float, N, 1> mean;
    Eigen::Matrix<float, N, N> covar;
};

/**
 * @brief The weights of the state mean and covariance from a sigma point
 * algorithm.
 * @tparam SIGMA_N The dimensionality of the state
 */
template <size_t SIGMA_N> struct Weights {
    Eigen::Matrix<float, SIGMA_N, 1> mean, covar;
};

} // namespace kalman