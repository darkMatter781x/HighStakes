#pragma once

#include "Eigen/Dense"

namespace kalman {

/**
 * @brief Compute the cross-covariance matrix between two sets of points.
 *
 * @tparam N_A Dimensionality of each A point.
 * @tparam N_B Dimensionality of each B point.
 * @tparam M Number of points.
 * @param aPts Points in A space.
 * @param bPts Points in B space.
 * @param aMean Mean of aPts.
 * @param bMean Mean of bPts.
 * @param weights Weights for each point pair.
 * @return Eigen::Matrix<float, N_A, N_B> th
 */
template <size_t N_A, size_t N_B, size_t M> Eigen::Matrix<float, N_A, N_B>
crossCovariance(const Eigen::Matrix<float, N_A, M>& aPts,
                const Eigen::Matrix<float, N_B, M>& bPts,
                const Eigen::Vector<float, N_A>& aMean,
                const Eigen::Vector<float, N_B>& bMean,
                const Eigen::Vector<float, M>& weights) {
  Eigen::Matrix<float, N_A, N_B> crossCovar;
  for (size_t i = 0; i < M; i++) {
    crossCovar +=
        weights(i) * (aPts.col(i) - aMean) * (bPts.col(i) - bMean).transpose();
  }
  return crossCovar;
}

} // namespace kalman