#pragma once

#include "Eigen/Dense"
#include <random>

namespace stats {

// TODO: guard with mutex
/** global random number generator */
extern std::mt19937 rng;

/**
 * @brief Evaluates the pdf of the gaussian distribution at x.
 * @param mu = mean
 * @param sigma = standard deviation
 */
float norm_pdf(float x, float mu, float sigma);

template <size_t N> struct MultivariateGaussian {
    Eigen::Vector<float, N> mu;
    Eigen::Matrix<float, N, N> sigma;

    /**
     * @brief Samples many times from the distribution.
     *
     * @tparam M Number of samples
     * @param result Where to store the samples
     */
    template <size_t M>
    void sampleMany(std::array<Eigen::Vector<float, N>, M>& result) {
      // TODO: deduplicate with code in Localization::OdomPredictor::predict()
      auto dist = std::normal_distribution<float>(0, 1);
      Eigen::Vector<float, N> z;
      Eigen::Matrix<float, N, N> L = sigma.llt().matrixL();

      for (int i = 0; i < N; i++) { z(i) = dist(rng); }

      for (size_t i = 0; i < M; i++) { result[i] = mu + L * z; }
    }
};

}; // namespace stats