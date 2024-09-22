#pragma once

#include "kalman/types.h"

namespace kalman {

/**
 * @brief Generates sigma points for the unscented Kalman filter, using
 * Van de Merwe's scaled sigma points algorithm.
 *
 * @tparam N The dimensionality of the state
 */
template <size_t N> class MerweScaledSigmaPoints {
  public:
    /** @brief The number of produced sigma points. */
    static constexpr size_t SIGMA_N = 2 * N + 1;

    /** @brief The input parameters that control the distribution of the sigma
     * points, and how to perform sqrt. */
    struct Config {
        /** @brief Controls distribution of sigma points. */
        float alpha, beta, kappa;
        /** @brief Function to perform sqrt. */
        std::function<Eigen::Matrix<float, N, N>(
            const Eigen::Matrix<float, N, N>&)>
            sqrt;
    };

    /** @brief The input parameters that control the distribution of the sigma
     * points, and how to perform sqrt. */
    const Config m_conf;

    /** @brief The weights for the sigma points. */
    struct Weights : kalman::Weights<SIGMA_N> {
        /** @brief creates the weights for the sigma points from the provided
         * config */
        static Weights fromConfig(const Config& conf) {
          Weights w;
          const float lambda = conf.alpha * conf.alpha * (N + conf.kappa) - N;
          w.mean = w.covar = Eigen::Matrix<float, SIGMA_N, 1>::Constant(
              1 / (2 * (N + lambda)));
          w.mean(0) = w.covar(0) = lambda / (N + lambda);
          w.covar(0) += 1 - conf.alpha * conf.alpha + conf.beta;
          return w;
        }
    };

    /** @brief Produces sigma points centered around the mean according to
     * Van de Merwe's scaled sigma points algorithm.  */
    Eigen::Matrix<float, N, SIGMA_N>
    computeSigmaPoints(const Eigen::Matrix<float, N, 1>& mean,
                       const Eigen::Matrix<float, N, N>& covar) const {
      Eigen::Matrix<float, N, SIGMA_N> sigmaPoints;
      sigmaPoints.col(0) = mean;

      const float lambda = m_conf.alpha * m_conf.alpha * (N + m_conf.kappa) - N;
      const Eigen::Matrix<float, N, N> U = m_conf.sqrt((lambda + N) * covar);

      for (size_t i = 0; i < N; i++) {
        sigmaPoints.col(i + 1) = mean + U.col(i);
        sigmaPoints.col(i + N + 1) = mean - U.col(i);
      }
      return sigmaPoints;
    }

    /** @brief The weights for the sigma points. */
    const Weights m_weights;

    /** @brief Constructs a new MerweScaledSigmaPoints object with the given
     * configuration. */
    MerweScaledSigmaPoints(Config config)
      : m_conf(config), m_weights(Weights::fromConfig(config)) {};
};

template <size_t N> using MerweSigmaPts = MerweScaledSigmaPoints<N>;

} // namespace kalman