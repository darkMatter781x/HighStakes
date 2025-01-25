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

// TODO: Document
// TODO: Move impl to .cpp
class NormPDF {
  private:
    const float m_mu;
    const float m_sigma;
    const float m_coeff;
  public:
    NormPDF(float mu, float sigma)
      : m_mu(mu), m_sigma(sigma), m_coeff(1 / sqrt(2 * M_PI) * m_sigma) {};

    inline float eval(float x) const {
      return m_coeff * exp(-.5 * pow((x - m_mu) / m_sigma, 2));
    }
};

template <size_t N> struct MultivariateGaussian {
    Eigen::Vector<float, N> mu;
    Eigen::Matrix<float, N, N> sigma;

    class Sampler {
      private:
        const MultivariateGaussian m_dist;
        std::normal_distribution<float> m_rngDist;
        const Eigen::Matrix<float, N, N> L;
        Eigen::Vector<float, N> z;

        friend MultivariateGaussian;

        Sampler(const MultivariateGaussian& dist)
          : m_dist(dist), m_rngDist {0, 1}, L(m_dist.sigma.llt().matrixL()) {}
      public:
        /**
         * @brief Samples once from the distribution.
         *
         * @param result Where to store the sample
         */
        void sampleOnce(Eigen::Vector<float, N>& result) {
          for (size_t i = 0; i < N; i++) { z(i) = m_rngDist(rng); }
          result = m_dist.mu + L * z;
        }

        /**
         * @brief Samples many times from the distribution.
         *
         * @tparam M Number of samples
         * @param result Where to store the samples
         */
        template <size_t M>
        void sampleMany(Eigen::Array<Eigen::Vector<float, N>, M, 1>& result) {
          for (auto& sample : result) { sampleOnce(sample); }
        }
    };

    Sampler makeSampler() const { return Sampler(*this); }

    /**
     * @brief Calculates the mean of the samples.
     *
     * @tparam M Number of samples
     * @param samples Array of samples
     * @return Eigen::Vector<float, N> Mean of the samples
     *
     * @time ignoring N: O(M)
     */
    template <size_t M> static Eigen::Vector<float, N>
    calcMuFromSamples(const Eigen::Array<Eigen::Vector<float, N>, M, 1>& samples) {
      Eigen::Vector<float, N> mu = Eigen::Vector<float, N>::Zero();
      for (const auto& sample : samples) mu += sample / M;
      // mu /= M;
      return mu;
    }

    /**
     * @brief Calculates the properties of the samples, assuming they are
     * gaussian.
     *
     * @tparam M Number of samples
     * @param samples Array of samples
     * @return MultivariateGaussian representation of the samples
     *
     * @time ignoring N: O(M)
     */
    template <size_t M> static MultivariateGaussian<N>
    fromSamples(const Eigen::Array<Eigen::Vector<float, N>, M, 1>& samples) {
      // TODO: investigate performance and numerical stability, improving it if
      // necessary
      Eigen::Vector<float, N> mu = calcMuFromSamples(samples);

      Eigen::Matrix<float, N, N> sigma = Eigen::Matrix<float, N, N>::Zero();
      for (const auto& sample : samples) {
        Eigen::Vector<float, N> diff = sample - mu;
        sigma += diff * diff.transpose() / M;
      }
      // sigma /= M;

      return {.mu = mu, .sigma = sigma};
    }
};

}; // namespace stats

namespace test {
void test_multivariate_gaussian();
}