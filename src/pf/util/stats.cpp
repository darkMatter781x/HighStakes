#include "pf/util/stats.h"
#include <cmath>
#include <iostream>
#include <math.h>

namespace stats {
// TODO: seed rng, potentially at compile time
std::mt19937 rng;

// TODO: measure performance impact, and potentially optimize
// coeff and part of exponent could be optimized by computing once for every
// invocation of the pdf with the same config. (use an object)
float norm_pdf(float x, float mu, float sigma) {
  // According to wikipedia: https://en.wikipedia.org/wiki/Normal_distribution
  // f(x) = 1/sqrt(2pi*sigma^2) * e^(-1/2 * ((x-mu)/sigma)^2)
  // f(x) = 1/(sqrt(2pi) * sigma) * (e^(-(1/2) / sigma^2))^((x-mu)^2)
  return 1 / (sqrt(2 * M_PI) * sigma) * exp(-.5 * pow((x - mu) / sigma, 2));
}
} // namespace stats

namespace test {

/**
 * @brief Manually tests MultivariateGaussian's sampling and analysis, comparing
 * it to numpy. Validate printed values with the following python code:
 * @code {.py}
 *  import numpy as np
 *  mu = np.array([7, 5])
 *  sigma = np.array([[2, -3], [-3, 11]])
 *  cpp_samples = <put output samples here>
 *
 *  py_samples = np.random.multivariate_normal(mu, sigma, 500)
 *
 *  print("py mean:\n", np.mean(py_samples, axis=0))
 *  print("py covar:\n", np.cov(py_samples.T))
 *  print("cpp mean:\n", np.mean(cpp_samples, axis=0))
 *  print("cpp covar:\n", np.cov(cpp_samples.T))
 * @endcode
 */
void test_multivariate_gaussian() {
  stats::MultivariateGaussian<2> gaussian {.mu = Eigen::Vector2f {7, 5},
                                           .sigma = Eigen::Matrix2f {}};
  gaussian.sigma << 2, -3, -3, 11;
  auto& result = *new std::array<Eigen::Vector2f, 500>();
  gaussian.makeSampler().sampleMany(result);

  printf("\n[");
  for (auto& sample : result) printf("[%f,%f],", sample.x(), sample.y());
  printf("]\n");

  const auto resultingGaussian =
      stats::MultivariateGaussian<2>::fromSamples(result);
  std::cout << "\nmean:\n" << resultingGaussian.mu << std::endl;
  std::cout << "\ncovar:\n" << resultingGaussian.sigma << std::endl;
}
} // namespace test