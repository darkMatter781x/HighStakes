#include "pf/util/stats.h"
#include <cmath>
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