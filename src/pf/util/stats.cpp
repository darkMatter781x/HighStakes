#include "pf/util/stats.h"
#include <cmath>
#include <math.h>

// TODO: measure performance impact, and potentially optimize
float norm_pdf(float x, float mu, float sigma) {
  // According to wikipedia: https://en.wikipedia.org/wiki/Normal_distribution
  // f(x) = 1/sqrt(2pi*sigma^2) * e^(-1/2 * ((x-mu)/sigma)^2)

  return 1 / (sqrt(2 * M_PI) * sigma) * exp(-.5 * pow((x - mu) / sigma, 2));
}