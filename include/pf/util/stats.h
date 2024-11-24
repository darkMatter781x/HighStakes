#pragma once

namespace stats {

/**
 * @brief Evaluates the pdf of the gaussian distribution at x.
 * @param mu = mean
 * @param sigma = standard deviation
 */
float norm_pdf(float x, float mu, float sigma);

}; // namespace stats