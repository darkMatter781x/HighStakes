#pragma once
#include "Eigen/Core"

namespace sensors {

/**
 * @brief Updates weights with measurement.
 * @tparam P Particle type. Must be addable by P, and multipliable by float.
 * @tparam N Number of particles.
 */
template <typename P, size_t N> struct Sensor {
    /**
     * @brief Modifies weights param based on measurement.
     * Does not need to normalize weights.
     */
    virtual void update(const Eigen::Array<P, N, 1>& particles,
                        Eigen::Array<float, N, 1>& weights) = 0;
};

} // namespace sensors