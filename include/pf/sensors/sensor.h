#pragma once
#include "lemlib/pose.hpp"
#include <array>

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
    virtual void update(const std::array<P, N>& particles,
                        std::array<float, N>& weights) = 0;
};

} // namespace sensors