#pragma once

#include "lemlib/pose.hpp"
#include "pf/sensors/laser.h"

namespace sensors {

/**
 * @brief Simulates the laser sensor.
 */
class SimLaser : public Laser {
  private:
    /** Actual pose simulated robot is at, not the computed pose */
    const lemlib::Pose& m_realPose;

    template <size_t N> friend struct SimLaserSensor;

    SimLaser(const Laser::Config config, const lemlib::Pose& botPose);
  public:
    Measurement sense() override;
};

/**
 * @brief Implements the PF sensor for the simulated laser sensor.
 */
template <size_t N> struct SimLaserSensor : public Laser::Sensor<N> {
    SimLaser m_laser;

    SimLaserSensor(const Laser::Config config, const lemlib::Pose& botPose)
      : m_laser(config, botPose), Laser::Sensor<N>(m_laser) {}
  private:
    using Laser::Sensor<N>::laser;
};
}; // namespace sensors