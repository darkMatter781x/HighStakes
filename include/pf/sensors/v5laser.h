#pragma once

#include "pf/sensors/laser.h"
#include "pros/distance.hpp"

namespace sensors {

/**
 * @brief Implements the laser sensor for the V5 laser sensor.
 */
class V5Laser : public Laser {
  private:
    pros::Distance& m_sensor;

    template <size_t N> friend struct V5LaserSensor;

    V5Laser(const Laser::Config config, pros::Distance& sensor);
  public:
    Measurement sense() override;
};

/**
 * @brief Implements the PF sensor for the V5 laser sensor.
 */
template <size_t N> struct V5LaserSensor : public Laser::Sensor<N> {
    V5Laser m_laser;

    V5LaserSensor(const Laser::Config config, pros::Distance& sensor)
      : m_laser(config, sensor), Laser::Sensor<N>(m_laser) {}
  private:
    using Laser::Sensor<N>::laser;
};
}; // namespace sensors