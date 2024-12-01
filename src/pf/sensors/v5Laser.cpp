#include "pf/sensors/v5laser.h"

using namespace sensors;

Laser::Measurement V5Laser::sense() {
  const float dist = m_sensor.get_distance();
  // TODO: determine if this is the correct way to get the standard deviation
  const float stdDev = m_sensor.get_confidence();
  return {dist, stdDev};
}

V5Laser::V5Laser(const Laser::Config config, pros::Distance& sensor)
  : Laser(config), m_sensor(sensor) {}
