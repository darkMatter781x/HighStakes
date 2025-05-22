#include "pf/sensors/v5laser.h"
#include "pros/error.h"
#include "units/units.hpp"
#include <optional>

using namespace sensors;

Laser::Measurement V5Laser::sense() {
  const int dist_mm = m_sensor.get_distance();

  // If sensor in failure state, return nullopt
  if (dist_mm == PROS_ERR) return std::nullopt;
  // If sensor failed to detect object, return MAX_MEASUREMENT
  if (dist_mm == 9999) return {MAX_MEASUREMENT};

  const Length dist = dist_mm * mm;

  return {float(dist.convert(in))};
}

V5Laser::V5Laser(const Laser::Config config, pros::Distance& sensor)
  : Laser(config), m_sensor(sensor) {}
