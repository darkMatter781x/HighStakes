#include "pf/sensors/v5laser.h"
#include "pros/error.h"
#include "units/units.hpp"
#include <optional>

using namespace sensors;

std::optional<Laser::Measurement> V5Laser::sense() {
  const int dist_mm = m_sensor.get_distance();
  if (dist_mm == 9999 || dist_mm == PROS_ERR) return std::nullopt;
  const Length dist = dist_mm * mm;
  // const int size = m_sensor.get_object_size();
  // // Likely hitting non wall
  // if (size < 60) return std::nullopt;
  // printf("dist: %fmm\n", dist_mm);
  // printf("dist: %fin\n", dist);
  // printf("size: %i\n", size);
  // printf("confidence: %i\n", confidence);
  return {{float(dist.convert(in)), float(dist.convert(in)) * 0.1f}};
}

V5Laser::V5Laser(const Laser::Config config, pros::Distance& sensor)
  : Laser(config), m_sensor(sensor) {}
