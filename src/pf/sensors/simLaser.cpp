#include "pf/sensors/simLaser.h"

using namespace sensors;

Laser::Measurement SimLaser::sense() {
  // TODO: Improve faking
  auto intersect =
      toRay({m_realPose.x, m_realPose.y}).findFirstIntersect(obstacles);
  if (!intersect) return {0, 3};
  return {intersect.value().first, 3};
}

SimLaser::SimLaser(const Laser::Config config, const lemlib::Pose& botPose)
  : Laser(config), m_realPose(botPose) {}
