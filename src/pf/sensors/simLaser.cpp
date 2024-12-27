#include "pf/sensors/simLaser.h"
#include "pf/util/stats.h"
#include <random>

using namespace sensors;

std::normal_distribution<float> distNoiseLaser(0, 1);

Laser::Measurement SimLaser::sense() {
  // TODO: Improve faking with noise
  auto ray = toRay({m_realPose.x, m_realPose.y});
  auto intersect = ray.findFirstIntersect(obstacles);
  if (!intersect) return {0, 3};
  float dist = intersect.value().first;
  // printf("[TELEMETRY.PF.LASERS]:%i,%f,%f,%f,%f\n", m_id, ray.start.x(),
  //        ray.start.y(), ray.start.x() + dist * ray.direction.x(),
  //        ray.start.y() + dist * ray.direction.y());
  const float theta = m_botTheta + m_theta;
  const float stdDev = dist * .05f;
  dist += stdDev * distNoiseLaser(stats::rng);
  return {dist, stdDev};
}

SimLaser::SimLaser(const Laser::Config config, const lemlib::Pose& botPose)
  : Laser(config), m_realPose(botPose) {}
