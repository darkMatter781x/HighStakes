#include "pf/sensors/laser.h"

using namespace sensors;

lines::Ray Laser::toRay(const Vec2& robotPos) const {
  const float theta = m_botTheta + m_theta;
  return {robotPos + m_pos};
};

float Laser::expectedDist(const Vec2& robotPos) const {
  return toRay(robotPos).findFirstIntersect(obstacles).value().first;
};

Laser::Laser(const Config config)
  : m_pos {config.pose.x, config.pose.y}, m_theta(config.pose.theta),
    m_botTheta(config.botTheta), obstacles(config.obstacles) {}
