#include "pf/sensors/laser.h"

using namespace sensors;

lines::Ray Laser::toRay(const Vec2& robotPos) const {
  const float theta = m_botTheta + m_theta;
  const float hypot = m_pos.norm();
  return {robotPos + hypot * Vec2 {cos(m_botTheta), sin(m_botTheta)},
          {cos(theta), sin(theta)}};
};

float Laser::expectedDist(const Vec2& robotPos) const {
  return toRay(robotPos).findFirstIntersect(obstacles).value().first;
};

Laser::Laser(const Config config)
  : m_pos {config.pose.x, config.pose.y}, m_theta(config.pose.theta),
    m_botTheta(config.botTheta), obstacles(config.obstacles) {}
