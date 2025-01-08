#include "pf/sensors/laser.h"
#include <cmath>

using namespace sensors;

size_t Laser::lastUsedId = 0;

lines::Ray Laser::toRay(const Vec2& robotPos) const {
  const float theta = m_botTheta + m_theta;
  const float hypot = m_pos.norm();
  // const Vec2 start = ;
  // // printf("m_pos: %4.2f,\t%4.2f\n", m_pos.x(), m_pos.y());
  // // printf("theta: %4.2f\n", theta);
  // // printf("hypot: %4.2f\n", hypot);
  // // printf("start: %4.2f,\t%4.2f\n", start.x(), start.y());
  return {robotPos + Vec2(hypot * Vec2 {cos(theta), sin(theta)}),
          {cos(theta), sin(theta)}};
};

Laser::Laser(const Config config)
  : m_pos {config.pose.x, config.pose.y}, m_theta(config.pose.theta),
    m_botTheta(config.botTheta), obstacles(config.obstacles),
    m_id(Laser::lastUsedId++) {}
