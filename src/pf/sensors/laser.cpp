#include "pf/sensors/laser.h"
#include "Eigen/src/Geometry/Rotation2D.h"
#include <cmath>

using namespace sensors;

size_t Laser::lastUsedId = 0;

lines::Ray Laser::toRay(const Vec2& robotPos) const {
  const float theta = m_botTheta + m_theta;
  const Eigen::Rotation2Df rot(m_botTheta);

  // const Vec2 start = ;
  // // printf("m_pos: %4.2f,\t%4.2f\n", m_pos.x(), m_pos.y());
  // // printf("theta: %4.2f\n", theta);
  // // printf("hypot: %4.2f\n", hypot);
  // // printf("start: %4.2f,\t%4.2f\n", start.x(), start.y());
  // if (m_id == 0) printf("toRay().theta: %f\n", theta);

  return {robotPos + rot * m_pos, {cos(theta), sin(theta)}};
};

Laser::Laser(const Config config)
  : m_pos {config.pose.x, config.pose.y}, m_theta(config.pose.theta),
    m_botTheta(config.botTheta), obstacles(config.obstacles),
    m_id(Laser::lastUsedId++) {}
