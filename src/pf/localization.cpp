#include "pf/localization.h"
#include "Eigen/src/Core/Matrix.h"
#include "pf/sensors/v5laser.h"
#include "pros/misc.hpp"

Localization::Localization(const Config& config)
  : m_pf({Eigen::Vector2f::Zero()}),
    m_odom(config.odomSensors, config.drivetrain), m_sensors(),
    m_obstacles(config.obstacles),
    // TODO: Implement randomFunc
    m_resampler([]() -> float { return 1.; }),
    m_predictor(
        config.covar,
        // TODO: Implement randomFunc
        [](Eigen::Matrix2f& covar) -> auto { return Eigen::Vector2f::Zero(); }),
    prevPose(m_odom.getPose()) {
  for (const auto& dConf : config.distances)
    m_sensors.push_back(std::make_unique<sensors::V5LaserSensor<N>>(
        sensors::V5Laser::Config {
            .pose = dConf.pose,
            .botTheta = m_odom.getPose().theta,
            .obstacles = m_obstacles,
        },
        dConf.sensor));
}

void Localization::update() {
  m_odom.update();

  // Only use distance sensors after match has started
  if (!pros::competition::is_disabled()) {
    lemlib::Pose displacement = m_odom.getPose() - prevPose;
    m_predictor.m_displacement = {displacement.x, displacement.y};
    m_pf.predict(m_predictor);

    m_pf.update(m_sensors, m_resampler);
    const auto mean = m_pf.getMean();

    m_odom.setPose({mean.x(), mean.y(), m_odom.getPose().theta});
  }
  prevPose = m_odom.getPose();
}

void Localization::setPose(lemlib::Pose pose, float stdDev) {
  prevPose = pose;
  // TODO: Implement
}

void Localization::OdomPredictor::predict(std::array<P, N>& particles) {
  for (size_t i = 0; i < N; i++) {
    particles[i] += m_displacement + randomFunc(m_covar);
  }
}

Localization::OdomPredictor::OdomPredictor(Eigen::Matrix2f covar,
                                           RandomFunc randomFunc)
  : m_covar(covar), m_displacement({0, 0}), randomFunc(randomFunc) {}