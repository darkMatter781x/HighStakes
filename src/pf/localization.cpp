#include "pf/localization.h"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/util.hpp"
#include "pf/sensors/v5laser.h"
#include "pros/misc.hpp"
#include "util/stats.h"

/** Uniform distribution for resampling */
float uniform_float_0_to_1() {
  static std::uniform_real_distribution<float> dist {0, 1};
  return dist(stats::rng);
}

Localization::Localization(const Config& config)
  : m_pf({Eigen::Vector2f::Zero()}),
    m_odom(config.odomSensors, config.drivetrain), m_sensors(),
    m_obstacles(config.obstacles), m_resampler(uniform_float_0_to_1),
    m_predictor(config.covar), m_prevPose(m_odom.getPose()), m_mutex() {
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
  m_mutex.take();

  m_odom.update();

  // Only use distance sensors after match has started
  if (!pros::competition::is_disabled()) {
    lemlib::Pose displacement = m_odom.getPose() - m_prevPose;
    m_predictor.m_displacement = {displacement.x, displacement.y};
    m_pf.predict(m_predictor);

    m_pf.update(m_sensors, m_resampler);
    const auto mean = m_pf.getMean();

    m_odom.setPose({mean.x(), mean.y(), m_odom.getPose().theta});
  }
  m_prevPose = m_odom.getPose();
  m_mutex.give();
}

void Localization::setPose(lemlib::Pose pose, float stdDev, bool radians,
                           bool standardPos) {
  m_mutex.take();
  if (!standardPos) pose.theta = M_PI_2 - pose.theta;
  if (!radians) pose.theta = lemlib::radToDeg(pose.theta);

  m_prevPose = pose;
  m_odom.setPose(pose);

  // TODO: Test
  stats::MultivariateGaussian<2> gaussian {
      .mu = {pose.x, pose.y},
      .sigma = Eigen::Matrix2f::Identity() * stdDev,
  };

  // Updates particles to be centered around the new pose
  m_pf.modifyParticles(
      [&gaussian](auto& particles) { gaussian.sampleMany<N>(particles); });
  m_mutex.give();
}

void Localization::OdomPredictor::predict(std::array<P, N>& particles) {
  // TODO: deduplicate with code in stats::MultivariateGaussian::sampleMany()
  auto dist = std::normal_distribution<float>(0, 1);
  Eigen::Vector<float, 2> z;
  Eigen::Matrix<float, 2, 2> L = m_covar.llt().matrixL();

  for (int i = 0; i < 2; i++) { z(i) = dist(stats::rng); }

  for (auto& particle : particles) { particle += m_displacement + L * z; }
}

Localization::OdomPredictor::OdomPredictor(Eigen::Matrix2f covar)
  : m_covar(covar), m_displacement({0, 0}) {}