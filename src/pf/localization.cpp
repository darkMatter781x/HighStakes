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

template <size_t N, size_t M>
std::array<Eigen::Vector<float, M>, N> makeZeroArray() {
  std::array<Eigen::Vector<float, M>, N> arr;
  std::fill(arr.begin(), arr.end(), Eigen::Vector<float, M>::Zero());
  return arr;
}

Localization::Localization(const Config& config)
  : m_pf(makeZeroArray<N, 2>()), m_odom(config.odomSensors, config.drivetrain),
    m_sensors(), m_obstacles(config.obstacles),
    m_resampler(uniform_float_0_to_1), m_predictor(config.covar),
    m_prevPose(m_odom.getPose()), m_mutex() {
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
  // Don't update if the odom hasn't been calibrated
  if (!m_odom.hasCalibrated()) return;

  m_mutex.take();

  m_odom.update();

  // Only use distance sensors after match has started
  // (Humans may be on the field before the match starts, interfering with the
  // distance sensors)
  if (!pros::competition::is_disabled()) {
    lemlib::Pose displacement = m_odom.getPose() - m_prevPose;
    m_predictor.m_displacement = {displacement.x, displacement.y};
    m_pf.predict(m_predictor);

    m_pf.update(m_sensors, m_resampler);
    const auto mean = m_pf.getMean();

    m_odom.setPose({mean.x(), mean.y(), m_odom.getPose().theta});
  }

  m_prevPose = m_odom.getPose();
  printf("pose: %s \n", format_as(m_odom.getPose()).c_str());
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

void Localization::calibrate() { m_odom.calibrate(); }

void Localization::OdomPredictor::predict(std::array<P, N>& particles) {
  // TODO: deduplicate with code in stats::MultivariateGaussian::sampleMany()
  auto dist = std::normal_distribution<float>(0, 1);
  Eigen::Matrix<float, 2, 2> L = m_covar.llt().matrixL();
  Eigen::Vector<float, 2> z;

  for (auto& particle : particles) {
    for (int i = 0; i < 2; i++) { z(i) = dist(stats::rng); }

    particle += m_displacement + L * z;
    if (std::isnan(particle(0)) || std::isnan(particle(1)))
      printf("particle is nan!\n");
  }
}

Localization::OdomPredictor::OdomPredictor(Eigen::Matrix2f covar)
  : m_covar(covar), m_displacement({0, 0}) {}