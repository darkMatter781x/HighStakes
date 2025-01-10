#include "pf/localization.h"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/util.hpp"
#include "pf/sensors/v5laser.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "profiler/profiler.h"
#include "util/stats.h"

/** Uniform distribution for resampling */
float uniform_float_0_to_1() {
  static std::uniform_real_distribution<float> dist {0, 1};
  return dist(stats::rng);
}

template <size_t N, size_t M>
std::unique_ptr<std::array<Eigen::Vector<float, M>, N>> makeZeroArray() {
  auto arr = std::make_unique<std::array<Eigen::Vector<float, M>, N>>();
  std::fill(arr->begin(), arr->end(), Eigen::Vector<float, M>::Zero());
  return arr;
}

static bool isInitialized = false;

Localization::Localization(const Config& config)
  : m_pf(makeZeroArray<N, 2>()), m_odom(config.odom),
    m_obstacles(config.obstacles),
    m_sensors(
        config.sensorFactory(SensorFactoryConfig {.botPose = m_odom->getPose(),
                                                  .obstacles = m_obstacles})),
    m_resampler(uniform_float_0_to_1), m_predictor(config.covar),
    m_prevPose(m_odom->getPose()), m_mutex() {
  setPose({0, 0, 0}, 10, true, true);
  isInitialized = true;
}

static Localization::PF::Data printParticles(Localization::PF::Data data) {
  return data;
  size_t i = 0;
  printf("[TELEMETRY.PF.PARTICLES]:%llu;", pros::micros());
  for (; i < Localization::N; i++) {
    printf("%f,%f,%f;", (*data.particles)[i].x(), (*data.particles)[i].y(),
           (*data.weights)[i]);
  }
  printf("\n");
  return data;
}

// TODO:  Identify performance bottlenecks and resolve them (PROFILING!?)
//        (Each epoch takes ~14ms, and we want <5ms)
void Localization::update() {
  // Don't update if the odom hasn't been calibrated
  if (!m_odom->hasCalibrated()) return;
  // TODO: Extract timing code to a Timer class
  static size_t iteration = 0;
  START_PROFILE("LL.update()") // Starts profiled section
  m_mutex.take();

  m_odom->update();

  // // Only use distance sensors after match has started
  // // (Humans may be on the field before the match starts, interfering with the
  // // distance sensors)
  // if (!pros::competition::is_disabled()) {
  //   lemlib::Pose displacement = m_odom->getPose() - m_prevPose;
  //   m_predictor.m_displacement = {displacement.x, displacement.y};

  //   PROFILE_SECTION("PF.predict", { m_pf.predict(m_predictor); })

  //   PROFILE_SECTION("PF.update", { m_pf.update(m_sensors, m_resampler); })

  //   const auto mean = m_pf.getMean();

  //   m_odom->setPose({mean.x(), mean.y(), m_odom->getPose().theta});

  //   // TODO:  Try denser telemetry. eg: send binary representation of floats
  //   //        rather than strings.
  //   // PROFILE_SECTION("PF.printParticles", { m_pf.modify(printParticles); })
  // }

  m_prevPose = m_odom->getPose();
  m_mutex.give();

  END_PROFILE() // Ends profiled section

  iteration++;
#ifdef ENABLE_PROFILING
  Profiler::get().printReport();
  if (iteration == 1000) {
    printf("[ProfSummary.Start]\n"); // Used by time script to identify which
                                     // data to put in log file
    Profiler::get().printReport(false, (char*)"");
    printf("[ProfSummary.End]\n");

    pros::delay(1000);
    std::abort();
  }
#endif

  // printf("[TELEMETRY.PF.CALC_POSE]:%f,%f,%f\n", m_odom->getPose().x,
  //        m_odom->getPose().y, m_odom->getPose().theta);
}

void Localization::setPose(lemlib::Pose pose, float stdDev, bool radians,
                           bool standardPos) {
  m_mutex.take();
  if (!radians) pose.theta = lemlib::degToRad(pose.theta);
  if (!standardPos) pose.theta = M_PI_2 - pose.theta;

  m_prevPose = pose;
  m_odom->setPose(pose);

  stats::MultivariateGaussian<2> gaussian {
      .mu = {pose.x, pose.y},
      .sigma = Eigen::Matrix2f::Identity() * stdDev,
  };

  // Updates particles to be centered around the new pose
  m_pf.modify([&gaussian](auto data) {
    gaussian.makeSampler().sampleMany(*(data.particles));
    return data;
  });
  if (isInitialized) m_pf.modify(printParticles);
  m_mutex.give();
}

void Localization::calibrate() { m_odom->calibrate(); }

void Localization::OdomPredictor::predict(std::array<P, N>& particles) {
  auto noiseSampler =
      stats::MultivariateGaussian<2> {.mu = {0, 0}, .sigma = m_covar}
          .makeSampler();

  Eigen::Vector<float, 2> noise;
  for (auto& particle : particles) {
    noiseSampler.sampleOnce(noise);
    particle += m_displacement + noise;
  }
}

Localization::OdomPredictor::OdomPredictor(Eigen::Matrix2f covar)
  : m_covar(covar), m_displacement({0, 0}) {}