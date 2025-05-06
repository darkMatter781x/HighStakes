#include "pf/localization.h"
#include "Eigen/src/Core/Array.h"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/util.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "profiler/profiler.h"
#include "util/logger.h"
#include "util/stats.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ostream>

extern "C" void vexSystemExitRequest();
extern "C" int32_t vexControllerGet(int id, int button);

inline static bool isMasterControllerPowerPressed() {
  return vexControllerGet(pros::controller_id_e_t::E_CONTROLLER_MASTER,
                          pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A +
                              1) == true;
}

/** Uniform distribution for resampling */
float uniform_float_0_to_1() {
  static std::uniform_real_distribution<float> dist {0, 1};
  return dist(stats::rng);
}

template <size_t N, size_t M>
std::unique_ptr<Eigen::Array<Eigen::Vector<float, M>, N, 1>> makeZeroArray() {
  auto arr = std::make_unique<Eigen::Array<Eigen::Vector<float, M>, N, 1>>();
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

Localization::PF::Data
Localization::printParticles(Localization::PF::Data data) const {
  // Print live telemetry
  if (false) {
    printf("[TELEMETRY.PF.PARTICLES]:%llu;", pros::micros());
    std::discrete_distribution<> dist(data.weights->begin(),
                                      data.weights->end());
    for (size_t i = 0; i < 25; i++) {
      int j = dist(stats::rng); // Sample an index
      printf("%f,%f,%f;", (*data.particles)[j].x(), (*data.particles)[j].y(),
             (*data.weights)[j]);
    }
    printf("\n");
  }
  logger::recordParticles(data, "particles");

  return data;
}

// TODO:  Identify performance bottlenecks and resolve them (PROFILING!?)
//        (Each epoch takes ~14ms, and we want <5ms)
void Localization::update() {
  // Don't update if the odom hasn't been calibrated
  if (!m_odom->hasCalibrated()) return;

  static bool done = false;
  if (done) return;

  // Sometimes the pose becomes NaN and idk why. This is an attempt to cope
  if (std::isnan(this->m_odom->getPose().x) ||
      std::isnan(this->m_odom->getPose().y)) {
    printf("\x1b[1;31m"
           "Localization: NaN-ed"
           "\x1b[0m");

    // Attempt to recover by globally localizing
    this->setPose({0, 0, this->m_odom->getPose().theta}, 64);
  }

  static size_t iteration = 0;
  static size_t lastStartTime = pros::millis();
  START_PROFILE("LL.update()") // Starts profiled section
  m_mutex.take();

  if (pros::millis() - lastStartTime > 20)
    printf("\x1b[1;35m"
           "Time jump:"
           "\x1b[0m"
           "\t%5i -> %5i\tDIFF: %5i\n",
           lastStartTime, pros::millis(), pros::millis() - lastStartTime);
  lastStartTime = pros::millis();

  m_odom->update();

  // Only use distance sensors after match has started
  // (Humans may be on the field before the match starts, interfering with the
  // distance sensors)
  if (this->enabled) {
    lemlib::Pose displacement = m_odom->getPose() - m_prevPose;
    m_predictor.m_displacement = {displacement.x, displacement.y};

    PROFILE_SECTION("PF.predict", { m_pf.predict(m_predictor); })

    PROFILE_SECTION("PF.update", { m_pf.update(m_sensors, m_resampler); })

    const auto mean = m_pf.getMean();

    m_odom->setPose({mean.x(), mean.y(), m_odom->getPose().theta});

    PROFILE_SECTION("PF.printParticles", {
      m_pf.modify([this](auto arg) -> auto {
        return this->printParticles(std::move(arg));
      });
    })
  }

  m_prevPose = m_odom->getPose();
  m_mutex.give();

  END_PROFILE() // Ends profiled section

  iteration++;
#ifdef ENABLE_PROFILING
  // Profiler::get().printReport();
#endif
  // if (pros::millis() % 100 < 10)
  // printf("Pose:\t(%03.3fin,\t%02.3fin,\t%03.2frad)\n", m_prevPose.x,
  //        m_prevPose.y, m_prevPose.theta);

  //   if (isMasterControllerPowerPressed() || iteration == 1000) {
  //     printf("[ProfSummary.Start]\n"); // Used by time script to identify
  //     which
  //                                      // data to put in log file
  // #ifdef ENABLE_PROFILING
  //     Profiler::get().printReport(false, (char*)"");
  // #endif
  //     printf("[ProfSummary.End]\n");

  //     pros::delay(1000);
  //     done = true;

  //     // // If sd card is attached
  //     // if (m_compressedLogFile != NULL) {
  //     //   LZ4F_writeClose(lz4File);
  //     //   fclose(m_compressedLogFile);
  //     // }
  //     // vexSystemExitRequest();
  //   }
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
    gaussian.makeSampler().sampleMany<N>(*(data.particles));
    return data;
  });
  if (isInitialized)
    m_pf.modify([this](auto arg) -> auto {
      return this->printParticles(std::move(arg));
    });
  m_mutex.give();
}

void Localization::calibrate() { m_odom->calibrate(); }

void Localization::OdomPredictor::predict(Eigen::Array<P, N, 1>& particles) {
  // const auto rotated_covar = covar. ;
  auto noiseSampler =
      stats::MultivariateGaussian<2> {.mu = {0, 0}, .sigma = m_covar}
          .makeSampler();

  Eigen::Vector<float, 2> noise;
  constexpr float fieldRadius = 140.25 / 2;
  for (auto& particle : particles) {
    noiseSampler.sampleOnce(noise);
    particle += m_displacement + noise;
    particle = {std::max(std::min(fieldRadius, particle.x()), -fieldRadius),
                std::max(std::min(fieldRadius, particle.y()), -fieldRadius)};
  }
}

Localization::OdomPredictor::OdomPredictor(Eigen::Matrix2f covar)
  : m_covar(covar), m_displacement({0, 0}) {}