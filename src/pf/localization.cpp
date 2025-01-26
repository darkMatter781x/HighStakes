#include "pf/localization.h"
#include "Eigen/src/Core/Array.h"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/util.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "profiler/profiler.h"
#include "util/stats.h"
#include <cstdint>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <fstream>
#include "../liblz4/lz4file.h"

extern "C" void vexSystemExitRequest();
extern "C" int32_t vexControllerGet(int id, int button);

inline static bool isMasterControllerPowerPressed() {
  return vexControllerGet(pros::controller_id_e_t::E_CONTROLLER_MASTER,
                          pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A +
                              1) == true;
}

/** @brief Function to read, increment, and update a value in a file */
int incrementValueInFile(const std::string& filename) {
  // Step 1: Open the file for reading
  std::ifstream inFile(filename);
  if (!inFile) {
    std::cerr << "Error opening file for reading!" << std::endl;
    return -1; // Return -1 to indicate an error
  }

  // Step 2: Read the value from the file
  std::int64_t value;
  inFile >> value;
  inFile.close();

  // Step 3: Increment the value
  value++;

  // Open the file for writing
  std::ofstream outFile(filename);
  if (!outFile) {
    std::cerr << "Error opening file for writing!" << std::endl;
    return -1; // Return -1 to indicate an error
  }

  // Write the incremented value back to the file
  outFile << value;
  outFile.close();

  return value;
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
    m_prevPose(m_odom->getPose()), m_mutex(), m_logFile(nullptr),
    m_compressedLogFile(nullptr) {
  setPose({0, 0, 0}, 10, true, true);
  std::int64_t index = incrementValueInFile("/usd/localization.index");
  printf("localization index: %lli", index);
  // if no errors incrementing value
  if (index != -1) {
    std::string logFileNameStr =
        "/usd/localization." + std::to_string(index) + ".log";
    const char* logFileName = logFileNameStr.c_str();
    std::string compressedLogFileNameStr =
        "/usd/localization." + std::to_string(index) + ".log.lz4";
    const char* compressedLogFileName = compressedLogFileNameStr.c_str();

    m_logFile = fopen(logFileName, "w");
    m_compressedLogFile = fopen(compressedLogFileName, "w");
  }

  isInitialized = true;
}

template <typename T, int N>
void writeLZ4Data(LZ4_writeFile_t* lz4File, const Eigen::Array<T, N, 1>& data) {
  size_t compressedSize =
      LZ4F_write(lz4File, reinterpret_cast<const char*>(data.data()),
                 sizeof(T) * data.size());
  if (LZ4F_isError(compressedSize)) {
    LZ4F_writeClose(lz4File);
    throw std::runtime_error("Failed to compress and write data!");
  }
}

static LZ4_writeFile_t* lz4File;

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

  // sd card is attached
  if (m_logFile != NULL) {
    if (lz4File == nullptr) {
      LZ4F_errorCode_t maybeErr =
          LZ4F_writeOpen(&lz4File, m_compressedLogFile, nullptr);
      if (LZ4F_isError(maybeErr)) {
        throw std::runtime_error(
            "Failed to initialize LZ4 compression context!");
      }
    }

    const auto time = Eigen::Array<uint64_t, 1, 1> {pros::micros()};
    writeLZ4Data(lz4File, time);
    // fwrite(&time, sizeof(time), 1, m_logFile);
    writeLZ4Data(lz4File, *data.particles);
    // fwrite(&*data.particles, sizeof(*data.particles), 1, m_logFile);
    writeLZ4Data(lz4File, *data.weights);
    // fwrite(&*data.weights, sizeof(*data.weights), 1, m_logFile);
  }

  return data;
}

// TODO:  Identify performance bottlenecks and resolve them (PROFILING!?)
//        (Each epoch takes ~14ms, and we want <5ms)
void Localization::update() {
  // Don't update if the odom hasn't been calibrated
  if (!m_odom->hasCalibrated()) return;

  static bool done = false;
  if (done) return;

  // TODO: Extract timing code to a Timer class
  static size_t iteration = 0;
  START_PROFILE("LL.update()") // Starts profiled section
  m_mutex.take();

  m_odom->update();

  // Only use distance sensors after match has started
  // (Humans may be on the field before the match starts, interfering with the
  // distance sensors)
  if (!pros::competition::is_disabled()) {
    lemlib::Pose displacement = m_odom->getPose() - m_prevPose;
    m_predictor.m_displacement = {displacement.x, displacement.y};

    PROFILE_SECTION("PF.predict", { m_pf.predict(m_predictor); })

    PROFILE_SECTION("PF.update", { m_pf.update(m_sensors, m_resampler); })

    const auto mean = m_pf.getMean();

    m_odom->setPose({mean.x(), mean.y(), m_odom->getPose().theta});

    // TODO:  Try denser telemetry. eg: send binary representation of floats
    //        rather than strings.
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
  Profiler::get().printReport();
#endif

  if (isMasterControllerPowerPressed()) {
    printf("[ProfSummary.Start]\n"); // Used by time script to identify which
                                     // data to put in log file
#ifdef ENABLE_PROFILING
    Profiler::get().printReport(false, (char*)"");
#endif
    printf("[ProfSummary.End]\n");

    pros::delay(1000);
    done = true;

    // If sd card is attached
    if (m_logFile != NULL) {
      LZ4F_writeClose(lz4File);
      fclose(m_logFile);
      fclose(m_compressedLogFile);
    }
    // vexSystemExitRequest();
  }
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