#include "pf/lemlibOdom.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/logger/logger.hpp"
#include <cmath>

const lemlib::Pose& LemLibOdom::getPose() const { return m_pose; }

lemlib::Pose& LemLibOdom::getPose() { return m_pose; }

void LemLibOdom::setPose(const lemlib::Pose& pose) { lemlib::setPose(pose); }

void LemLibOdom::update() {
  lemlib::update();
  m_pose = lemlib::getPose();
}

// The following code is from lemlib/chassis/chassis.cpp:
// https://github.com/LemLib/LemLib/blob/fb222dbdf250925ccffc79cf3ee46656a8c80faa/src/lemlib/chassis/chassis.cpp#L44C1-L93C2
// Which is licensed under the MIT License found at:
// https://github.com/LemLib/LemLib/blob/fb222dbdf250925ccffc79cf3ee46656a8c80faa/LICENSE

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(lemlib::OdomSensors& sensors) {
  int attempt = 1;
  bool calibrated = false;
  // calibrate inertial, and if calibration fails, then repeat 5 times or until
  // successful
  while (attempt <= 5) {
    sensors.imu->reset();
    // wait until IMU is calibrated
    do pros::delay(10);
    while (sensors.imu->get_status() != pros::ImuStatus::error &&
           sensors.imu->is_calibrating());
    // exit if imu has been calibrated
    if (!isnanf(sensors.imu->get_heading()) &&
        !std::isinf(sensors.imu->get_heading())) {
      calibrated = true;
      break;
    }
    // indicate error
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
    lemlib::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
    attempt++;
  }
  // check if calibration attempts were successful
  if (attempt > 5) {
    sensors.imu = nullptr;
    lemlib::infoSink()->error("IMU calibration failed, defaulting to tracking "
                              "wheels / motor encoders");
  }
}

void LemLibOdom::calibrate() {
  // lemlib::OdomSensors ;
  const bool calibrateImu = true;
  // calibrate the IMU if it exists and the user doesn't specify otherwise
  if (m_sensors.imu != nullptr && calibrateImu) calibrateIMU(m_sensors);
  // initialize odom
  if (m_sensors.vertical1 == nullptr)
    m_sensors.vertical1 = new lemlib::TrackingWheel(
        m_drivetrain.leftMotors, m_drivetrain.wheelDiameter,
        -(m_drivetrain.trackWidth / 2), m_drivetrain.rpm);
  if (m_sensors.vertical2 == nullptr)
    m_sensors.vertical2 = new lemlib::TrackingWheel(
        m_drivetrain.rightMotors, m_drivetrain.wheelDiameter,
        m_drivetrain.trackWidth / 2, m_drivetrain.rpm);
  m_sensors.vertical1->reset();
  m_sensors.vertical2->reset();
  if (m_sensors.horizontal1 != nullptr) m_sensors.horizontal1->reset();
  if (m_sensors.horizontal2 != nullptr) m_sensors.horizontal2->reset();
  setSensors(m_sensors, m_drivetrain);

  // rumble to controller to indicate success
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}
