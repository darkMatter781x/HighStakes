#include "robot.h"
#include "pros/motor_group.hpp"

lemlib::Drivetrain RobotConfig::makeDrivetrain() const {
  return {&this->motors.left,
          &this->motors.right,
          this->dimensions.trackWidth,
          this->dimensions.driveWheelDiameter,
          this->dimensions.driveWheelRpm,
          this->tunables.horizontalDrift};
}

lemlib::OdomSensors RobotConfig::makeSensors() const {
  return {new lemlib::TrackingWheel(&this->sensors.vert,
                                    this->dimensions.vertEncDiameter,
                                    this->dimensions.vertEncDistance,
                                    this->dimensions.vertEncGearRatio),
          nullptr,
          new lemlib::TrackingWheel(&this->sensors.hori,
                                    this->dimensions.horiEncDiameter,
                                    this->dimensions.horiEncDistance,
                                    this->dimensions.horiEncGearRatio),
          nullptr, &this->sensors.imu};
}

Robot::Robot(const RobotConfig& config)
  : lemlib::Chassis(config.makeDrivetrain(), config.tunables.lateralController,
                    config.tunables.angularController, config.makeSensors(),
                    &config.tunables.driveCurve),
                         new lemlib::TrackingWheel(
                             &config.sensors.hori,
                             config.dimensions.horiEncDiameter,
                             config.dimensions.horiEncDistance,
                             config.dimensions.horiEncGearRatio),
                         nullptr, &config.sensors.imu}},
    m_config(config) {}

Robot& Robot::get() { return Robot::instance; }
Robot Robot::instance {RobotConfig::config};
