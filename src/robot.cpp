#include "robot.h"
#include "pros/misc.h"
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
    m_mogo {config.pneumatics.mogoClamp}, mogo(m_mogo),
    m_intakeSensor(config.sensors.intake),
    m_intake {config.motors.intake, m_intakeSensor}, intake(m_intake),
    m_lift {config.motors.lift, config.sensors.lift, Lift::Config::config},
    lift(m_lift), m_config(config), m_gamepad(pros::E_CONTROLLER_MASTER),
    gamepad(m_gamepad), selector(AutonSelector::getRef(
                            config.sensors.autonSelector, m_intakeSensor)) {}

Robot Robot::instance {RobotConfig::config};
