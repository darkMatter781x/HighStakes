#include "robot.h"
#include "Eigen/src/Core/Matrix.h"
#include "pf/localization.h"
#include "pf/util/lines.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include <vector>

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

std::vector<lines::Segment> makeObstacles() {
  return {lines::Segment({0, 0}, {0, 0})};
}

Localization::Config makeLocalizationConfig(const RobotConfig& config) {
  return {
      .odomSensors = config.makeSensors(),
      .drivetrain = config.makeDrivetrain(),
      .obstacles = makeObstacles(),
      .distances = config.sensors.lasers,
      .covar = Eigen::Matrix2f::Identity() * .01,
  };
}

Robot::Robot(const RobotConfig& config)
  : lemlib::Chassis(config.makeDrivetrain(), config.tunables.lateralController,
                    config.tunables.angularController, config.makeSensors(),
                    &config.tunables.driveCurve),
    m_mogo {config.pneumatics.mogoClamp}, mogo(m_mogo),
    m_intakeSensor(config.sensors.intake),
    m_intake {config.motors.intake, m_intakeSensor,
              config.pneumatics.ringKicker, Intake::Config::config},
    intake(m_intake),
    m_lift {config.motors.lift, config.sensors.lift, Lift::Config::config},
    lift(m_lift), m_config(config), m_gamepad(pros::E_CONTROLLER_MASTER),
    gamepad(m_gamepad), selector(AutonSelector::getRef(
                            config.sensors.autonSelector, m_intakeSensor)),
    m_localization(makeLocalizationConfig(config)),
    localization(m_localization) {}

Robot Robot::instance {RobotConfig::config};
