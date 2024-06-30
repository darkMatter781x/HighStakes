#include "robot.h"

Robot::Robot(RobotConfig& config)
  : lemlib::Chassis {lemlib::Drivetrain {&config.motors.left,
                                         &config.motors.right,
                                         config.dimensions.trackWidth,
                                         config.dimensions.driveWheelDiameter,
                                         config.dimensions.driveWheelRpm,
                                         config.tunables.horizontalDrift},
                     config.tunables.lateralController,
                     config.tunables.angularController,
                     lemlib::OdomSensors {
                         new lemlib::TrackingWheel(
                             &config.sensors.vert,
                             config.dimensions.vertEncDiameter,
                             config.dimensions.vertEncDistance,
                             config.dimensions.vertEncGearRatio),
                         nullptr,
                         new lemlib::TrackingWheel(
                             &config.sensors.hori,
                             config.dimensions.horiEncDiameter,
                             config.dimensions.horiEncDistance,
                             config.dimensions.horiEncGearRatio),
                         nullptr, &config.sensors.imu}},
    m_config(config) {}

Robot* Robot::get() { return Robot::instance; }