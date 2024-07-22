#include "robot.h"
const RobotConfig RobotConfig::config {
    .motors = RobotConfig::Motors::motors,
    .pneumatics = RobotConfig::Pneumatics::pneumatics,
    .sensors = RobotConfig::Sensors::sensors,
    .leds = RobotConfig::LEDs::leds,
    .dimensions = RobotConfig::Dimensions::dimensions,
    .tunables = RobotConfig::Tunables::tunables,
};