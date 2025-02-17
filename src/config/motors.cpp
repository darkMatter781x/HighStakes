#include "pros/motors.h"
#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {14, -4, -12},
    .right {-10, 8, 9},
    .intake {{13, pros::E_MOTOR_GEAR_BLUE}},
    .LadyBrown {{1, pros::E_MOTOR_GEAR_RED}},
};