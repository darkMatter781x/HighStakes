#include "pros/motors.h"
#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {7, -6, -18},
    .right {-13, 11, 12},
    .intake {{8, pros::E_MOTOR_GEAR_BLUE}},
    .LadyBrown {{17, pros::E_MOTOR_GEAR_RED}},
};