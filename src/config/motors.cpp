#include "pros/motors.h"
#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {12, -14, -17},
    .right {-13, 15, 16},
    .intake {{11, pros::E_MOTOR_GEAR_BLUE}},
    .LadyBrown {{18, pros::E_MOTOR_GEAR_RED}},
};