#include "pros/motors.h"
#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {12, -14, -18},
    .right {-13, 15, 16},
    .intake {{17, pros::E_MOTOR_GEAR_BLUE}},
    .LadyBrown {{19, pros::E_MOTOR_GEAR_RED}},
};