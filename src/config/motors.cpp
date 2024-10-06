#include "pros/motors.h"
#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {-16, -15, -14},
    .right {13, 12, 11},
    .intake {{-17, pros::E_MOTOR_GEAR_BLUE}},
    .lift {-9},
};