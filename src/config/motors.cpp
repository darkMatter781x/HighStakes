#include "pros/motors.h"
#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {-19, 16, -18},
    .right {20, -14, 17},
    .intake {{-15}, pros::v5::MotorGears::blue},
    .LadyBrown {{-4}, pros::v5::MotorGears::red},
};