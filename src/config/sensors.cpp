#include "config.h"
#include "pros/adi.hpp"

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {20},
    .hori {19},
    .lift {10},
    .imu {7},
    .intake {8},
    .autonSelector = pros::adi::Potentiometer {'D'},
};