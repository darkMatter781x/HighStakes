#include "config.h"
#include "pros/adi.hpp"

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {13},
    .hori {19},
    .lift {5},
    .imu {4},
    .intake {6},
    .autonSelector = pros::adi::Potentiometer {'D'}};