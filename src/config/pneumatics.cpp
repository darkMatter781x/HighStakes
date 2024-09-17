#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'A', false},
    .ringKicker = pros::adi::Pneumatics {'B', false},
};