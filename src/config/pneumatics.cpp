#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'A', false},
    .ringClaw = pros::adi::Pneumatics {'D', false},
};