#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'C', false},
    .ringClaw = pros::adi::Pneumatics {'D', false},
};