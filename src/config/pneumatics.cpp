#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'E', true, true},
    .kicker = pros::adi::Pneumatics {'F', false},
    .leftDoinker = pros::adi::Pneumatics {'H', false},
    .rightDoinker = pros::adi::Pneumatics {'A', false},
    .intakeLift = pros::adi::Pneumatics {'B', false}};