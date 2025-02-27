#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'A', true, true},
    .kicker = pros::adi::Pneumatics {'E', false},
    .doinker = pros::adi::Pneumatics {'B', false},
    .intakeLift = pros::adi::Pneumatics {'C', false}};