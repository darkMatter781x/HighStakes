#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'B', true, true},
    .kicker = pros::adi::Pneumatics {'A', false},
    .doinker = pros::adi::Pneumatics {'H', false},
    .intakeLift = pros::adi::Pneumatics {'C', false}};