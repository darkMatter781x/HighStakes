#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'E', true, true},
    .kicker = pros::adi::Pneumatics {'A', false},
    .doinker = pros::adi::Pneumatics {'B', false},
    .intakeLift = pros::adi::Pneumatics {'F', false}};