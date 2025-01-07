#include "config.h"

RobotConfig::Pneumatics RobotConfig::Pneumatics::pneumatics {
    .mogoClamp = pros::adi::Pneumatics {'B', false},
    .ringKicker = pros::adi::Pneumatics {'A', false},
    .doinker = pros::adi::Pneumatics {'H', false},
    .intakeLift = pros::adi::Pneumatics {'C', true, true}};