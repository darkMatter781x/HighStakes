#include "subsystems/lift.h"
#include "lemlib/chassis/chassis.hpp"

Lift::Config Lift::Config::config {
    .bottom = 120,
    /** parallel to ground ish */
    .middle = 80,
    .top = 50,
    .gearRatio = 1.0 / 3,
    .controllerSettings =
        lemlib::ControllerSettings {2, 0.01, 0, 5, 3, 150, 5, 300, 0},
};