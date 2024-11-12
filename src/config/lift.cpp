#include "subsystems/lift.h"
#include "lemlib/chassis/chassis.hpp"

Lift::Config Lift::Config::config {
    .bottom = 290,
    .middle = 330,
    .top = 357.5,
    .gearRatio = 1.0,
    .controllerSettings =
        lemlib::ControllerSettings {40, 10, 30, 0, 3, 150, 5, 300, 0},
};