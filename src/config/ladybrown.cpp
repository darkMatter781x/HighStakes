#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"

LadyBrown::Config LadyBrown::Config::config {
    .idle = 0.00,
    .collect = 287.00,
    .score = 90.00,
    .controllerSettings =
        lemlib::ControllerSettings {300, 3, 1600, 0, 4, 100, 8, 100, 0},
};