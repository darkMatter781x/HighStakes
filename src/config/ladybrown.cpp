#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"

LadyBrown::Config LadyBrown::Config::config {
    .idle = 0,
    .collect = 25,
    .score = 132.00,
    .controllerSettings =
        lemlib::ControllerSettings {65, 5, 220, 0, 4, 100, 8, 100, 0},
};