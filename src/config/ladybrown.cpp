#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"

LadyBrown::Config LadyBrown::Config::config {
    .idle = 358.00,
    .collect = 293.00,
    .idle_collect = 150.00,
    .score = 100.00,
    .controllerSettings =
        lemlib::ControllerSettings {300, 3, 1600, 0, 4, 100, 8, 100, 0},
};