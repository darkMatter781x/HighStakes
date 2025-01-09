#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"
#include <functional>

LadyBrown::Config LadyBrown::Config::config {
    .idle = 0,
    .collect = 34.17,
    .score = 148.00,
    .controllerSettings =
        lemlib::ControllerSettings {65, 5, 220, 0, 4, 100, 8, 100, 0},
};