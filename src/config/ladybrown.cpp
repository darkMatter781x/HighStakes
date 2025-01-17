#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"
#include <functional>

LadyBrown::Config LadyBrown::Config::config {
    .idle = 0,
    .collect = 30.17,
    .score = 159.00,
    .controllerSettings =
        lemlib::ControllerSettings {67, 5.5, 225, 0, 4, 100, 8, 100, 0},
};