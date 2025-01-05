#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"
#include <functional>

LadyBrown::Config LadyBrown::Config::config {
    .idle = 0,
    .collect = 29.17,
    .move_score = 129.37,
    .idle_score = 167.87,
    .controllerSettings =
        lemlib::ControllerSettings {40, 10, 30, 0, 3, 150, 5, 300, 0},
};