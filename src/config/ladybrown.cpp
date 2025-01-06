#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"
#include <functional>

LadyBrown::Config LadyBrown::Config::config {
    .idle = 0,
    .collect = 39.17,
    .move_score = 127.37,
    .idle_score = 165.87,
    .controllerSettings =
        lemlib::ControllerSettings {75, 1, 40, 0, 3, 150, 5, 300, 0},
};