#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"

LadyBrown::Config LadyBrown::Config::config {
    .idle = 3,
    .collect = 337.5,
    .idle_score = 295.00,
    .score = 180,
    .flip = 127.5,
    .controllerSettings =
        lemlib::ControllerSettings {300, 0, 2500, 0, 4, 100, 8, 100, 0},
};