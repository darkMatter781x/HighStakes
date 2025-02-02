#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"

LadyBrown::Config LadyBrown::Config::config {
    .idle = 360.00 - 360.f,
    .collect = 320.00 - 360.f,
    .collect2 = 290.00 - 360.f,
    .idle_collect = 260.00 - 360.f,
    .score = 145.00 - 360.f,
    .controllerSettings =
        lemlib::ControllerSettings {300, 3, 1600, 0, 4, 100, 8, 100, 0},
};