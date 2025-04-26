#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"

LadyBrown::Config LadyBrown::Config::config {
    .idle = 10,
    .collect = 345,
    .idle_score = 310,
    .score = 180 - 30,
    .run_into_ally = 203.f,
    .flip = 100.f,
    .controllerSettings =
        lemlib::ControllerSettings {300, 0, 2500, 0, 4, 100, 8, 100, 0},
    .sensorGearRatio = 1 / 2.0f,
};