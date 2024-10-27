#include "lemlib/chassis/chassis.hpp"
#include "config.h"

RobotConfig::Tunables RobotConfig::Tunables::tunables {
    .lateralController = lemlib::ControllerSettings {14.25, 0.0, 64, 0.0, 0.0,
                                                     0.0, 0.0, 0.0, 0.0},
    .angularController = lemlib::ControllerSettings {3.6875, 0.0, 28, 0.0, 0.0,
                                                     0.0, 0.0, 0.0, 0.0},
    .horizontalDrift = 0.0,
    .imuGain = 0.0,
    .driveCurve = lemlib::ExpoDriveCurve {0, 0, 1}};