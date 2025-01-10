#include "lemlib/chassis/chassis.hpp"
#include "config.h"

RobotConfig::Tunables RobotConfig::Tunables::tunables {
    .lateralController =
        lemlib::ControllerSettings {
            16, // kP
            0, // kI
            90, // kD
            3, // windup range
            1, // smallErrorRange
            100, // smallErrorTimeout
            3, // largeErrorRange
            500, // largeErrorTimeout
            0 // slew rate
        },
    .angularController =
        lemlib::ControllerSettings {
            3.625, // kP
            0, // kI
            28, // kD
            15, // windup range
            1, // smallErrorRange
            100, // smallErrorTimeout
            3.5, // largeErrorRange
            350, // largeErrorTimeout
            0 // slew rate
        },
    .horizontalDrift = 10.0,
    .imuGain = 0.0,
    .driveCurve = lemlib::ExpoDriveCurve {0, 0, 1}};