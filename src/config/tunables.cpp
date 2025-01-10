#include "lemlib/chassis/chassis.hpp"
#include "config.h"

RobotConfig::Tunables RobotConfig::Tunables::tunables {
    .lateralController =
        lemlib::ControllerSettings {
            16, // kP
            0, // kI
            90, // kD
            3, // windup range
            0, // smallErrorRange
            float(INT_MAX), // smallErrorTimeout
            0, // largeErrorRange
            float(INT_MAX), // largeErrorTimeout
            0 // slew rate
        },
    .angularController =
        lemlib::ControllerSettings {
            3.625, // kP
            0, // kI
            28, // kD
            15, // windup range
            0, // smallErrorRange
            float(INT_MAX), // smallErrorTimeout
            0, // largeErrorRange
            float(INT_MAX), // largeErrorTimeout
            0 // slew rate
        },
    .horizontalDrift = 10.0,
    .imuGain = 0.0,
    .driveCurve = lemlib::ExpoDriveCurve {0, 0, 1}};