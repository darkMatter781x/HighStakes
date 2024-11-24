#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

pros::Distance laserA {6};
pros::Distance laserB {10};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {20},
    .hori {19},
    .lift {10},
    .imu {7},
    .intake {8},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {-1, -4, 0},
                .sensor {laserA},
            },
            {
                .pose {-1, -6, M_PI},
                .sensor {laserB},
            },
        },
};