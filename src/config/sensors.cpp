#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserA {2};
pros::Distance laserB {3};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {20},
    .hori {19},
    .lift {10},
    .imu {7},
    .intake {1},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {4, 7, M_PI_2},
                .sensor {laserB},
            },
            {
                .pose {-5.5, 2, M_PI},
                .sensor {laserA},
            },
        },
};