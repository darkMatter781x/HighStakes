#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserA {2};
pros::Distance laserB {3};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {21},
    .hori {30},
    .lb {4},
    .imu {20},
    .intake {1},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {4.35, 7, M_PI_2},
                .sensor {laserB},
            },
            {
                .pose {-5.8, 2, M_PI},
                .sensor {laserA},
            },
        },
};