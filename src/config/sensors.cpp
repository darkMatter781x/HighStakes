#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserFront {2};
pros::Distance laserLeft {3};
pros::Distance laserRight {4};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {21},
    .hori {30},
    .lb {11},
    .imu {20},
    .intake {1},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {4, 6.5, 0},
                .sensor {laserFront},
            },
            {
                .pose {-5.8, 2, M_PI_2},
                .sensor {laserLeft},
            },
            {
                .pose {5.8, 2, -M_PI_2},
                .sensor {laserRight},
            },
        },
};