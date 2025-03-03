#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserFront {1};
pros::Distance laserLeft {21};
pros::Distance laserRight {2};
pros::Distance laserBack {3};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {20},
    .hori {30},
    .lb {5},
    .imu {19},
    .intake {9},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {7, 4.1, 0},
                .sensor {laserFront},
            },
            {
                .pose {2, 6, M_PI_2},
                .sensor {laserLeft},
            },
            {
                .pose {2, -6, -M_PI_2},
                .sensor {laserRight},
            },
            {
                .pose {-2.1, 5.8, M_PI},
                .sensor {laserBack},
            },
        },
};