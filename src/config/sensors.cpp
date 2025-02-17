#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserFront {22};
pros::Distance laserLeft {23};
pros::Distance laserRight {24};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {3},
    .hori {30},
    .lb {16},
    .imu {2},
    .intake {11},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {8, -4, 0},
                .sensor {laserFront},
            },
            {
                .pose {2.25, 5.8, M_PI_2},
                .sensor {laserLeft},
            },
            {
                .pose {2.25, -5.8, -M_PI_2},
                .sensor {laserRight},
            },
        },
};