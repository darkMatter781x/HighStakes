#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserFront {3};
pros::Distance laserLeft {2};
pros::Distance laserRight {1};
pros::Distance laserBack {11};

RobotConfig::Sensors RobotConfig::Sensors::sensors {
    .vert {30},
    .hori {-13},
    .lb {-5},
    .imu {12},
    .intake {6},
    .autonSelector = pros::adi::Potentiometer {'D'},
    .lasers =
        {
            {
                .pose {5, 2.25, 0}, // TODO: Configure position
                .sensor {laserFront},
            },
            {
                .pose {2.4, 5.8, M_PI_2}, // TODO: Configure position
                .sensor {laserLeft},
            },
            {
                .pose {2.4, -5.8, -M_PI_2}, // TODO: Configure position
                .sensor {laserRight},
            },
            {
                .pose {-5, -3, M_PI}, // TODO: Configure position
                .sensor {laserBack},
            },
        },
};