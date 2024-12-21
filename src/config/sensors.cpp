#include "config.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cmath>

pros::Distance laserA {6};
pros::Distance laserB {10};
pros::Distance laserC {11};
pros::Distance laserD {12};

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
                .pose {0, 0, 0},
                .sensor {laserA},
            },
            {
                .pose {0, 0, M_PI_4},
                .sensor {laserA},
            },
            // {
            //     .pose {0, 0, M_PI_2},
            //     .sensor {laserB},
            // },
            // {
            //     .pose {0, 0, 3 * M_PI_4},
            //     .sensor {laserB},
            // },
            // {
            //     .pose {0, 0, M_PI},
            //     .sensor {laserC},
            // },
            // {
            //     .pose {0, 0, 5 * M_PI_4},
            //     .sensor {laserC},
            // },
            // {
            //     .pose {0, 0, 3 * M_PI_2},
            //     .sensor {laserD},
            // },
            // {
            //     .pose {0, 0, 7 * M_PI_4},
            //     .sensor {laserD},
            // },
        },
};