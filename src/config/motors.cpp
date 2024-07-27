#include "config.h"

RobotConfig::Motors RobotConfig::Motors::motors {
    .left {-1, -2, -8},
    .right {3, 4, 9},
    .intake {5},
    // 5.5Ws
    .lift {6, 7},
};