#include "config.h"

// 1, 3, 4, 6 adi ports work
RobotConfig::LEDs RobotConfig::LEDs::leds = {
    .leftLift = LedStrip::create({20, 'B'}, 33, 0.1) /* nullptr */,
    .rightLift = LedStrip::create({20, 'B'}, 33, 0.1) /* nullptr */,
    .leftUnderGlow = LedStrip::create({9, 'A'}, 45, 0.1) /* nullptr */,
    .rightUnderGlow = LedStrip::create({20, 'A'}, 45, 0.1) /* nullptr */,
};x