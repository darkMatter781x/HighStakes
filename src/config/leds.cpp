#include "config.h"

// 1, 3, 4, 6 adi ports work
RobotConfig::LEDs RobotConfig::LEDs::leds = {
    .leftLift = LedStrip::create({8, 'C'}, 33, 0.1) /* nullptr */,
    .rightLift = LedStrip::create({8, 'H'}, 33, 0.1) /* nullptr */,
    .leftUnderGlow = LedStrip::create({8, 'E'}, 45, 0.1) /* nullptr */,
    .rightUnderGlow = LedStrip::create({8, 'F'}, 45, 0.1) /* nullptr */,
};