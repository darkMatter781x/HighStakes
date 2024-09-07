#include "config.h"

// 1, 3, 4, 6 adi ports work
RobotConfig::LEDs RobotConfig::LEDs::leds = {
    .lift = LedStrip::create({8, 'D'}, 33, 0.1),
    .leftUnderGlow = LedStrip::create({8, 'A'}, 45, 0.1),
    .rightUnderGlow = LedStrip::create({8, 'C'}, 45, 0.1)};