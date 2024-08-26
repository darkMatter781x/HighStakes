#include "config.h"

RobotConfig::LEDs RobotConfig::LEDs::leds = {
    .lift = LedStrip::create({0, 'A'}, 64, 0.1),
    .leftUnderGlow = LedStrip::create({10, 'A'}, 45, 0.1),
    .rightUnderGlow = LedStrip::create({10, 'E'}, 45, 0.1)};