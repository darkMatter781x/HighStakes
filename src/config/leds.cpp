#include "config.h"

RobotConfig::LEDs RobotConfig::LEDs::leds = {.lift =pros::adi::LED{'A', 64},
                                             .underGlow = pros::adi::LED{'B', 64}};