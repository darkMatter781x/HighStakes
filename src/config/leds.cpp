#include "config.h"

RobotConfig::LEDs RobotConfig::LEDs::leds = {.lift =pros::adi::LED{'A', 64},
                                             .leftUnderGlow = pros::adi::LED{{10, 'A'}, 45},
                                             .rightUnderGlow = pros::adi::LED{{10, 'E'}, 45}};