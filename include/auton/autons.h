#pragma once

#include "color.h"
#include <functional>
#include <optional>

namespace auton {
struct Auton {
    char* label;
    std::function<void(ALLIANCE)> run;
    std::optional<char*> labelForController;
    /** Displayed less prominently than label on brain */
    char* description;
};

namespace autons {
extern Auton negativeRush;
extern Auton positiveRush;
extern Auton safeAWP;
extern Auton skills;
extern Auton doNothing;
} // namespace autons
} // namespace auton