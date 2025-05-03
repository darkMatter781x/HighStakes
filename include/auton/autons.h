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
extern Auton negativeElims;
extern Auton safePositive;
extern Auton negAWP;
extern Auton posAWP;
extern Auton skills;
extern Auton doNothing;
extern Auton posRush;
} // namespace autons
} // namespace auton