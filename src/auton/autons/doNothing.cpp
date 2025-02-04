#include "auton/autons.h"
#include <cstdio>

void doNothingFunc(ALLIANCE alliance);

auton::Auton auton::autons::doNothing = {
    .label = (char*)"Do nothing",
    .run = doNothingFunc,
    .labelForController = (char*)"do nothing",
    .description = (char*)"the nuclear option",
};

// Autonomous for negative rush (side of the field with negative corners)
void doNothingFunc(ALLIANCE alliance) { printf("yo what's up\n"); }