#include "subsystems/intake.h"

constexpr double INCHES_PER_CM = 0.3937008;

Intake::Config Intake::Config::config = {
    .intakingToLiftInches = 6,
    .outtakingToLiftDuration = 800,
    .intakingToKickDuration = 0,
    .idlingToKickDuration = 250,  
    .chainRatio = 1.0 /* rot */ / (18.0 /* teeth */ * 1.0 /* cm/tooth */ * INCHES_PER_CM),
};