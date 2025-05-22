#include "config.h"
#include "dimensions.h"

RobotConfig::Dimensions RobotConfig::Dimensions::dimensions = {
    .trackWidth = dimensions::robot::TRACK_WIDTH,
    .driveWheelDiameter =
        3.25f * 84.0 / 75.5 * 84.0 / 83.0 /** Scale factor to make more accurate */,
    .driveWheelRpm = 450,
    .vertEncDiameter = 10000.f, /** Doesn't exist */
    .vertEncDistance = 1.25f,
    .vertEncGearRatio = 1.f,
    .horiEncDiameter =
        2.f * 72.0 / 73.0f, /** Scale factor to make more accurate */
    .horiEncDistance = -2.25,
    .horiEncGearRatio = 1,
    .drivetrainWidth = dimensions::robot::DRIVE_WIDTH,
    .drivetrainLength = dimensions::robot::DRIVE_LENGTH,
};