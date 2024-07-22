#include "config.h"
#include "dimensions.h"

RobotConfig::Dimensions RobotConfig::Dimensions::dimensions = {
    .trackWidth = dimensions::robot::TRACK_WIDTH,
    .driveWheelDiameter = 4.0,
    .driveWheelRpm = 200,
    .driveEncGearRatio = 1.0,
    .vertEncDiameter = 2.0,
    .vertEncDistance = 2.0,
    .vertEncGearRatio = 1.0,
    .horiEncDiameter = 2.0,
    .horiEncDistance = 2.0,
    .horiEncGearRatio = 1.0,
    .drivetrainWidth = dimensions::robot::DRIVE_WIDTH,
    .drivetrainLength = dimensions::robot::DRIVE_LENGTH};