#include "config.h"
#include "dimensions.h"

RobotConfig::Dimensions RobotConfig::Dimensions::dimensions = {
    .trackWidth = dimensions::robot::TRACK_WIDTH,
    .driveWheelDiameter = lemlib::Omniwheel::NEW_275,
    .driveWheelRpm = 450,
    .driveEncGearRatio = 0.024,
    .vertEncDiameter = lemlib::Omniwheel::NEW_275,
    .vertEncDistance = 0,
    .vertEncGearRatio = 60.0 / 36,
    .horiEncDiameter = lemlib::Omniwheel::NEW_275,
    .horiEncDistance = -2,
    .horiEncGearRatio = 60.0 / 36,
    .drivetrainWidth = dimensions::robot::DRIVE_WIDTH,
    .drivetrainLength = dimensions::robot::DRIVE_LENGTH};