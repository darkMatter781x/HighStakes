#pragma once

#include "lemlib/pose.hpp"

/**
 * @brief Interface for odometry sensors.
 */
class Odom {
  public:
    /** @brief Calculates new pose with new data */
    virtual void update() = 0;

    /** @returns Const ref to current pose. Theta is in standard radians. */
    virtual const lemlib::Pose& getPose() const = 0;

    /** @brief Sets pose to newPose. Theta must be in standard radians. */
    virtual void setPose(lemlib::Pose pose) = 0;

    /** @brief Calibrates sensors */
    virtual void calibrate() = 0;

    /** @returns Whether the sensors have been calibrated yet */
    virtual bool hasCalibrated() const = 0;
};