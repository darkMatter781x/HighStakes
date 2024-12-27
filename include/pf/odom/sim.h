#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pf/odom/odom.h"

/**
 * @brief Simulated odometry interface
 */
class SimOdom : public Odom {
  private:
    /** @brief Updated by update() and setPose(). Theta is in standard radians.
     */
    lemlib::Pose m_pose;
    /** @brief Updated by update(). Represents the pose that the robot is
     * actually in. Theta is in standard radians. */
    lemlib::Pose m_realPose;

    /** @brief Used to determine when calibration is complete. Measured in
     * milliseconds */
    size_t m_calibrationStartTime = -1;
  public:
    /** @returns Const ref to the current pose. Theta is in standard radians. */
    const lemlib::Pose& getPose() const override;
    /**
     * @returns Const ref to the current real pose. Theta is in standard
     * radians.
     */
    const lemlib::Pose& getRealPose() const;
    /** @brief Sets pose to newPose. Theta must be in standard radians. */
    void setPose(lemlib::Pose newPose) override;

    /** @brief Calibrates odometry sensors, if it hasn't been done before */
    void calibrate() override;

    bool hasCalibrated() const override;

    /** Calculates new pose using sensor data. */
    void update() override;

    SimOdom();
};
