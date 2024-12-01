#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pf/odom/odom.h"

/**
 * @brief Object wrapper for LemLib odometry using tracking wheels.
 */
class LemLibOdom : public Odom {
  private:
    /** Used to setup odom */
    lemlib::OdomSensors m_sensors;
    /** Used to setup odom */
    lemlib::Drivetrain m_drivetrain;

    /** @brief Updated by update() and setPose(). Used to return references.
     * Theta is in standard radians. */
    lemlib::Pose m_pose;

    bool m_hasCalibrated = false;
  public:
    /** @returns Const ref to current pose. Theta is in standard radians. */
    const lemlib::Pose& getPose() const override;
    /** @returns Mutable ref to current pose. Theta is in standard radians. */
    lemlib::Pose& getPose();
    /** @brief Sets pose to newPose. Theta must be in standard radians. */
    void setPose(lemlib::Pose newPose) override;

    /** @brief Calibrates odometry sensors, if it hasn't been done before */
    void calibrate() override;

    bool hasCalibrated() const override;

    /** Calculates new pose using sensor data. */
    void update() override;

    LemLibOdom(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain);
};
