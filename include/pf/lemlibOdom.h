#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/chassis/chassis.hpp"

/**
 * @brief Object wrapper for LemLib odometry using tracking wheels.
 */
class LemLibOdom {
  private:
    /** Used to setup odom */
    lemlib::OdomSensors m_sensors;
    /** Used to setup odom */
    lemlib::Drivetrain m_drivetrain;

    /** @brief Updated by update() and setPose(). Used to return references.
     * Theta is in standard radians. */
    lemlib::Pose m_pose;
  public:
    /** @returns Const ref to current pose. Theta is in standard radians. */
    const lemlib::Pose& getPose() const;
    /** @returns Mutable ref to current pose. Theta is in standard radians. */
    lemlib::Pose& getPose();
    /** @brief Sets pose to newPose. Theta must be in standard radians. */
    void setPose(lemlib::Pose newPose);
    void calibrate();

    /** Calculates new pose using sensor data. */
    void update();

    LemLibOdom(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain);
};
