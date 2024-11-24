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

    /** Updated by update(). Used to return references. */
    lemlib::Pose m_pose;
  public:
    const lemlib::Pose& getPose() const;
    lemlib::Pose& getPose();
    void setPose(const lemlib::Pose& pose);
    void calibrate();

    /** Calculates new pose using sensor data. */
    void update();

    LemLibOdom(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain);
};
