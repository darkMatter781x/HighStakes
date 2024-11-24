#pragma once
#include "lemlib/Pose.hpp"
#include "lemlib/chassis/chassis.hpp"

/**
 * @brief Object wrapper for LemLib odometry using tracking wheels.
 */
class LemLibOdom {
  private:
    /** Used to setup odom */
    lemlib::OdomSensors& m_sensors;
    /** Used to setup odom */
    lemlib::Drivetrain& m_drivetrain;
  public:
    lemlib::Pose getPose() const;
    void setPose(const lemlib::Pose& pose);
    void calibrate();

    /** Calculates new pose using sensor data. */
    void update();

    LemLibOdom(lemlib::OdomSensors& sensors, lemlib::Drivetrain& drivetrain)
      : m_sensors(sensors), m_drivetrain(drivetrain) {}
};
