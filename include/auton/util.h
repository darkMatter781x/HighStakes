#pragma once

#include "lemlib/pose.hpp"
#include <climits>
#include <cmath>
#include <functional>

namespace auton::util {
const float DEFAULT_SLEW = 15;

/**
 * @brief Tank drive with slew rate control
 * Slew rate control is a method of limiting the rate of change of the voltage
 * sent to the motors. This is useful for ensuring that the tracking wheels
 * always contact the ground.
 *
 * @param left left side power
 * @param right right side power
 * @param ms time to run for
 * @param slew slew rate
 */
void tank(float left, float right, int ms, float slew = DEFAULT_SLEW);

/**
 * @brief Stops sending voltage to the drivetrain motors
 */
void stop();

/**
 * @brief Wait until the robot is within a circle with a radius of error and
 * centered at pose for time milliseconds.
 *
 * @param pose pose to find distance from robot to
 * @param error
 * @param time
 */
void waitUntilDistToPose(lemlib::Pose pose, float error, int time = 0,
                         bool checkMotionRunning = true);

/**
 * @brief Gets the distance between the robot's heading and a target heading
 *
 * @param ang target heading in degrees
 * @return a positive float with range of [0,180]
 */
float robotAngDist(float target);

bool isMotionRunning();

void waitUntil(std::function<bool()> condition, int timeConditionIsTrue = 0,
               int timeout = INT_MAX, bool resetTrueStartTime = false);

/**
 * @brief
 * Lemlib uses heading, which is like a compass, whereas trig functions like
 * sine and cosine use trig angles. Therefore we must convert the trig angle to
 * a heading.
 *
 * @param angle trig angle
 * @return heading angle
 */
constexpr float trigAngleToHeading(float angle) {
  return -(angle * 180 / M_PI) + 90;
}
} // namespace auton::util