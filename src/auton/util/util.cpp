#include "auton/util.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "lemlib/util.hpp"
#include <cmath>

namespace auton::util {

void tankUpdate(float left, float right, float slew) {
  static float prevLeft = 0;
  static float prevRight = 0;
  const float leftLimited = lemlib::slew(left, prevLeft, slew);
  const float rightLimited = lemlib::slew(right, prevRight, slew);
  bot.tank(leftLimited, rightLimited, 0);
  prevLeft = leftLimited;
  prevRight = rightLimited;
}

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
void tank(float left, float right, int ms, float slew) {
  const int startTime = pros::millis();
  do {
    tankUpdate(left, right, slew);
    pros::delay(10);
  } while (pros::millis() - startTime < ms);
}

void stop() { tank(0, 0, 0, 0); }

/**
 * @brief Wait until the robot is within a circle with a radius of error and
 * centered at pose for time milliseconds.
 *
 * @param pose pose to find distance from robot to
 * @param error
 * @param time
 */
void waitUntilDistToPose(lemlib::Pose pose, float error, int time,
                         bool checkMotionRunning) {
  printf("start wait\n");
  const int start = pros::millis();
  int inRangeStartTime = 0;
  do {
    if (inRangeStartTime == 0) {
      if (bot.getPose().distance(pose) < error)
        inRangeStartTime = pros::millis();
    } else if (pros::millis() - inRangeStartTime > time) {
      break;
    }
    pros::delay(10);
  } while (!checkMotionRunning || bot.isInMotion());
  printf("time in wait: %i\n", pros::millis() - start);
}

float robotAngDist(float target) {
  return std::fabs(std::remainder(bot.getPose().theta - target, 360));
}

void waitUntil(std::function<bool()> condition, int timeConditionIsTrue,
               int timeout, bool resetTrueStartTime) {
  const int start = pros::millis();
  int conditionTrueStartTime = 0;
  while ((pros::millis() - start < timeout)) {
    const bool condVal = condition();
    if (!condVal && resetTrueStartTime) conditionTrueStartTime = 0;
    if (conditionTrueStartTime == 0 && condVal)
      conditionTrueStartTime = pros::millis();
    if (conditionTrueStartTime != 0 &&
        (pros::millis() - conditionTrueStartTime) > timeConditionIsTrue)
      break;

    pros::delay(10);
  }
}

} // namespace auton::util

lemlib::Pose lemlib::Pose::fromPolar(float r, float theta, bool inRadians,
                                     bool standardPos) {
  const float originalTheta = theta;
  if (!inRadians) theta = lemlib::degToRad(theta);
  if (!standardPos) theta = M_PI / 2 - theta;
  return {r * std::cos(theta), r * std::sin(theta), originalTheta};
}