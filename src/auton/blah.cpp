#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "robot.h"
#include "dimensions.h"

using namespace lemlib;

void Robot::customMoveToPose(/** Yet to be transformed */ const Pose rawTarget,
                             const size_t timeout, const Pose offset,
                             MoveToPoseParams params, bool async) {
  // take the mutex
  this->requestMotionStart();
  // were all motions cancelled?
  if (!this->motionRunning) return;
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task(
        [&]() { customMoveToPose(rawTarget, timeout, offset, params, false); });
    this->endMotion();
    pros::delay(10); // delay to give the task time to start
    return;
  }

  // reset PIDs and exit conditions
  lateralPID.reset();
  lateralLargeExit.reset();
  lateralSmallExit.reset();
  angularPID.reset();
  angularLargeExit.reset();
  angularSmallExit.reset();

  // transform target pose
  const Pose transformedTarget = this->m_transform->transformPose(rawTarget);
  // calculate target pose in standard form
  Pose target =
      transformedTarget.withTheta(M_PI - degToRad(transformedTarget.theta));
  if (!params.forwards)
    target.theta = fmod(target.theta + M_PI, 2 * M_PI); // backwards movement

  // use global horizontalDrift is horizontalDrift is 0
  if (params.horizontalDrift == 0)
    params.horizontalDrift = drivetrain.horizontalDrift;

  // initialize vars used between iterations
  Pose lastPose = getPose();
  distTraveled = 0;
  Timer timer(timeout);
  bool lateralSettled = false;
  bool prevSameSide = false;
  float prevLateralOut = 0; // previous lateral power
  float prevAngularOut = 0; // previous angular power
  const int compState = pros::competition::get_status();

  bool close = false;

  // main loop
  while (!(timer.isDone() || lateralSettled && (angularLargeExit.getExit() ||
                                                angularSmallExit.getExit())) &&
         this->motionRunning) {
    // update position
    const Pose centerPose = lemlib::Chassis::getPose(true, true);
    // Calculate position of the offset point
    const Pose pose = centerPose + offset.rotate(centerPose.theta);
    // update distance traveled
    distTraveled += pose.distance(lastPose);
    lastPose = pose;

    // calculate distance to the target point
    const float distTarget = pose.distance(target);

    close |= distTarget < 7.5;

    // // check if the robot is close enough to the target to start settling
    // if (distTarget < 7.5 && close == false) {
    //   close = true;
    //   params.maxSpeed = fmax(fabs(prevLateralOut), 60);
    // }

    // check if the lateral controller has settled
    if (lateralLargeExit.getExit() && lateralSmallExit.getExit())
      lateralSettled = true;

    const float longitudinalCarrotDistance = params.lead * distTarget;
    const float angularCarrotDistance = longitudinalCarrotDistance - 4;

    /** Carrot to be used to calculate longitudinal power. */
    const Pose longitudinalCarrot =
        target -
        Pose(cos(target.theta), sin(target.theta)) * longitudinalCarrotDistance;
    /** Carrot to be used to calculate angular power. */
    const Pose angularCarrot =
        target -
        Pose(cos(target.theta), sin(target.theta)) * angularCarrotDistance;

    // // calculate if the robot is on the same side as the carrot point
    // const bool robotSide =
    //     (pose.y - target.y) * -sin(target.theta) <=
    //     (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
    // const bool carrotSide =
    //     (carrot.y - target.y) * -sin(target.theta) <=
    //     (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
    // const bool sameSide = robotSide == carrotSide;
    // // exit if close
    // if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
    // prevSameSide = sameSide;

    // calculate error
    const float adjustedRobotTheta =
        params.forwards ? pose.theta : pose.theta + M_PI;
    const float angularError =
        angleError(adjustedRobotTheta, pose.angle(angularCarrot));
    float longitudinalError = pose.distance(longitudinalCarrot);
    // only use cos when settling
    // otherwise just multiply by the sign of cos
    // maxSlipSpeed takes care of lateralOut
    if (close)
      longitudinalError *=
          cos(angleError(pose.theta, pose.angle(angularCarrot)));
    else
      longitudinalError *=
          sgn(cos(angleError(pose.theta, pose.angle(angularCarrot))));

    // update exit conditions
    lateralSmallExit.update(longitudinalError);
    lateralLargeExit.update(longitudinalError);
    angularSmallExit.update(radToDeg(angularError));
    angularLargeExit.update(radToDeg(angularError));

    // get output from PIDs
    float lateralOut = lateralPID.update(longitudinalError);
    float angularOut = angularPID.update(radToDeg(angularError));

    // apply restrictions on angular speed
    angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

    // apply restrictions on lateral speed
    lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

    // constrain lateral output by max accel, only if accelerating in direction
    // of desired travel
    if ((params.forwards && lateralOut > prevLateralOut) ||
        (!params.forwards && lateralOut < prevLateralOut))
      lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

    // constrain lateral output by the max speed it can travel at without
    // slipping
    const float radius = 1 / fabs(getCurvature(pose, longitudinalCarrot));
    const float maxSlipSpeed(sqrt(params.horizontalDrift * radius * 9.8));
    lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
    // prioritize angular movement over lateral movement
    const float overturn =
        fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
    if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

    // prevent moving in the wrong direction
    if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
    else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

    // constrain lateral output by the minimum speed
    if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0)
      lateralOut = fabs(params.minSpeed);
    if (!params.forwards && -lateralOut < fabs(params.minSpeed) &&
        lateralOut < 0)
      lateralOut = -fabs(params.minSpeed);

    // update previous output
    prevAngularOut = angularOut;
    prevLateralOut = lateralOut;

    infoSink()->debug("lateralOut: {} angularOut: {}", lateralOut, angularOut);

    // ratio the speeds to respect the max speed
    float leftPower = lateralOut + angularOut;
    float rightPower = lateralOut - angularOut;
    const float ratio =
        std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
    if (ratio > 1) {
      leftPower /= ratio;
      rightPower /= ratio;
    }

    // move the drivetrain
    drivetrain.leftMotors->move(leftPower);
    drivetrain.rightMotors->move(rightPower);

    // delay to save resources
    pros::delay(10);
  }

  // stop the drivetrain
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
  // set distTraveled to -1 to indicate that the function has finished
  distTraveled = -1;
  this->endMotion();
}

/** Hole */
void goToCenter() {
  using namespace dimensions::all;

  /** Center of the rushed rings */
  const Pose ringRushCenter {0, TILE * 2};
  /** Rushed ring that will be intaked. */
  const Pose intakedRing =
      ringRushCenter + Pose {-RING_DIAM / 2, -RING_DIAM / 2};
  /** Rushed ring that will be doinked. */
  const Pose doinkedRing =
      ringRushCenter + Pose {-RING_DIAM / 2, RING_DIAM / 2};
  /** How far the hook part of the doinker sticks out from the drivetrain.  */
  const float doinkerLength = 5;
  /** How close the center of the robot must be to intake a ringa. */
  const float intakeDistance = 2 + DRIVE_LENGTH / 2;
  /** Where the hook part of the doinker is, relative to the center of the bot.
   * X is forwards/backwards and Y is side to side. */
  const Pose doinkerOffset {DRIVE_LENGTH / 2 + doinkerLength, -DRIVE_WIDTH / 2};

  const float ringRushHeading = BLUE_STATION - 30;

  // /** How offset the center of the intake can be from the intaked ring. */
  // const float maxIntakeOffset = 2;
  // const Pose ringRushTarget =
  //     (doinkedRing + doinkerOffset.rotate((90 - ringRushHeading) * M_PI /
  //     180))
  //         .withTheta(ringRushHeading);

  bot.customMoveToPose(doinkedRing.withTheta(ringRushHeading), 2000,
                       doinkerOffset, {.lead = .5});
}
