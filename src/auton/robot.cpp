#include "robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

constexpr auto ALLIANCE_DEFAULT = ALLIANCE::RED;

void Robot::setAlliance(ALLIANCE alliance) { m_alliance = alliance; }

ALLIANCE Robot::getAlliance() const { return m_alliance; }

COLOR Robot::getColor() const { return getAlliance(); }

bool needsTransform() { return bot.getAlliance() != ALLIANCE_DEFAULT; }

/** if in radians, stays in radians */
float transformHeading(float origThetaDegrees) {
  if (needsTransform()) return 0 - origThetaDegrees;
  return origThetaDegrees;
}

/** if in radians, stays in radians */
lemlib::Pose transformPose(lemlib::Pose original) {
  if (needsTransform())
    return {-original.x, original.y, transformHeading(original.theta)};
  else return original;
}

lemlib::DriveSide transformDriveSide(lemlib::DriveSide original) {
  constexpr auto LEFT = lemlib::DriveSide::LEFT;
  constexpr auto RIGHT = lemlib::DriveSide::RIGHT;
  if (needsTransform()) switch (original) {
      case LEFT: return RIGHT;
      case RIGHT: return LEFT;
    }
  else return original;
}

lemlib::AngularDirection transformAngDir(lemlib::AngularDirection original) {
  constexpr auto CCW = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
  constexpr auto CW = lemlib::AngularDirection::CW_CLOCKWISE;
  constexpr auto AUTO = lemlib::AngularDirection::AUTO;
  if (needsTransform()) switch (original) {
      case CCW: return CW;
      case CW: return CCW;
      case AUTO: return AUTO;
    }
  else return original;
}

void Robot::setPose(lemlib::Pose newPose, bool radians) {
  lemlib::Chassis::setPose(transformPose(newPose), radians);
}

lemlib::Pose Robot::getPose(bool radians) {
  return transformPose(lemlib::Chassis::getPose(radians));
}

void Robot::turnToPoint(lemlib::Pose target, int timeout,
                        lemlib::TurnToPointParams params, bool async) {
  const auto transformed = transformPose(target);
  lemlib::Chassis::turnToPoint(
      transformed.x, transformed.y, timeout,
      {
          .forwards = params.forwards,
          .direction = transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::turnToHeading(float theta, int timeout,
                          lemlib::TurnToHeadingParams params, bool async) {
  const auto transformed = transformHeading(theta);
  lemlib::Chassis::turnToHeading(
      transformed, timeout,
      {
          .direction = transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::swingToPoint(lemlib::Pose target, lemlib::DriveSide lockedSide,
                         int timeout, lemlib::SwingToPointParams params,
                         bool async) {
  const auto transformed = transformPose(target);
  lemlib::Chassis::swingToPoint(
      transformed.x, transformed.y, transformDriveSide(lockedSide), timeout,
      {
          .forwards = params.forwards,
          .direction = transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::swingToHeading(float theta, lemlib::DriveSide lockedSide,
                           int timeout, lemlib::SwingToHeadingParams params,
                           bool async) {
  const auto transformed = transformHeading(theta);
  lemlib::Chassis::swingToHeading(
      transformed, transformDriveSide(lockedSide), timeout,
      {
          .direction = transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::moveToPose(lemlib::Pose target, int timeout,
                       lemlib::MoveToPoseParams params, bool async) {
  const auto transformed = transformPose(target);
  lemlib::Chassis::moveToPose(transformed.x, transformed.y, transformed.theta,
                              timeout,
                              {
                                  .forwards = params.forwards,
                                  .horizontalDrift = params.horizontalDrift,
                                  .lead = params.lead,
                                  .maxSpeed = params.maxSpeed,
                                  .minSpeed = params.minSpeed,
                                  .earlyExitRange = params.earlyExitRange,
                              },
                              async);
}

void Robot::moveToPoint(lemlib::Pose target, int timeout,
                        lemlib::MoveToPointParams params, bool async) {
  const auto transformed = transformPose(target);
  lemlib::Chassis::moveToPoint(transformed.x, transformed.y, timeout,
                               {
                                   .forwards = params.forwards,
                                   .maxSpeed = params.maxSpeed,
                                   .minSpeed = params.minSpeed,
                                   .earlyExitRange = params.earlyExitRange,
                               },
                               async);
}