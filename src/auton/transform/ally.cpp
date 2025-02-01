#include "auton/transform.h"
#include "color.h"

namespace auton {
bool AllianceTransform::needsTransform() const {
  return alliance != normalAlliance;
}

lemlib::Pose AllianceTransform::transformPose(lemlib::Pose oldPose) {
  if (needsTransform())
    return {-oldPose.x, oldPose.y, transformHeading(oldPose.theta)};
  else return oldPose;
}

lemlib::DriveSide
AllianceTransform::transformDriveSide(lemlib::DriveSide original) {
  constexpr auto LEFT = lemlib::DriveSide::LEFT;
  constexpr auto RIGHT = lemlib::DriveSide::RIGHT;
  if (needsTransform()) switch (original) {
      case LEFT: return RIGHT;
      case RIGHT: return LEFT;
    }
  return original;
}

lemlib::AngularDirection
AllianceTransform::transformAngDir(lemlib::AngularDirection original) {
  constexpr auto CCW = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;
  constexpr auto CW = lemlib::AngularDirection::CW_CLOCKWISE;
  constexpr auto AUTO = lemlib::AngularDirection::AUTO;
  if (needsTransform()) switch (original) {
      case CCW: return CW;
      case CW: return CCW;
      case AUTO: return AUTO;
    }
  return original;
}

float AllianceTransform::transformHeading(float oldHeading) {
  if (needsTransform()) return -oldHeading;
  else return oldHeading;
}

AllianceTransform::AllianceTransform(ALLIANCE alliance, ALLIANCE normalAlliance)
  : alliance(alliance), normalAlliance(normalAlliance) {}
} // namespace auton