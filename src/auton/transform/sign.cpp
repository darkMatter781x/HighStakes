#include "auton/transform.h"
#include "dimensions.h"

namespace auton {

bool SignTransform::needsTransform() const { return sign != normalSign; }

lemlib::Pose SignTransform::transformPose(lemlib::Pose oldPose) {
  if (needsTransform())
    return {oldPose.x, -oldPose.y, transformHeading(oldPose.theta)};
  else return oldPose;
}

lemlib::DriveSide
SignTransform::transformDriveSide(lemlib::DriveSide original) {
  constexpr auto LEFT = lemlib::DriveSide::LEFT;
  constexpr auto RIGHT = lemlib::DriveSide::RIGHT;
  if (needsTransform()) switch (original) {
      case LEFT: return RIGHT;
      case RIGHT: return LEFT;
    }
  return original;
}

lemlib::AngularDirection
SignTransform::transformAngDir(lemlib::AngularDirection original) {
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

float SignTransform::transformHeading(float oldHeading) {
  if (needsTransform()) return dimensions::field::DIR::STANDS - oldHeading;
  else return oldHeading;
}

SignTransform::SignTransform(SIGN sign, SIGN normalSign)
  : sign(sign), normalSign(normalSign) {}
} // namespace auton