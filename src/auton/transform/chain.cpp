#include "auton/transform.h"

namespace auton {
lemlib::Pose ChainedTransform::transformPose(lemlib::Pose oldPose) {
  return m_t2->transformPose(m_t1->transformPose(oldPose));
}

float ChainedTransform::transformHeading(float oldAngle) {
  return m_t2->transformHeading(m_t1->transformHeading(oldAngle));
}

lemlib::DriveSide
ChainedTransform::transformDriveSide(lemlib::DriveSide oldDriveSide) {
  return m_t2->transformDriveSide(m_t1->transformDriveSide(oldDriveSide));
}

lemlib::AngularDirection
ChainedTransform::transformAngDir(lemlib::AngularDirection oldAngDir) {
  return m_t2->transformAngDir(m_t1->transformAngDir(oldAngDir));
}

ChainedTransform::ChainedTransform(std::shared_ptr<Transformation> transformA,
                                   std::shared_ptr<Transformation> transformB)
  : m_t1(transformA), m_t2(transformB) {}

std::shared_ptr<ChainedTransform>
ChainedTransform::make(std::shared_ptr<Transformation> transformA,
                       std::shared_ptr<Transformation> transformB) {
  return std::make_shared<ChainedTransform>(transformA, transformB);
}

} // namespace auton