#include "auton/transform.h"
#include "lemlib/pose.hpp"

namespace auton {
lemlib::Pose ChainedTransform::transformPose(lemlib::Pose oldPose) {
  return m_t2->transformPose(m_t1->transformPose(oldPose));
}
} // namespace auton