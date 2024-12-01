#include "pf/odom/sim.h"

const lemlib::Pose& SimOdom::getPose() const { return m_pose; }

const lemlib::Pose& SimOdom::getRealPose() const { return m_realPose; }

void SimOdom::setPose(lemlib::Pose pose) { m_pose = pose; }

void SimOdom::update() {
  static lemlib::Pose prevRealPose = m_realPose;
  if (m_hasCalibrated) {
    // Drive forward
    m_realPose.y += .1;

    // Update pose
    lemlib::Pose displacement = m_realPose - prevRealPose;
    m_pose.x += displacement.x;
    m_pose.y += displacement.y;
    m_pose.theta += displacement.theta;

    // Update prev real pose
    prevRealPose = m_realPose;
  }
}

bool SimOdom::hasCalibrated() const { return m_hasCalibrated; }

SimOdom::SimOdom() : m_pose {0, 0}, m_realPose {0, 0} {}

void SimOdom::calibrate() { m_hasCalibrated = true; }
