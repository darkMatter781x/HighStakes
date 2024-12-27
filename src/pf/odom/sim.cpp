#include "pf/odom/sim.h"
#include "lemlib/pose.hpp"
#include "pf/util/stats.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <random>

const lemlib::Pose& SimOdom::getPose() const { return m_pose; }

const lemlib::Pose& SimOdom::getRealPose() const { return m_realPose; }

void SimOdom::setPose(lemlib::Pose pose) { m_pose = pose; }

std::normal_distribution<float> distNoise(0, 1);

void SimOdom::update() {
  static lemlib::Pose prevRealPose = m_realPose;
  /** Pose had pf not existed */
  static lemlib::Pose rawOdomPose = m_realPose;
  if (hasCalibrated()) {
    // printf("[TELEMETRY.PF.ACTUAL_POSE]:%f,%f,%f\n", m_realPose.x,
    // m_realPose.y,
    //        m_realPose.theta);
    // printf("[TELEMETRY.PF.RAW_ODOM_POSE]:%f,%f,%f\n", rawOdomPose.x,
    //        rawOdomPose.y, rawOdomPose.theta);

    // m_realPose.theta = M_PI / 4;
    // m_realPose.x += .1 * cos(m_realPose.theta);
    // m_realPose.y += .1 * sin(m_realPose.theta);

    // Drive forward in circle with radius 12 inches
    /** Number of iterations for a full rotation */
    constexpr int period = 200;
    /** Magic coefficient to make the circle radius 12 inches. See
     * https://www.desmos.com/calculator/7veyhyceh6 */
    constexpr float magicCoefficient = 12 * M_PI / period;
    m_realPose.theta -= 2 * M_PI / period;
    // printf("theta_odom:\t%f\n", m_realPose.theta);
    m_realPose.x += magicCoefficient * cos(m_realPose.theta);
    m_realPose.y += magicCoefficient * sin(m_realPose.theta);

    // Update pose
    lemlib::Pose displacement {m_realPose.x - prevRealPose.x,
                               m_realPose.y - prevRealPose.y,
                               m_realPose.theta - prevRealPose.theta};
    // Inject noise
    displacement.x += .05 * distNoise(stats::rng);
    displacement.y += .05 * distNoise(stats::rng);

    rawOdomPose.x += displacement.x;
    rawOdomPose.y += displacement.y;
    rawOdomPose.theta += displacement.theta;

    m_pose.x += displacement.x;
    m_pose.y += displacement.y;
    m_pose.theta += displacement.theta;

    // Update prev real pose
    prevRealPose = m_realPose;
  }
}

bool SimOdom::hasCalibrated() const {
  if (m_calibrationStartTime == -1) return false;
  return (pros::millis() - m_calibrationStartTime) > 1000;
}

SimOdom::SimOdom() : m_pose {0, 0}, m_realPose {0, 0} {}

void SimOdom::calibrate() { m_calibrationStartTime = pros::millis(); }
