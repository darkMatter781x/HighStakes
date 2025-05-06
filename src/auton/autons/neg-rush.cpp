#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "auton/autons.h"
#include "auton/util.h"
#include "units/units.hpp"
#include <cmath>
#include <cstddef>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;

void negativeRushFunc(ALLIANCE alliance);

auton::Auton auton::autons::negativeRush = {
    .label = (char*)"Ring Rush",
    .run = negativeRushFunc,
    .labelForController = (char*)"ring rush",
    .description = (char*)"Front of bot faces opposite driver station. "
                          "Starts on tile edge closest to mogo. "
                          "Set using Shreya's setting stick. "
                          "Yoink center ring, "
                          "score on mogo, "
                          "intake safe stack, ally stack, preload, "
                          "and touch ladder.",
};

/** @returns Whether a ring is detected in the lady brown by using intake
 * velocity. */
static bool isRingPushedIntoLadyBrown() {
  const float velo = bot.m_config.motors.intake.get_actual_velocity();
  return std::abs(velo) < 1;
}

static void pulseIntake() {
  bot.intake.stop();
  pros::delay(50);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  pros::delay(150);
  // Stop intake to prevent intake from cooking itself
  bot.intake.stop();
}

static void waitUntilRingIsInLB(bool& ringSensed, bool& ringCollected,
                                bool checkMotionRunning = false,
                                size_t timeout = INT_MAX) {
  waitUntil(
      [&ringSensed, &ringCollected, checkMotionRunning] {
        ringSensed |= bot.intake.getSensedRing() != std::nullopt;
        ringCollected |= ringSensed && isRingPushedIntoLadyBrown();
        return ringCollected || (!bot.isInMotion() && checkMotionRunning);
      },
      0, timeout);
}

// Autonomous for negative rush (side of the field with negative corners)
void negativeRushFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  const Time startTime = pros::millis() * msec;

  // Disable MCL
  bot.localization.enabled = false;

  bot.intake.dejamEnabled = true;

  // Reset ring counters
  bot.intake.numOfKickRings = 0;
  bot.intake.numOfMogoRings = 0;
  bot.intake.numOfRings = 0;
  bot.intake.numOfLiftRings = 0;

  // Front of bot facing opposite driver station
  // Aligned using Shreyas's setting tool
  // Starts on tile edge closest to mogo
  const Pose startPose {-TILE * 2 - 27. / 2. / 2. - 0.2,
                        TILE * 1 + DRIVE_WIDTH / 2 + 1.9, BLUE_STATION - 9.5};
  bot.setPose(startPose);

  const Pose preloadRing {-TILE * 2, TILE * 1};

  // Drop intake
  bot.intakeLift.retract();

  // === Yoink center ring

  /** Center ring 2nd closest to referee on our side. */
  const Pose centerRing {-RING_DIAM / 2, TILE * 2 - RING_DIAM / 2};
  const Pose centerRingTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2 + 4, BLUE_STATION - 25) + centerRing +
      Pose {0, (bot.getColor() == COLOR::RED ? 2.f : 0.f)};

  const Pose safeStack {-TILE, TILE * 2};
  // Go past mogo and safe ring stack
  {
    // const float targetHeading = BLUE_STATION - 30;
    bot.moveToPose(centerRingTarget, 1000,
                   {
                       .horizontalDrift = 2,
                       .lead = .4,
                       .minSpeed = 100,
                       .earlyExitRange = 6,
                   });
  }
  // Wait a bit to deploy doinkers
  bot.waitUntil(5);
  // Extend doinker to grab center ring closest to Referee
  switch (bot.getColor()) {
    case COLOR::BLUE: bot.rightDoinker.extend(); break;
    case COLOR::RED: bot.leftDoinker.extend(); break;
  }

  // Once past safe stack, move on
  waitUntil([safeStack] {
    return bot.getPose().x > (safeStack.x - 2) || !bot.isInMotion();
  });
  bot.cancelMotion();

  // Intake center ring and grab other ring
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);
  bot.moveToPose(centerRingTarget, 1000, {.horizontalDrift = 2, .lead = .4});

  waitUntil([] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  });
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();
  bot.cancelMotion();

  // === Score center ring on mogo

  // Push back of bot into mogo and index it
  const lemlib::Pose mogo {-TILE, TILE - 1, BLUE_STATION - 30};
  bot.moveToPoint(mogo.withTheta(BLUE_STATION - 30), 1500,
                  {.forwards = false, .maxSpeed = 72});

  // Stop intake once ring is sensed
  waitUntil([] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  });
  bot.intake.stop();

  // Clamp mogo
  bot.waitUntilDone();
  bot.mogoClamp.retract();

  // Release ring in doinker
  bot.rightDoinker.retract();
  bot.leftDoinker.retract();
  const Pose doinkedRing = bot.getPose() +
                           Pose {
                               13,
                               ((bot.getColor() == COLOR::RED ? 1.f : 1.f) *
                                (+DRIVE_WIDTH / 2.f - .25f)),
                           }
                               .rotate((90 - bot.getPose().theta) * M_PI / 180);
  printf("doinkedRing: %f, %f\n", doinkedRing.x, doinkedRing.y);
  // Give robot time to clamp
  pros::delay(350);

  // Intake center ring to mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);

  // === Intake doinked ring

  bot.moveToPoint(doinkedRing, 2000, {.maxSpeed = 72});
  bot.waitUntilDone();
  // Give time to intake
  pros::delay(100);

  // === Intake safe stack

  bot.moveToPoint(safeStack + Pose {0, 5}, 3000, {.maxSpeed = 72});
  bot.waitUntilDone();
  pros::delay(100);

  // === Intake corner ring into LB

  // const Pose cornerTarget =
  //     Pose::fromPolar(-DRIVE_LENGTH / 2 - RING_DIAM / 2 + 3, REFEREE - 15) +
  //     Pose {MIN_X + RING_DIAM / 2, MAX_Y - RING_DIAM / 2};
  const Pose cornerTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2 - RING_DIAM / 2 + 2, REFEREE - 45) +
      Pose {MIN_X, MAX_Y} + Pose {0, 1};

  // Turn toward corner, avoiding red ring of just intaked stack
  bot.swingToPoint(cornerTarget, lemlib::DriveSide::RIGHT, 500,
                   {
                       .minSpeed = 64,
                       .earlyExitRange = 20,
                   });
  bot.waitUntilDone();

  // Go quickly towards corner
  bot.moveToPose(cornerTarget, 2500,
                 {.horizontalDrift = 2, .lead = .6, .minSpeed = 72});
  // Once near corner, go slower to avoid messing up odometry
  waitUntilDistToPose(cornerTarget, 18, 0, true);
  bot.cancelMotion();
  // Ram into corner, in order to wedge intake into corner
  bot.moveToPose(cornerTarget, 2500,
                 {.horizontalDrift = 2, .lead = .6, .minSpeed = 48});
  bot.waitUntilDone();

  // Continue pushing into corner
  tank(48, 48, 10, 0);

  // Once ring is sensed, we can continue on
  waitUntil([] { return bot.intake.getSensedRing() == bot.getColor(); }, 0,
            500);

  // Adjust odom pose
  bot.setPose(bot.getPose() + Pose {+3 - 1, -3 + .5});

  // === Pickup preload ring

  // Turn towards preload ring
  bot.swingToPoint(preloadRing, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 96, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Move to preload ring
  bot.moveToPose((preloadRing + Pose {0, DRIVE_LENGTH / 2}).withTheta(AUDIENCE),
                 1500);
  bot.waitUntilDone();
  pros::delay(300);

  // === Intake ring next to alliance wall stake

  const Pose allyStake {-TILE * 3 + 1, 0};

  /** Stack in front of ally stake  */
  const Pose allyStackTarget {-TILE * 2, +DRIVE_LENGTH / 2, AUDIENCE};

  // Move to stack
  bot.moveToPose(allyStackTarget, 1500);
  // Once prior ring intaked or near stack, lift intake
  waitUntil(
      [&] {
        return bot.getColor() == bot.intake.getSensedRing() ||
               bot.getPose().distance(allyStackTarget) < 15 ||
               !bot.isInMotion();
      },
      0, 500);
  bot.intakeLift.extend();
  // Once near stack, raise LB to collect ring
  waitUntilDistToPose(allyStackTarget, 3, 0, true);
  // Disable dejamming when intaking to LB
  bot.intake.dejamEnabled = false;
  // Raise LB to collect ring
  bot.lady.collect();

  bool lbRingSensed = false;
  bool lbRingCollected = false;
  // Wait until ring is sensed
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, true);
  bot.cancelMotion();

  // Drop intake on stack to intake the ring
  bot.intakeLift.retract();

  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, false, 500);
  // Once ring is collected, stop intake
  if (lbRingCollected) bot.intake.stop();

  // === Score on alliance wall stake

  // Turn in order to score on ally stake, clearing front of stake
  // Turn backwards to avoid red ring at the bottom of ally stack
  bot.swingToPoint(allyStake, lemlib::DriveSide::LEFT, 500);
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, true);
  // Once ring is collected, stop intake
  if (lbRingCollected) bot.intake.stop();
  bot.waitUntilDone();

  // Index onto wall stake
  tank(72, 72, 750, 0);
  stop();

  // Wait a little longer to intake ring to LB
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, false, 500);
  // Once ring is collected, stop intake
  bot.intake.stop();
  // Move LB to score
  bot.lady.score();

  // Back away from wall stake to score on it
  const Pose allyStakeTarget =
      Pose::fromPolar(-7, bot.getPose().theta) + bot.getPose() + Pose {0, -5};
  bot.moveToPoint(allyStakeTarget, 1500, {.forwards = false, .maxSpeed = 64});
  bot.waitUntilDone();
  pros::delay(300);

  // === Touch ladder

  // Move to ladder
  bot.moveToPose({-TILE + 3, 8. / 2 + DRIVE_LENGTH / 2 + 3, RED_STATION}, 1500,
                 {.forwards = false});
  // Raise LB to go above ladder rung
  bot.waitUntil(8);
  bot.lady.goToIdleScore();
  // Stop intake to stop it from getting stuck on ladder
  bot.intake.stop();

  // Use hang to touch ladder rung from below
  bot.waitUntilDone();
  bot.lady.flip();
}
