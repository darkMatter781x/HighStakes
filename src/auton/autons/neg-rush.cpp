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

  // Default settings
  const auto defaultAngularPID = bot.angularPID;

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
      Pose::fromPolar(-DRIVE_LENGTH / 2 + 3, BLUE_STATION - 25) + centerRing +
      Pose {0, (bot.getColor() == COLOR::RED ? 1.5f : 0.f)};

  const Pose safeStack {-TILE, TILE * 2};
  // Go past mogo and safe ring stack
  {
    // const float targetHeading = BLUE_STATION - 30;
    bot.moveToPose(centerRingTarget, 1000,
                   {
                       .horizontalDrift = 2,
                       .lead = .4,
                       .minSpeed = 96,
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
    return bot.getPose().x > (safeStack.x - 5) || !bot.isInMotion();
  });
  bot.cancelMotion();

  // Attempt to make angle more accurate
  bot.angularPID.kP = 2;
  bot.angularPID.kD = 48;

  // Intake center ring and grab other ring
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.moveToPose(centerRingTarget, 1000, {.horizontalDrift = 2, .lead = .4});

  waitUntil([&] {
    return bot.intake.getSensedRing() == bot.getColor() ||
           !bot.isInMotion() /*  ||
bot.getPose().distance(centerRingTarget) < 7 */
        ;
  });
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();
  bot.cancelMotion();

  // // Go straight to ring center
  // bot.moveToPoint(centerRingTarget, 1000);
  // waitUntil([&] {
  //   return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  // });
  // if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();
  // bot.cancelMotion();

  bot.angularPID = defaultAngularPID;

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
  pros::delay(250);

  // === Intake safe stack

  bot.moveToPoint(safeStack + Pose {0, 5}, 3000, {.maxSpeed = 72});
  bot.waitUntilDone();
  pros::delay(250);

  // === Intake corner ring into LB

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
  pros::delay(200);

  if (bool runAllianceStake = true) {
    // === Intake ring next to alliance wall stake

    const Pose allyStake {-TILE * 3 + 1, 0};

    /** Stack in front of ally stake  */
    const Pose allyStackTarget {-TILE * 2 - 1, +DRIVE_LENGTH / 2 + 2, AUDIENCE};

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
    // Give time for intake to stop
    pros::delay(250);
    // Move LB to score
    bot.lady.flip();

    // Back away from wall stake to score on it
    const Pose allyStakeTarget = Pose::fromPolar(-7.5, bot.getPose().theta) +
                                 bot.getPose() + Pose {0, -5};
    bot.moveToPoint(allyStakeTarget, 1500, {.forwards = false, .maxSpeed = 64});
    bot.waitUntilDone();
    pros::delay(300);
    tank(-64, -64, 100, 0);
    // // === Touch ladder

    // // Move to ladder
    // bot.moveToPose({-TILE + (8. / 2 + DRIVE_LENGTH / 2 + 1),
    //                 8. / 2 + DRIVE_LENGTH / 2 + 1, RED_STATION},
    //                1500,
    //                {
    //                    .forwards = false,
    //                });
    // // Raise LB to go above ladder rung
    // bot.waitUntil(8);
    // bot.lady.goToIdleScore();

    // bot.waitUntilDone();

    // // // Stop intake to stop it from getting stuck on ladder
    // // bot.intake.stop();

    // // // Use hang to touch ladder rung from below
    // // bot.waitUntilDone();
    // // bot.lady.flip();

  } else {
    // // === Touch ladder (no alliance stake shenanigans)

    // const Pose ladderTouchTarget {-TILE * .5, TILE * .5, REFEREE - 45};
    // // Go to ladder
    // bot.moveToPose(ladderTouchTarget, 5000,
    //                {.forwards = false, .maxSpeed = 80 /* Go slow to give the
    //                auto time to intake */});

    // // Stop intake to prevent it from getting stuck on ladder
    // waitUntilDistToPose(ladderTouchTarget, 12);
    // bot.intake.stop();

    // // Raise lady brown to touch ladder
    // bot.lady.goToIdleScore();

    // // Drop LB at last second
    // waitUntil([startTime] {
    //   return (pros::millis() * msec - startTime) > 14.8 * sec;
    // });
    // bot.lady.goToIdle();
    // pros::delay(100);
    // bot.lady.setState(LadyBrown::COAST);
  }

  const Pose corner {MIN_X, MIN_Y};
  // === Deposit mogo in positive corner
  const Pose clearCornerTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2 - RING_DIAM * sqrt(2) - 2,
                      AUDIENCE + 45) +
      corner;
  bot.moveToPose(clearCornerTarget, 3000,
                 {
                     .horizontalDrift = 2,
                     .lead = .4,
                     .minSpeed = 127,
                 });
  bot.waitUntil(8);
  bot.lady.goToIdleScore();

  // Deploy doinker corner once close
  waitUntilDistToPose(clearCornerTarget, 6, 0, true);
  switch (bot.getColor()) {
    case COLOR::RED: bot.rightDoinker.extend(); break;
    case COLOR::BLUE: bot.leftDoinker.extend(); break;
  }
  bot.waitUntilDone();

  // Clear corner
  bot.turnToPoint(corner, 750, {.forwards = false, .minSpeed = 64});
}
