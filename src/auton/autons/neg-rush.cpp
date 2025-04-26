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
    .label = (char*)"Negative Rush",
    .run = negativeRushFunc,
    .labelForController = (char*)"neg rush",
    .description = (char*)"Front of bot faces opposite driver station. "
                          "Starts on tile edge closest to mogo. "
                          "Set using Shreya's setting stick. "
                          "Yoink center ring, "
                          "score on mogo, "
                          "intake safe stack, ally stacthenk, preload, "
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

  bot.intake.dejamEnabled = false;

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
      Pose::fromPolar(-DRIVE_LENGTH / 2 + 2, BLUE_STATION - 25) + Pose {0, 3} +
      centerRing;
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
  bot.waitUntil(12);
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
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.moveToPose(centerRingTarget, 1000, {.horizontalDrift = 2, .lead = .4});

  waitUntil([] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  });
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();

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
                               ((bot.getColor() == COLOR::RED ? -1.f : 1.f) *
                                (+DRIVE_WIDTH / 2.f - .25f)),
                           }
                               .rotate((90 - bot.getPose().theta) * M_PI / 180);
  printf("doinkedRing: %f, %f\n", doinkedRing.x, doinkedRing.y);
  // Give robot time to clamp
  pros::delay(350);

  // Intake center ring to mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // === Intake doinked ring

  bot.moveToPoint(doinkedRing, 2000, {.maxSpeed = 64});
  bot.waitUntilDone();
  // Give time to intake
  pros::delay(400);

  // === Intake safe stack

  bot.moveToPoint(safeStack + Pose {0, 2}, 3000, {.maxSpeed = 64});
  bot.waitUntilDone();
  pros::delay(500);

  // === Intake corner ring into LB

  // const Pose cornerTarget =
  //     Pose::fromPolar(-DRIVE_LENGTH / 2 - RING_DIAM / 2 + 3, REFEREE - 15) +
  //     Pose {MIN_X + RING_DIAM / 2, MAX_Y - RING_DIAM / 2};
  const Pose cornerTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2 - RING_DIAM / 2 + 3, REFEREE - 45) +
      Pose {MIN_X, MAX_Y} + Pose {0, 1};

  // Turn toward corner, avoiding red ring of just intaked stack
  bot.swingToPoint(cornerTarget, lemlib::DriveSide::RIGHT, 500,
                   {
                       .minSpeed = 64,
                       .earlyExitRange = 20,
                   });
  bot.waitUntilDone();
  const size_t mogoRingsAfterSafeStack = bot.intake.numOfMogoRings;

  // Ram into corner, in order to wedge intake into corner
  bot.moveToPose(cornerTarget, 2500,
                 {.horizontalDrift = 2, .lead = .6, .minSpeed = 56});
  waitUntil([&] {
    return mogoRingsAfterSafeStack < bot.intake.numOfMogoRings ||
           bot.intake.numOfMogoRings == 3 || !bot.isInMotion();
  });
  // Raise LB to collect ring
  bot.lady.collect();

  bool lbRingSensed = false;
  bool lbRingCollected = false;
  // Wait until ring is sensed before moving on
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, true);
  bot.waitUntilDone();

  // Continue pushing into corner
  tank(72, 72, 30, 0);
  // Wait until ring is sensed before moving on
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, false, 500);

  // Adjust odom pose
  // bot.setPose(bot.getPose() + Pose {-2, -5});

  // === Pickup preload ring

  // Turn towards preload ring
  bot.swingToPoint(preloadRing, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 96, .earlyExitRange = 30});
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, true);
  // If ring is sensed, stop intake, and raise LB 
  if (lbRingCollected) { bot.intake.stop(); }
  bot.waitUntilDone();

  // Move to preload ring
  bot.moveToPose((preloadRing + Pose {0, DRIVE_LENGTH / 2}).withTheta(AUDIENCE),
                 2000, {.maxSpeed = 64});
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, true);
  // If ring is sensed, stop intake, and raise LB
  bool lbRaised = false;
  if (lbRingCollected && !lbRaised) {
    lbRaised = true;
    pulseIntake();
    pulseIntake();
    bot.intake.stop();
    bot.lady.goToIdleScore();
    // Give time to raise LB
    pros::delay(400);
    // Continue intaking
    bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
    bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  }

  bot.waitUntilDone();
  // Give time to intake
  pros::delay(500);

  // === Intake ring next to alliance wall stake

  const Pose allyStake {-TILE * 3 + 1, 0};
  /** Distance from ally stake needed to score on it */
  const float distanceToAllyStake = 13;

  /** Stack in front of ally stake  */
  const Pose allyStackTarget =
      (allyStake +
       Pose {DRIVE_WIDTH / 2 + distanceToAllyStake, +DRIVE_WIDTH / 2})
          .withTheta(AUDIENCE);

  // Lift intake and move to stack
  bot.intakeLift.extend();
  bot.moveToPose(allyStackTarget, 1500);
  waitUntilRingIsInLB(lbRingSensed, lbRingCollected, true);
  // If ring is sensed, stop intake, and raise LB
  if (lbRingCollected && !lbRaised) {
    lbRaised = true;
    bot.intake.stop();
    bot.lady.goToIdleScore();
    // Give time to raise LB
    pros::delay(400);
    // Continue intaking
    bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
    bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  }
  bot.waitUntilDone();

  // Drop intake on stack
  bot.intakeLift.retract();

  // === Score on alliance wall stake

  // Use doinker to clear front of ally stake
  switch (bot.getColor()) {
    case COLOR::BLUE: bot.leftDoinker.extend(); break;
    case COLOR::RED: bot.rightDoinker.extend(); break;
  }
  // Give doinker time to extend
  pros::delay(200);

  // Turn in order to score on ally stake, clearing front of stake
  bot.swingToPoint(allyStake, lemlib::DriveSide::RIGHT, 500);
  bot.waitUntilDone();

  // Get doinker out of the way
  bot.leftDoinker.retract();
  bot.rightDoinker.retract();

  const Pose allyStakeTarget =
      Pose::fromPolar(-distanceToAllyStake, bot.getPose().theta) + allyStake;

  // Move to correct distance from ally stake
  bot.moveToPose(allyStakeTarget, 1500);
  bot.waitUntilDone();

  // Score on ally stake
  bot.lady.score();
  // Give time to score
  pros::delay(500);

  // === Touch ladder

  // Move to ladder
  bot.moveToPose({-TILE, TILE / 2, RED_STATION}, 1000, {.forwards = false});
  // Move LB to touch ladder once
  bot.waitUntil(8);
  bot.lady.goToIdleScore();
  return;

  // // === Intake corner ring

  // const Pose negCorner {MIN_X, MAX_Y};
  // const Pose negCornerTarget =
  //     Pose::fromPolar(-DRIVE_LENGTH / 2, REFEREE - 45) + negCorner;
  // bot.moveToPose(negCornerTarget, 2000, {.minSpeed = 56});
  // bot.waitUntilDone();

  // // === Intake preload

  // // Back up from corner
  // tank(-64, -64, 150, 4);

  // bot.swingToPoint(preloadRing, lemlib::DriveSide::RIGHT, 500,
  //                  {.minSpeed = 64, .earlyExitRange = 30});
  // bot.moveToPoint(preloadRing, 2000);
  // bot.waitUntilDone();
  // pros::delay(500);

  // // === Collect ring stack next to alliance stake into LB

  // // Place intake on top of stack
  // const Pose allyStack {-TILE * 2, 0};
  // bot.intakeLift.extend();
  // bot.moveToPose((allyStack + Pose {-3, DRIVE_LENGTH /
  // 2}).withTheta(AUDIENCE),
  //                1000);

  // // Raise LB to collect ring
  // bot.waitUntil(24);
  // bot.lady.collect();
  // bot.waitUntilDone();

  // // Drop intake on stack
  // bot.intakeLift.retract();

  // // Give time to pickup (300ms)
  // waitUntil([] { return isRingPushedIntoLadyBrown(); }, 0, 300);
  // if (isRingPushedIntoLadyBrown()) bot.intake.stop();

  // // Back up for 250ms
  // tank(-96, -96, 0, 0);
  // waitUntil([&] { return isRingPushedIntoLadyBrown(); }, 0, 50);
  // if (isRingPushedIntoLadyBrown()) bot.intake.stop();
  // tank(-24, -24, 0, 0);
  // waitUntil([&] { return isRingPushedIntoLadyBrown(); }, 0, 200);
  // if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();

  // // === Score ring on alliance stake

  // // Turn towards ally stake
  // const Pose allyStake {MIN_X + .5, 0};
  // bot.swingToPoint(allyStake, lemlib::DriveSide::LEFT, 750);
  // bot.waitUntilDone();

  // // Move to correct distance from ally stake
  // bot.moveToPoint(allyStake + Pose::fromPolar(-13, bot.getPose().theta) +
  //                     Pose {0, -1},
  //                 750);
  // bot.waitUntilDone();

  // // Score on ally stake
  // bot.lady.flip();
  // pros::delay(500);

  // tank(-96, -96, 100, 4);
  // bot.lady.goToIdle();
  // tank(-96, -96, 300, 4);

  // stop();

  // // === Go near pos corner
  // const Pose nearPosCornerTarget {-TILE * 2.5, -TILE * 1.75, REFEREE};
  // bot.turnToPoint(nearPosCornerTarget, 500, {.forwards = false});
  // bot.moveToPose(nearPosCornerTarget, 2000, {.forwards = false});
  // bot.lady.goToIdle();
  // return;

  // // === Doink pos corner stack

  // // Go around stack
  // bot.moveToPoint({-TILE * 1.6, 0}, 1000,
  //                 {.minSpeed = 127, .earlyExitRange = 6});

  // const Pose posCorner {MIN_X, MIN_Y};

  // // Move in front of pos corner stack
  // const Pose posCornerTarget = {-TILE * 2.5, -TILE * 2.5, AUDIENCE};
  // bot.moveToPose(posCornerTarget, 3000, {.lead = .5});

  // bot.waitUntil(12);
  // bot.lady.goToIdle();

  // bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  // bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // // Once near, deploy doinker
  // waitUntilDistToPose(posCornerTarget, 11, 0, true);
  // bot.leftDoinker.extend();

  // // Spin to doink rings
  // bot.turnToPoint(posCorner, 500,
  //                 {
  //                     .forwards = false,
  //                     .direction = AngDir::CW_CLOCKWISE,
  //                     .minSpeed = 127,
  //                 });
  // bot.waitUntilDone();

  // // === Push mogo into corner

  // // Release mogo
  // bot.mogoClamp.extend();

  // // Push mogo
  // const Pose depositMogoTarget =
  //     posCorner.withTheta(REFEREE + 45) +
  //     Pose::fromPolar(DRIVE_LENGTH / 2 + 3, REFEREE + 45);
  // bot.moveToPose(depositMogoTarget, 2000, {.forwards = false, .minSpeed =
  // 72});
}
