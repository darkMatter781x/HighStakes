#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "auton/autons.h"
#include "auton/util.h"
#include "units/units.hpp"

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
  return std::abs(velo) < 0.5;
}

// Autonomous for negative rush (side of the field with negative corners)
void negativeRushFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  const Time startTime = pros::millis() * msec;

  // Disable MCL
  bot.localization.enabled = false;

  // Front of bot facing opposite driver station
  // Aligned using Shreyas's setting tool
  // Starts on tile edge closest to mogo
  const Pose startPose {-TILE * 2 - 4.4, TILE * 1 + DRIVE_WIDTH / 2 + .5,
                        BLUE_STATION};
  bot.setPose(startPose);
  const Pose preloadRing {-TILE * 2, TILE * 1 - RING_DIAM / 2};

  // Drop intake
  bot.intakeLift.retract();

  // === Yoink center ring

  const Pose centerRing {-RING_DIAM / 2, TILE * 2 - RING_DIAM / 2};
  const Pose safeStack {-TILE, TILE * 2};
  // Go past mogo and safe ring stack
  {
    // const float targetHeading = BLUE_STATION - 30;
    bot.moveToPoint((safeStack + Pose {6, -RING_DIAM / 2 - DRIVE_WIDTH / 2 + 2})
                        .withTheta(BLUE_STATION),
                    1000, {.minSpeed = 127, .earlyExitRange = 6});
  }
  // bot.led.autonMessage(0x00FF00, 500);
  // Once past safe stack, move on
  waitUntil([safeStack] {
    return bot.getPose().x > (safeStack.x) || !bot.isInMotion();
  });
  bot.led.autonMessage(0x00FFFF, 500);
  bot.cancelMotion();

  // Intake center ring
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 2, BLUE_STATION - 30) +
                     centerRing,
                 1000, {});

  waitUntil([] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  });
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();

  // === Score center ring on mogo

  // Push back of bot into mogo and index it
  const lemlib::Pose mogo {-TILE, TILE, BLUE_STATION - 30};
  bot.moveToPose(mogo.withTheta(BLUE_STATION - 30), 1500, {.forwards = false});

  // Stop intake once ring is sensed
  waitUntil([] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  });
  bot.intake.stop();

  // Clamp mogo
  bot.waitUntilDone();
  bot.mogoClamp.retract();

  // Give robot time to clamp
  pros::delay(250);

  // Intake center ring to mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // === Intake safe stack

  bot.moveToPoint({-TILE - 1, TILE * 2 - 1}, 3000);
  bot.waitUntilDone();
  pros::delay(500);

  // === Intake corner ring

  const Pose negCorner {MIN_X, MAX_Y};
  const Pose negCornerTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2, REFEREE - 45) + negCorner;
  bot.moveToPose(negCornerTarget, 2000, {.minSpeed = 56});
  bot.waitUntilDone();

  // === Intake preload

  // Back up from corner
  tank(-64, -64, 150, 4);

  bot.swingToPoint(preloadRing, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 64, .earlyExitRange = 30});
  bot.moveToPoint(preloadRing, 2000);
  bot.waitUntilDone();
  pros::delay(500);

  // === Collect ring stack next to alliance stake into LB

  // Place intake on top of stack
  const Pose allyStack {-TILE * 2, 0};
  bot.intakeLift.extend();
  bot.moveToPose((allyStack + Pose {-3, DRIVE_LENGTH / 2}).withTheta(AUDIENCE),
                 1000);

  // Raise LB to collect ring
  bot.waitUntil(24);
  bot.lady.collect();
  bot.waitUntilDone();

  // Drop intake on stack
  bot.intakeLift.retract();

  // Give time to pickup (300ms)
  waitUntil([] { return isRingPushedIntoLadyBrown(); }, 0, 300);
  if (isRingPushedIntoLadyBrown()) bot.intake.stop();

  // Back up for 250ms
  tank(-96, -96, 0, 0);
  waitUntil([&] { return isRingPushedIntoLadyBrown(); }, 0, 50);
  if (isRingPushedIntoLadyBrown()) bot.intake.stop();
  tank(-24, -24, 0, 0);
  waitUntil([&] { return isRingPushedIntoLadyBrown(); }, 0, 200);
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();

  // === Score ring on alliance stake

  // Turn towards ally stake
  const Pose allyStake {MIN_X + .5, 0};
  bot.swingToPoint(allyStake, lemlib::DriveSide::LEFT, 750);
  bot.waitUntilDone();

  // Move to correct distance from ally stake
  bot.moveToPoint(allyStake + Pose::fromPolar(-13, bot.getPose().theta) +
                      Pose {0, -1},
                  750);
  bot.waitUntilDone();

  // Score on ally stake
  bot.lady.flip();
  pros::delay(500);

  tank(-96, -96, 100, 4);
  bot.lady.goToIdle();
  tank(-96, -96, 300, 4);

  stop();

  // === Go near pos corner
  const Pose nearPosCornerTarget {-TILE * 2.5, -TILE * 1.75, REFEREE};
  bot.turnToPoint(nearPosCornerTarget, 500, {.forwards = false});
  bot.moveToPose(nearPosCornerTarget, 2000, {.forwards = false});
  bot.lady.goToIdle();
  return;

  // === Doink pos corner stack

  // Go around stack
  bot.moveToPoint({-TILE * 1.6, 0}, 1000,
                  {.minSpeed = 127, .earlyExitRange = 6});

  const Pose posCorner {MIN_X, MIN_Y};

  // Move in front of pos corner stack
  const Pose posCornerTarget = {-TILE * 2.5, -TILE * 2.5, AUDIENCE};
  bot.moveToPose(posCornerTarget, 3000, {.lead = .5});

  bot.waitUntil(12);
  bot.lady.goToIdle();

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Once near, deploy doinker
  waitUntilDistToPose(posCornerTarget, 11, 0, true);
  bot.leftDoinker.extend();

  // Spin to doink rings
  bot.turnToPoint(posCorner, 500,
                  {
                      .forwards = false,
                      .direction = AngDir::CW_CLOCKWISE,
                      .minSpeed = 127,
                  });
  bot.waitUntilDone();

  // === Push mogo into corner

  // Release mogo
  bot.mogoClamp.extend();

  // Push mogo
  const Pose depositMogoTarget =
      posCorner.withTheta(REFEREE + 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 3, REFEREE + 45);
  bot.moveToPose(depositMogoTarget, 2000, {.forwards = false, .minSpeed = 72});
}
