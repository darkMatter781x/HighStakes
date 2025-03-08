#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"
#include "units/units.hpp"

using lemlib::Pose;

using namespace dimensions::all;
using namespace DIR;

using namespace auton::util;

void safeAWPFunc(ALLIANCE alliance);

auton::Auton auton::autons::safeAWP = {
    .label = (char*)"Safe AWP",
    .run = safeAWPFunc,
    .labelForController = (char*)"Safe AWP",
    .description = (char*)"Start on positive side of alliance wall stake, "
                          "facing away from ref. "
                          "Gets solo sig AWP. "
                          "Alliance wall stake - 2. "
                          "Neg mogo - 1. "
                          "Pos mogo - 1.",
};

// Autonomous for negative rush (side of the field with negative corners)
void safeAWPFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  bot.intake.numOfMogoRings = 0;
  bot.intake.numOfLiftRings = 0;

  const Time startTime = pros::millis() * msec;

  // Disable MCL during match auto
  bot.localization.enabled = false;

  const Pose startPose {-TILE * 2 - 6.66, TILE - 11.53, RED_STATION - 45 - 3};
  bot.setPose(startPose);

  // === Score preload on ally stake

  // Collect preload into LB
  bot.lady.collect();
  pros::delay(700); // Wait for LB to settle
  bot.intake.intakeBothTo(Intake::MOGO); // Intake preload

  // Wait until preload collected
  pros::delay(300);

  bot.moveToPoint(Pose::fromPolar(4, bot.getPose().theta) + bot.getPose(), 800);
  bot.waitUntilDone();

  // Move hook out of the way
  bot.intake.outtake();
  pros::delay(30);

  bot.lady.flip();
  pros::delay(400);

  // Stop intake from heating up
  bot.intake.stop();

  // Back straight out of stake
  tank(-96, -96, 100, 4);
  bot.lady.goToIdle();
  tank(-96, -96, 300, 4);

  // === Intake top ring of stack next to ally stake

  const Pose negMogo = {-TILE, TILE};
  const Pose allySafeStack {-TILE * 2, 0};

  // Turn to ally stake
  bot.turnToPoint(allySafeStack, 500);
  bot.waitUntilDone();
  // Reset lady brown
  bot.lady.goToIdle();
  stop();

  // Intake stack
  bot.intakeLift.extend();
  const Pose allySafeStackTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2, bot.getPose().theta) + allySafeStack;

  bot.moveToPoint(allySafeStackTarget, 1000, {.earlyExitRange = 2});
  bot.intake.intakeBothTo(Intake::MOGO);

  // Once on top of stack, drop intake to intake
  bot.waitUntilDone();
  bot.intakeLift.retract();

  // Give time to pickup
  pros::delay(300);

  // Back up for 200ms
  tank(-96, -96, 0, 0);
  waitUntil([&] { return bot.intake.getSensedRing() == bot.getColor(); }, 0,
            50);
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();
  tank(-24, -24, 0, 0);
  waitUntil([&] { return bot.intake.getSensedRing() == bot.getColor(); }, 0,
            150);
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();

  // === Clamp mogo on negative side

  // Push back of bot into mogo and index it
  bot.moveToPoint(negMogo, 1500, {.forwards = false, .maxSpeed = 64});

  // Wait until red ring is sensed
  waitUntil(
      [&] {
        return bot.intake.getSensedRing() == bot.getColor() ||
               !bot.isInMotion();
      },
      0, 1000);
  // Then stop the intake to hold the ring
  bot.intake.stop();

  // Wait until near mogo to clamp it
  waitUntilDistToPose(negMogo, 5, 0, true);
  bot.mogoClamp.retract();
  // Reset mogo ring counter
  bot.intake.numOfMogoRings = 0;

  // Give robot time to clamp
  pros::delay(250);
  bot.waitUntilDone();

  // === Score held ring and negative safe stack on mogo

  const Pose negSafeStack = {-TILE, TILE * 2};

  // Turn towards stack
  bot.turnToPoint(negSafeStack + Pose {-1, 0}, 500,
                  {.minSpeed = 72, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Score on mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Go towards negative safe stack and intake it
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 3, REFEREE) + negSafeStack,
                 1000
                 /* {.minSpeed = 64 , .earlyExitRange = 4} */);
  bot.waitUntilDone();
  // Give time to intake
  pros::delay(300);

  // // Intake center ring
  // const Pose centerRing {-RING_DIAM / 2, TILE * 2 + RING_DIAM / 2};
  // bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 2, BLUE_STATION + 30) +
  //                    centerRing,
  //                1000, {});

  // === Touch ladder

  // Raise hang to touch ladder
  bot.lady.flip();

  // Give time to intake
  pros::delay(250);

  // Go to ladder
  const Pose ladderTouchTarget {-TILE * .5 + 1, -(-TILE * .5 - 1),
                                REFEREE - 45};
  bot.moveToPose(
      ladderTouchTarget, 2000,
      {.forwards = false, .maxSpeed = 64, .minSpeed = 36, .earlyExitRange = 6});
  // Stop intake to prevent it from getting stuck on ladder
  bot.waitUntil(12);
  bot.intake.stop();
  waitUntilDistToPose(ladderTouchTarget, 8, 0, true);
  bot.cancelMotion();
}
