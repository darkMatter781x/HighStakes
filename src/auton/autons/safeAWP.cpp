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

  bot.moveToPoint(Pose::fromPolar(3, bot.getPose().theta) + bot.getPose(), 800);
  bot.waitUntilDone();

  // Move hook out of the way
  bot.intake.outtake();
  pros::delay(30);

  bot.lady.score();
  pros::delay(600);

  // Stop intake from heating up
  bot.intake.stop();

  // Push ring down onto stake
  bot.lady.goToIdle();
  pros::delay(100);

  // === Intake top ring of stack next to ally stake

  const Pose negMogo = {-TILE, TILE};
  const Pose allySafeStack {-TILE * 2, 0};

  // Back straight out of stake
  tank(-96, -96, 400, 4);

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
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 2, REFEREE) + negSafeStack,
                 1000
                 /* {.minSpeed = 64 , .earlyExitRange = 4} */);
  bot.waitUntilDone();
  // Give time to intake
  pros::delay(300);

  // === Deposit mogo in positive half and push alliance partner bot

  // Go around ally stake rings
  bot.moveToPoint({-TILE * 1.7, 0}, 800,
                  {.forwards = false, .minSpeed = 96, .earlyExitRange = 6});
  bot.waitUntilDone();

  // Release mogo to make sure mogo hitting rings doesn't stop us
  bot.mogoClamp.extend();
  // Stop intake to prevent hooking the mogo with hooks
  bot.intake.stop();

  // Deposit mogo
  bot.moveToPoint({-TILE * 2.5, -TILE * 2}, 2500,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});
  bot.waitUntilDone();

  // === Clamp mogo on positive side

  const Pose posMogo = {-TILE, -TILE};

  // Get out of corner
  bot.moveToPoint({-TILE * 2.5, -TILE}, 1000,
                  {.minSpeed = 48, .earlyExitRange = 5});

  // Turn towards mogo
  bot.swingToPoint(posMogo, lemlib::DriveSide::RIGHT, 750,
                   {.forwards = false, .minSpeed = 24, .earlyExitRange = 15});

  // Go pick up mogo
  bot.moveToPoint(posMogo, 1500, {.forwards = false, .maxSpeed = 64});

  // Wait until near mogo to clamp it
  waitUntilDistToPose(posMogo, 5, 0, true);
  bot.mogoClamp.retract();
  // Reset mogo ring counter
  bot.intake.numOfMogoRings = 0;

  pros::delay(250);
  bot.waitUntilDone();

  // === Score positive safe stack on mogo

  const Pose posSafeStack = {-TILE, -TILE * 2};

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Turn towards stack
  bot.swingToPoint(posSafeStack, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});
  // Go towards positive safe stack and intake it
  bot.moveToPoint(posSafeStack, 1000,
                  {.minSpeed = 64 /* , .earlyExitRange = 4 */});
  bot.waitUntilDone();

  // === Touch ladder

  // Raise hang to touch ladder
  bot.lady.flip();

  // Give time to intake
  pros::delay(250);

  // Go to ladder
  const Pose ladderTouchTarget {-TILE * .5, -TILE * .5, AUDIENCE + 45};
  bot.moveToPose(ladderTouchTarget, 2000,
                 {.forwards = false, .minSpeed = 127, .earlyExitRange = 6});
  // Stop intake to prevent it from getting stuck on ladder
  bot.waitUntil(12);
  bot.intake.stop();
  waitUntilDistToPose(ladderTouchTarget, 8, 0, true);
  bot.cancelMotion();

  //   // Drop lady at last second
  //   waitUntil(
  //       [startTime] { return (pros::millis() * msec - startTime) > 14.8 *
  //       sec; });
  //   bot.lady.goToIdle();
  //   pros::delay(100);
  //   bot.lady.setState(LadyBrown::COAST);
}
