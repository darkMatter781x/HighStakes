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

  const Pose startPose {-TILE * 2 - 8.5, TILE - 9.5, RED_STATION - 45 - 3};
  bot.setPose(startPose);

  // === Score preload on ally stake

  // Collect preload into LB
  bot.lady.collect();
  pros::delay(1000); // Wait for LB to settle
  bot.intake.intakeBothTo(Intake::MOGO); // Intake preload

  // Wait until preload collected
  pros::delay(300);

  // Prime LB
  bot.lady.runIntoAlly();
  // Prevent hooks from getting stuck on the mogo
  bot.intake.outtake();
  // Wait for LB to be to ready to score
  pros::delay(1000);

  // Push ring into stake
  tank(64, 64, 300);

  // Stop intake from heating up
  bot.intake.stop();

  // Push ring down onto stake
  bot.lady.score();
  pros::delay(100);

  // Back out of stake
  tank(-64, -64, 600, 0);
  // Reset lady brown
  bot.lady.goToIdle();
  stop();

  // === Intake top ring of stack next to ally stake

  const Pose negMogo = {-TILE, TILE};
  const Pose allySafeStack {-TILE * 2, 0};

  // Intake stack
  bot.intakeLift.extend();
  bot.moveToPose(
      (allySafeStack + Pose {0, DRIVE_LENGTH / 2 + 2}).withTheta(AUDIENCE),
      1000);
  bot.intake.intakeBothTo(Intake::MOGO);

  // Once on top of stack, drop intake to intake
  bot.waitUntilDone();
  bot.intakeLift.retract();

  // Wait until red ring is sensed
  waitUntil([&] { return bot.intake.getSensedRing() == COLOR::RED; }, 0, 1500);
  // Then stop the intake to hold the ring
  bot.intake.stop();
  // === Clamp mogo on negative side

  // // Make it more precise
  // bot.lateralLargeExit.range = 0.4;
  // bot.lateralLargeExit.time = 50;
  // bot.lateralSmallExit.range = 1;
  // bot.lateralSmallExit.time = 500;

  // Push back of bot into mogo and index it
  bot.moveToPose(negMogo.withTheta(RED_STATION - 30), 1500,
                 {.forwards = false});

  // Clamp mogo once
  bot.waitUntilDone();
  bot.mogoClamp.retract();
  // Reset mogo ring counter
  bot.intake.numOfMogoRings = 0;

  // Give robot time to clamp
  pros::delay(250);

  // // Reset settings
  // bot.lateralLargeExit = defaultLatLargeExit;
  // bot.lateralSmallExit = defaultLatSmallExit;

  // === Score held ring and negative safe stack on mogo

  const Pose negSafeStack = {-TILE, TILE * 2};

  // Turn towards stack
  bot.turnToPoint(negSafeStack, 500, {.minSpeed = 72, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Score on mogo
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(bot.getColor(), Intake::KICK);

  // Go towards negative safe stack and intake it
  bot.moveToPoint(negSafeStack, 1000, {.minSpeed = 64, .earlyExitRange = 4});

  // === Deposit mogo in positive half and push alliance bot

  bot.moveToPose({-TILE * 2.5, -TILE * 2.5, REFEREE}, 2500,
                 {.forwards = false, .minSpeed = 64, .earlyExitRange = 5});

  // Wait until ring is intaked to release mogo
  waitUntil([] { return bot.intake.numOfMogoRings >= 2 || !bot.isInMotion(); });

  // Release mogo to make sure mogo hitting rings doesn't stop us
  bot.mogoClamp.extend();
  bot.intake.outtake();

  bot.waitUntilDone();

  // === Clamp mogo on positive side

  const Pose posMogo = {-TILE, -TILE};

  // Get out of corner
  bot.moveToPoint({-TILE * 2.5, -TILE}, 1000,
                  {.minSpeed = 64, .earlyExitRange = 5});

  // Turn towards mogo
  bot.swingToPoint(posMogo, lemlib::DriveSide::RIGHT, 500,
                   {.forwards = false, .minSpeed = 64, .earlyExitRange = 30});

  // Go pick up mogo
  bot.moveToPose(posMogo.withTheta(RED_STATION - 30), 1000,
                 {.forwards = false});
  bot.waitUntilDone();

  // Clamp mogo
  bot.mogoClamp.retract();
  pros::delay(250);

  // === Score positive safe stack on mogo
  const Pose posSafeStack = {-TILE, -TILE * 2};

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Turn towards stack
  bot.swingToPoint(posSafeStack, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});
  // Go towards positive safe stack and intake it
  bot.moveToPoint(posSafeStack, 1000, {.minSpeed = 64, .earlyExitRange = 4});
  bot.waitUntilDone();

  // === Touch ladder

  // Raise lady brown to touch ladder
  bot.lady.goToIdleScore();

  // Go to ladder
  bot.moveToPose({-TILE * .5, -TILE * .5, AUDIENCE + 45}, 2000,
                 {.forwards = false, .minSpeed = 64, .earlyExitRange = 5});

  // Stop intake to prevent it from getting stuck on ladder
  bot.waitUntil(12);
  bot.intake.stop();

  // Drop lady at last second
  waitUntil(
      [startTime] { return (pros::millis() * msec - startTime) > 14.8 * sec; });
  bot.lady.goToIdle();
  pros::delay(100);
  bot.lady.setState(LadyBrown::COAST);
}
