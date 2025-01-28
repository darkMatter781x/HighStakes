#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "subsystems/intake.h"
#include "auton.h"
#include "subsystems/ladybrown.h"
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

  const Pose startPose {-TILE * 2 + 1. / 2 - DRIVE_WIDTH / 2 - 5. / 2,
                        0 - TILE + 1. / 2 + 8.5, AUDIENCE};
  // Start on positive side of alliance wall stake, facing away from referee
  // TODO: Implement this
  bot.setPose(startPose, 1);

  // === Score preload on ally stake

  // Make as precise as possible
  bot.lateralLargeExit.range = 0.4;
  bot.lateralLargeExit.time = 50;
  bot.lateralSmallExit.range = 0.6;
  bot.lateralSmallExit.time = 300;

  const Pose allyStake = {MIN_X, 2, 0};

  // Disable MCL during scoring on ally stack
  bot.localization.enabled = false;

  // Get in front of ally stake
  bot.moveToPoint(bot.getPose().withY(allyStake.y), 2000,
                  {.forwards = false, .maxSpeed = 56});
  // Turn to ally stake
  bot.turnToPoint(allyStake, 500,
                  {
                      .forwards = false,
                      .maxSpeed = 64,
                  });
  bot.moveToPoint(bot.getPose().withX(-TILE * 2.5 - 2), 1000,
                  {.forwards = false, .maxSpeed = 72});
  bot.waitUntilDone();

  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // Score on ally stack
  bot.intake.intakeBothTo(Intake::MOGO);
  pros::delay(450);

  // Dont get intake stuck
  bot.intake.outtake();

  // Enable MCL during scoring on ally stack
  bot.localization.enabled = false;

  // === Intake top ring of stack next to ally stake

  // Intake stack
  bot.intakeLift.retract();
  bot.moveToPose({-TILE * 1.5, TILE * .5, 45}, 2500, {.maxSpeed = 72});

  // Give intake time unstuck
  pros::delay(50);
  bot.intake.intakeBothTo(Intake::MOGO);

  // Wait until red ring is sensed
  waitUntil([&] {
    return bot.intake.getSensedRing() == COLOR::RED || !bot.isInMotion();
  });

  // Outtake just in case blue ring is at bottom of intake
  bot.intake.outtake();
  bot.cancelMotion();

  // Back up away from mogo
  tank(-64, -64, 50);
  // Stop intake
  bot.intake.stop();
  tank(-64, -64, 150);

  // === Clamp mogo on negative side

  const Pose negMogo = {-TILE, TILE};

  // Turn towards mogo
  bot.turnToPoint(negMogo, 500,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 30});

  // Make it more precise
  bot.lateralLargeExit.range = 0.4;
  bot.lateralLargeExit.time = 50;
  bot.lateralSmallExit.range = 1;
  bot.lateralSmallExit.time = 500;

  // Push back of bot into mogo and index it
  bot.moveToPose(negMogo.withTheta(RED_STATION - 30), 1500,
                 {.forwards = false});

  // Clamp mogo once
  bot.waitUntilDone();
  bot.mogoClamp.retract();

  // Give robot time to clamp
  pros::delay(250);

  // Reset settings
  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // === Score held ring and negative safe stack on mogo

  const Pose negSafeStack = {-TILE, TILE * 2};

  // Turn towards stack
  bot.swingToPoint(negSafeStack, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Score on mogo
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(bot.getColor(), Intake::KICK);

  // Go towards negative safe stack and intake it
  bot.moveToPoint(negSafeStack, 1000, {.minSpeed = 64, .earlyExitRange = 4});

  // === Deposit mogo in positive half and push alliance bot

  bot.moveToPose({-TILE * 2.5, -TILE * 2.5, REFEREE}, 2500,
                 {.forwards = false, .minSpeed = 64, .earlyExitRange = 5});
  bot.waitUntilDone();

  // Release mogo
  bot.intake.outtake();
  bot.mogoClamp.extend();

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
  bot.lady.goToIDLE_SCORE();

  // Go to ladder
  bot.moveToPose({-TILE * .5, -TILE * .5, AUDIENCE + 45}, 2000,
                 {.forwards = false, .minSpeed = 64, .earlyExitRange = 5});

  // Stop intake to prevent it from getting stuck on ladder
  bot.waitUntil(12);
  bot.intake.stop();

  // Drop lady at last second
  waitUntil(
      [startTime] { return (pros::millis() * msec - startTime) > 14.8 * sec; });
  bot.lady.goToIDLE();
  pros::delay(100);
  bot.lady.setState(LadyBrown::COAST);
}
