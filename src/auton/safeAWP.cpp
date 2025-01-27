#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "subsystems/intake.h"
#include "auton.h"
#include "subsystems/ladybrown.h"

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

  // Start on positive side of alliance wall stake, facing away from referee
  // TODO: Implement this
  bot.setPose({-TILE * 2 + 1. / 2 - DRIVE_WIDTH / 2 - 5. / 2,
               0 - TILE + 25. / 2 + 1. / 2, AUDIENCE},
              1);

  // === Score preload on ally stake

  // Get in front of ally stake
  bot.moveToPoint(bot.getPose().withY(3), 3000, {.forwards = false});
  // Turn to ally stake
  bot.turnToPoint({MIN_X, 2}, 500,
                  {
                      .forwards = false,
                      .maxSpeed = 64,
                  });
  // Make as precise as possible
  bot.lateralLargeExit.range = 0.25;
  bot.lateralSmallExit.range = 0.25;
  bot.moveToPoint(bot.getPose().withX(MIN_X + 8.35), 1000,
                  {.forwards = false, .maxSpeed = 48});
  bot.waitUntilDone();
  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // Score on ally stack
  bot.intake.intakeBothTo(Intake::MOGO);
  pros::delay(450);

  // Dont get intake stuck
  bot.intake.outtake();

  // Intake stack
  bot.intakeLift.retract();
  bot.moveToPose({-TILE * 1.5, TILE * .5, 45}, 1500, {.maxSpeed = 64});

  // Give intake time unstuck
  pros::delay(50);
  bot.intake.intakeBothTo(Intake::MOGO);

  // Wait until red ring is sensed
  waitUntil([&] {
    return bot.intake.getSensedRing() == COLOR::RED || !bot.isInMotion();
  });

  bot.intake.stop();

  // === Intake top ring of stack next to ally stake, and score on ally stake

  // Clamp mogo on negative side

  // Score negative safe stack on mogo

  // Deposit mogo in positive half

  // Clamp mogo on positive side

  // Score positive safe stack on mogo

  // Touch ladder
}
