#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/timer.hpp"
#include "robot.h"

using namespace dimensions::all;
using namespace DIR;
using lemlib::Pose;

using namespace auton::util;

void positiveRushFunc(ALLIANCE alliance);

auton::Auton auton::autons::positiveRush = {
    .label = (char*)"Positive Rush",
    .run = positiveRushFunc,
    .labelForController = (char*)"mogo rush",
    .description = (char*)"Rushes mogo and scores 1 ring on each of 2 mogos, "
                          "and 1 ring on "
                          "alliance stake, for a total of 9pts.",
};

// Autonomous for positive rush (side of the field with positive corners)
void positiveRushFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  bot.intake.numOfMogoRings = 0;

  // Back of bot facing blue driver station
  // Back of bot aligned with side of the puzzling furthest to the mogo
  // Right side of bot aligned with close side of puzzling
  bot.setPose({-TILE * 2 - dimensions::robot::DRIVE_LENGTH / 2 - .5,
               -TILE - DRIVE_WIDTH / 2 - .5, RED_STATION});

  // Drop intake
  bot.intakeLift.retract();

  // Go to mogo
  const lemlib::Pose mogo {-TILE, -TILE, RED_STATION - 25};
  bot.moveToPose(mogo, 2000,
                 {
                     .forwards = false,
                     .maxSpeed = 64,
                 });
  // Wait until near mogo
  waitUntilDistToPose(mogo, 9, 0, true);
  // Clamp
  bot.mogoClamp.retract();

  bot.waitUntilDone();
  pros::delay(500);

  // Intake preload to mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Intake safe stack
  bot.moveToPoint({-TILE - 1, -TILE * 2}, 3000);

  bot.waitUntilDone();
  // Give more time to intake
  pros::delay(500);

  /** Stack of rings next to alliance wall stake. Top is red */
  const Pose allyStakeStack {-TILE * 2, 0};
  // Turn to ring stack
  bot.turnToPoint(allyStakeStack, 1000, {.maxSpeed = 64, .earlyExitRange = 15});
  bot.waitUntilDone();

  // Lift intake to intake rings
  bot.intakeLift.extend();

  // Go to ring stack to intake it
  bot.moveToPose(allyStakeStack.withTheta(REFEREE) +
                     Pose {bot.getColor() == COLOR::RED ? 2.f : 4.f, -10},
                 4000, {.maxSpeed = 64});
  bot.waitUntilDone();

  // Drop intake on ring
  bot.intakeLift.retract();

  lemlib::Timer partiallyIntakedTimer {1000};
  bool ringSensed = false;
  while (!partiallyIntakedTimer.isDone()) {
    if (ringSensed == false && bot.intake.getSensedRing() == bot.getColor()) {
      printf("sensed ring!\n");
      ringSensed = true;
      tank(-72, -72, 200, 0);
      break;
    }
    pros::delay(10);
  }
  stop();
  if (ringSensed == true) pros::delay(300);
  bot.intake.stop();

  // Back up to unstuck intake
  tank(-32, -32, 300, 0);
  stop();
  if (bot.intake.numOfMogoRings < 3) {
    // Resume intaking
    bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
    bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  }
  pros::delay(1000);

  // Touch bar
  bot.moveToPose(
      lemlib::Pose {-TILE / 2 - 2, -TILE / 2 + 2, RED_STATION - 45} +
          lemlib::Pose::fromPolar(DRIVE_LENGTH / 2 - 10, RED_STATION - 45),
      2000, {.forwards = false});

  // Move lady brown to prepare to touch bar
  bot.waitUntil(12);
  bot.lady.setState(LadyBrown::SCORE);
  bot.intake.stop(); // Don't hit bar with hook

  // Move lady brown to touch ladder
  bot.waitUntilDone();
  bot.lady.setState(LadyBrown::IDLE);
  pros::delay(500);
  bot.lady.setState(LadyBrown::COAST);
}
