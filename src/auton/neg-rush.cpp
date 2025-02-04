#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "subsystems/intake.h"
#include "auton.h"
#include "subsystems/ladybrown.h"
#include "leds/functions.h"

using namespace dimensions::all;
using namespace DIR;

using namespace auton::util;

void negativeRushFunc(ALLIANCE alliance);

auton::Auton auton::autons::negativeRush = {
    .label = (char*)"Negative Half AWP",
    .run = negativeRushFunc,
    .labelForController = (char*)"neg half",
    .description =
        (char*)"Grabs mogo, scores 4 rings on it, and touches ladder.",
};

// Autonomous for negative rush (side of the field with negative corners)
void negativeRushFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  // Back of bot blue driver station
  // Back of bot aligned with side of the puzzling closest to the mogo
  // Centered on tile edge
  bot.setPose({-TILE * 2 - dimensions::robot::DRIVE_LENGTH / 2 - .5,
               TILE * 2 - DRIVE_WIDTH / 2 - .5, RED_STATION});

  // Drop intake
  bot.intakeLift.retract();
  bot.led.autonMessage(0xFF0000, 1000, pros::millis());

  // Go to mogo
  const lemlib::Pose mogo {-TILE, TILE, RED_STATION + 30};
  bot.moveToPose(mogo, 2000,
                 {
                     .forwards = false,
                     .maxSpeed = 64,
                 });
  // Wait until near mogo
  waitUntilDistToPose(mogo, 9, 0, true);
  // Clamp
  bot.mogoClamp.retract();
  bot.led.autonMessage(0x00FF00, 1000, pros::millis());

  bot.waitUntilDone();
  pros::delay(500);

  // Intake preload to mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Intake safe stack
  bot.moveToPoint({-TILE - 1, TILE * 2 - 1}, 3000);
  bot.waitUntilDone();
  pros::delay(500);

  // Intake 1st ring on line
  bot.moveToPose({-DRIVE_LENGTH / 2 - 2, TILE * 2 - 4, BLUE_STATION}, 2000);

  // Back away from line
  bot.waitUntilDone();
  tank(-32, -32, 300); // Back away straight
  bot.moveToPose({-TILE - 3, TILE * 2 + 2, BLUE_STATION}, 2000,
                 {.forwards = false});

  // Intake 2nd ring on line
  bot.moveToPose({-DRIVE_LENGTH / 2 - 2, TILE * 2 + 5, BLUE_STATION}, 2000);

  // Back away
  bot.waitUntilDone();
  tank(-32, -32, 450); // Back away straight
  // Go to bar
  bot.moveToPose(
      lemlib::Pose {-TILE / 2, TILE / 2, RED_STATION + 45} +
          lemlib::Pose::fromPolar(DRIVE_LENGTH / 2 - 11, RED_STATION + 45),
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

  // // Go backwards to line up for filling goal
  // bot.moveToPoint(
  //     {
  //         -TILE * 2,
  //         TILE * 2,
  //     },
  //     1000, {.forwards = false, .maxSpeed = 64});
  // const lemlib::Pose allyWallStack {-TILE * 2, 0};
  // bot.turnToPoint(allyWallStack, 1000);
  //
  // // Push over ally wall stack
  // bot.moveToPoint(allyWallStack, 1000, {.maxSpeed = 64});
  // bot.waitUntil(10);
  // bot.intake.stop();
  // bot.waitUntilDone();
}
