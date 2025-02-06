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
                          "intake safe stack, ally stack, preload, "
                          "and touch ladder.",
};

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

  // === Intake preload

  bot.moveToPoint(preloadRing, 2000);
  bot.waitUntilDone();
  pros::delay(500);

  // === Intake stack next to alliance stake

  // Place intake on top of stack
  const Pose allyStack {-TILE * 2, 0};
  bot.intakeLift.extend();
  bot.moveToPose((allyStack + Pose {-3, DRIVE_LENGTH / 2}).withTheta(AUDIENCE),
                 1000);
  bot.waitUntilDone();

  // Drop intake on stack
  bot.intakeLift.retract();
  pros::delay(500);

  // === Touch ladder

  const Pose ladderTarget {-TILE * .5 - 3, TILE * .5 + 1, REFEREE - 45};
  // Go to ladder
  bot.moveToPose(ladderTarget, 3000, {.forwards = false, .maxSpeed = 64});

  // Stop intake to prevent it from getting stuck on ladder
  waitUntilDistToPose(ladderTarget, 12, 0, true);
  bot.intake.stop();
  // Raise lady brown to touch ladder
  bot.lady.goToIdleScore();

  // Drop LB at last second
  waitUntil(
      [startTime] { return (pros::millis() * msec - startTime) > 14.8 * sec; });
  bot.lady.goToIdle();
  pros::delay(100);
  bot.lady.setState(LadyBrown::COAST);
}
