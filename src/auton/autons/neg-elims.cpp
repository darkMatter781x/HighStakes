#include "color.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "auton/autons.h"
#include "auton/util.h"
#include "subsystems/intake.h"
#include "units/units.hpp"

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;

static void negativeElimsFunc(ALLIANCE alliance);

auton::Auton auton::autons::negativeElims = {
    .label = (char*)"Negative Elims",
    .run = negativeElimsFunc,
    .labelForController = (char*)"neg ELIMS",
    .description = (char*)"Front of bot faces opposite driver station. "
                          "Starts on tile edge closest to mogo. "
                          "Set using Shreya's setting stick. "
                          "Yoink center ring, "
                          "score on mogo, "
                          "intake safe stack, ally stack, preload, "
                          "and touch ladder.",
};

// Autonomous for negative rush (side of the field with negative corners)
static void negativeElimsFunc(ALLIANCE alliance) {
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
  const Pose preloadRing {-TILE * 2 - 3, TILE * 1 - RING_DIAM / 2 - 1};

  // Drop intake
  //bot.intakeLift.retract();

  // === Yoink center ring

  const Pose centerRing {-RING_DIAM / 2 + 1, TILE * 2 - RING_DIAM / 2};
  const Pose safeStack {-TILE, TILE * 2};
  // Go past mogo and safe ring stack
  {
    bot.moveToPoint((safeStack + Pose {4.2, -RING_DIAM / 2 - DRIVE_WIDTH / 2 + 3})
                        .withTheta(BLUE_STATION),
                    1000, {.minSpeed = 72, .earlyExitRange = 6});
  }
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

  bot.moveToPoint(safeStack + Pose {1, -5}, 3000,
                  {.minSpeed = 48, .earlyExitRange = 4});

  // // TODO: Figure out how to doink for different alliances
  // // === Intake rings from negative corner stack

  const Pose negCorner {MIN_X, MAX_Y};
  const Pose goofyOtherSpot {-TILE * 1, TILE * 2 - 2};
  // // Get in front of stack
  const Pose negCornerTarget =
       Pose::fromPolar(-DRIVE_LENGTH / 2 - 8 +
                           (bot.getColor() == COLOR::RED ? 1 : 0),
                       REFEREE - 55) +
       negCorner;
  
  bot.intake.intakeTo(ALLIANCE::RED, Intake::MOGO);

  bot.moveToPose(goofyOtherSpot, 500, {.minSpeed = 42, .earlyExitRange=6});
  bot.waitUntilDone();
  //bot.turnToHeading(REFEREE - 55, 5000);
  bot.moveToPose(negCornerTarget, 2000, {.minSpeed = 42});
  pros::delay(500);

  // // == Doink stack
  // // Deploy doinker once near stack
  // waitUntilDistToPose(negCornerTarget, 11, 0, true);
  // bot.doinker.extend();
  // // Give time for doinker to deploy
  // pros::delay(150);

  // // Spin to doink
  // bot.turnToHeading(
  //     REFEREE - 45 + (bot.getColor() == COLOR::RED ? -60 : +80), 1000,
  //     {.direction = bot.getColor() == COLOR::RED ?
  //     AngDir::CCW_COUNTERCLOCKWISE
  //                                                : AngDir::CW_CLOCKWISE,
  //      .minSpeed = 127,
  //      .earlyExitRange = 5});
  // bot.waitUntilDone();

  // // Retract doinker after spinning
  // bot.doinker.retract();

  // // == Intake the rings
  // if (bot.getColor() == COLOR::BLUE) {
  //   // Intake
  //   bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  //   bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  //   // Move forward
  //   tank(64, 32, 500);
  // } // bot.moveToPoint(
  // //     bot.getPose() +
  // //         Pose::fromPolar(5, 90 - lemlib::radToDeg(bot.getPose().theta)),
  // //     500);

  // === Intake preload

  // Adjust odometry
  bot.setPose(bot.getPose());

  // Go to preload
  bot.moveToPose(preloadRing.withTheta(AUDIENCE), 2000, {.lead = .8});
  bot.waitUntilDone();
  pros::delay(700);

  // === Intake stack next to alliance stake to LB

  // Place intake on top of stack
  const Pose allyStack {-TILE * 2, 0};
  bot.intakeLift.extend();
  bot.moveToPose(
      (allyStack + Pose {0, DRIVE_LENGTH / 2 + 1}).withTheta(AUDIENCE), 1000);
  bot.waitUntilDone();

  // Drop intake on stack
  bot.intakeLift.retract();

  // Give time to intake
  pros::delay(1000);
  // // Raise LB
  // bot.lady.collect();
  // Back up to intake
  tank(-96, -96, 500);
  stop();

  // // === Score on ally stake
   const Pose scoreOnAllyStake {-TILE * 2.5 + 1.5, -2 , RED_STATION};
   bot.moveToPose(scoreOnAllyStake, 2500);
   bot.waitUntilDone();
   bot.intake.outtake();
   bot.lady.flip();

   // Give time to score LB
   pros::delay(600);
   // Back out to release LB
   tank(-127, -127, 300, 0);
   stop();

  // // === Touch ladder

   const Pose ladderTarget {-TILE * .5 - 1, TILE * .5 - 1, REFEREE - 45};
  // // Go to ladder
   bot.moveToPose(ladderTarget, 3000, {.forwards = false});

  // // Stop intake to prevent it from getting stuck on ladder
  // waitUntilDistToPose(ladderTarget, 12, 0, true);
  // bot.intake.stop();
  // // Raise lady brown to touch ladder
  // bot.lady.goToIdleScore();

  // // Drop LB at last second
  // waitUntil(
  //     [startTime] { return (pros::millis() * msec - startTime) > 14.8 * sec;
  //     });
  // bot.lady.goToIdle();
  // pros::delay(100);
  // bot.lady.setState(LadyBrown::COAST);

  // // === Go near pos corner
  // const Pose nearPosCornerTarget {-TILE * 2.5, -TILE * 1.75, REFEREE};
  // bot.turnToPoint(nearPosCornerTarget, 500, {.forwards = false});
  // bot.moveToPose(nearPosCornerTarget, 2000, {.forwards = false});
  // bot.lady.goToIdle();
  // return;

  // === Doink pos corner stack

  // Go around stack
  //bot.moveToPoint({-TILE * 1.6, 0}, 1000,
  //                {.minSpeed = 127, .earlyExitRange = 6});

  //const Pose posCorner {MIN_X, MIN_Y};

  // Move in front of pos corner stack
  //const Pose posCornerTarget = {-TILE * 2.5, -TILE * 2.5, AUDIENCE};
  //bot.moveToPose(posCornerTarget, 3000, {.lead = .5});

  //bot.waitUntil(12);
  //bot.lady.goToIdle();

  //bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  //bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Once near, deploy doinker
  //waitUntilDistToPose(posCornerTarget, 11, 0, true);
  // bot.doinker.extend();

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
