#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/util.hpp"
#include "robot.h"
#include "units/units.hpp"

using namespace dimensions::all;
using namespace DIR;
using lemlib::Pose;

using namespace auton::util;

void safePositiveFunc(ALLIANCE alliance);

auton::Auton auton::autons::safePositive = {
    .label = (char*)"Positive Safe",
    .run = safePositiveFunc,
    .labelForController = (char*)"pos safe",
    .description = (char*)"Start on negative side of alliance wall stake, "
                          "facing towards referee."
                          "ally-1, mogo-2, touch ladder",
};

// Autonomous for positive rush (side of the field with positive corners)
void safePositiveFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  bot.intake.numOfMogoRings = 0;

  const Time startTime = pros::millis() * msec;

  // Start on negative side of alliance wall stake, facing towards referee
  const Pose startPose {-TILE * 2 + 1. / 2 - DRIVE_WIDTH / 2 - 5. / 2,
                        -(0 - TILE + 1. / 2 + 8.5), REFEREE};
  bot.setPose(startPose, 1);

  // === Score preload on ally stake

  // Make as precise as possible
  bot.lateralLargeExit.range = 0.4;
  bot.lateralLargeExit.time = 50;
  bot.lateralSmallExit.range = 0.6;
  bot.lateralSmallExit.time = 300;

  const Pose allyStake = {MIN_X, -2, 0};

  // Disable MCL during scoring on ally stake
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
  bot.waitUntilDone();

  // Back up towards ally stake
  bot.moveToPoint(bot.getPose().withX(-TILE * 2.5 - 1.75), 1000,
                  {.forwards = false, .maxSpeed = 72});
  bot.waitUntilDone();

  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // Score on ally stack
  bot.intake.intakeBothTo(Intake::MOGO);
  pros::delay(450);

  // Dont get intake stuck
  bot.intake.outtake();

  // Enable MCL after scoring on ally stack
  bot.localization.enabled = false;
  // Give MCL time to stabilize
  pros::delay(10);

  // === Intake top ring of stack next to ally stake

  const Pose posMogo = {-TILE, -TILE};

  // Drop intake in order to intake mogo
  bot.intakeLift.retract();

  const Pose allyStackTarget =
      posMogo + Pose::fromPolar(-(DRIVE_LENGTH / 2 + 9), BLUE_STATION + 30);

  // Intake stack
  bot.moveToPoint(allyStackTarget, 750, {.maxSpeed = 72});

  // Give intake time to unstuck
  pros::delay(50);
  bot.intake.intakeBothTo(Intake::MOGO);

  // Wait for a bit after intaking first ring
  waitUntilDistToPose(allyStackTarget, 5, 0, true);
  bot.cancelMotion();
  pros::delay(500);

  // Continue on
  bot.moveToPoint(allyStackTarget, 1500, {.maxSpeed = 72});

  // Wait until red ring is sensed
  waitUntil([&] { return bot.intake.getSensedRing() == bot.getColor(); }, 0,
            2000);

  // Outtake just in case blue ring is at bottom of intake
  bot.intake.outtake();
  bot.cancelMotion();

  // Back up away from mogo
  tank(-64, -64, 50);
  // Stop intake
  bot.intake.stop();
  tank(-64, -64, 150);

  // === Clamp mogo on positive side

  // Turn towards mogo
  bot.turnToPoint(posMogo, 500,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 45});

  // Make it more precise
  bot.lateralLargeExit.range = 0.4;
  bot.lateralLargeExit.time = 50;
  bot.lateralSmallExit.range = 1;
  bot.lateralSmallExit.time = 500;

  // Push back of bot into mogo and index it
  bot.moveToPose(posMogo.withTheta(RED_STATION + 30), 1500,
                 {.forwards = false});

  // Clamp mogo
  bot.waitUntilDone();
  bot.mogoClamp.retract();
  // Reset mogo ring counter
  bot.intake.numOfMogoRings = 0;

  // Give robot time to clamp
  pros::delay(250);

  // Reset settings
  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // // === Intake bottom ring of corner stack

  // // Go around blue ring
  // bot.moveToPoint({-TILE * 2.5, -TILE * 2}, 1000,
  //                 {.minSpeed = 96, .earlyExitRange = 3});

  // // Go to corner
  // const Pose corner {MIN_X, MIN_Y};
  // bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 10, AUDIENCE + 45) +
  //                    corner,
  //                1500, {.lead = .5});

  // // Deploy doinker
  // bot.waitUntil(12);
  // bot.doinker.extend();

  // // Doink stack
  // bot.turnToHeading(bot.getColor() == COLOR::RED ? AUDIENCE : RED_STATION,
  // 500,
  //                   {
  //                       .minSpeed = 64,
  //                   });
  // bot.waitUntilDone();
  // bot.doinker.retract();
  // bot.waitUntilDone();

  // bot.moveToPoint(bot.getPose().withY(MIN_Y + DRIVE_LENGTH / 2 + 3), 500);

  // === Score positive safe stack on mogo

  const Pose posSafeStack = {-TILE, -TILE * 2};

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Adjust odometry
  bot.setPose(bot.getPose() + Pose {-5, 0}, 1);

  // Turn towards stack
  bot.swingToPoint(posSafeStack, lemlib::DriveSide::LEFT, 500);
  bot.waitUntilDone();

  // Go towards positive safe stack and intake it
  bot.moveToPose(Pose::fromPolar(
                     -DRIVE_LENGTH / 2 + 8,
                     90 - lemlib::radToDeg(bot.getPose().angle(posSafeStack))) +
                     posSafeStack,
                 2000);
  bot.waitUntilDone();

  // === Touch ladder

  // Go to ladder
  bot.moveToPose({-TILE * .5, -TILE * .5, AUDIENCE + 45}, 2000,
                 {.forwards = false});

  // Stop intake to prevent it from getting stuck on ladder
  bot.waitUntil(12);
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
