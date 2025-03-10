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

  const Pose startPose {-TILE * 2 - 4.4, -(TILE * 1 + DRIVE_WIDTH / 2 + .5),
                        BLUE_STATION};
  bot.setPose(startPose);

  // === Score preload on middle mogo

  const Pose midMogo {0, -TILE * 2};
  const Pose safeStack {-TILE, -TILE * 2};

  // Go past mogo and safe ring stack
  {
    bot.moveToPoint((safeStack + Pose {4, RING_DIAM / 2 + DRIVE_WIDTH / 2 - 2})
                        .withTheta(BLUE_STATION),
                    1000, {.minSpeed = 64, .earlyExitRange = 6});
  }

  // Collect preload into LB
  bot.lady.collect();
  pros::delay(400); // Wait for LB to settle
  bot.intake.intakeBothTo(Intake::MOGO); // Intake preload
  bot.waitUntilDone();

  // Go to actual mogo
  const Pose midMogoTarget =
      Pose::fromPolar(-15, BLUE_STATION + 30) + midMogo + Pose {-1, -1};
  bot.lateralLargeExit.time = 10;
  bot.lateralSmallExit.time = 10;
  bot.lateralLargeExit.range = 1;
  bot.lateralSmallExit.range = 1;

  bot.moveToPoint(midMogoTarget, 2500);

  // Move hook out of the way
  bot.intake.outtake();
  pros::delay(30);
  bot.intake.stop();

  // Once ready, score on mogo
  bot.waitUntilDone();
  bot.lady.score();
  pros::delay(400); // Give time to score

  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // Tip the mid mogo
  bot.lady.flip();
  bot.turnToHeading(AUDIENCE + 30, 500,
                    {.minSpeed = 127, .earlyExitRange = 10});

  // Turn towards mogo
  const Pose posMogo {-TILE, -TILE};
  bot.turnToPoint(posMogo, 500, {.forwards = false});
  bot.waitUntilDone();

  // Go to mogo
  bot.moveToPoint(posMogo, 500, {.forwards = false});

  // Retract mogo
  bot.waitUntil(9);
  bot.mogoClamp.retract();

  pros::delay(500);

  const Pose posSafeStack = {-TILE, -TILE * 2};

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Turn towards stack
  bot.swingToPoint(posSafeStack, lemlib::DriveSide::RIGHT, 500);
  bot.waitUntilDone();

  bot.moveToPoint(posSafeStack, 1000);

  return;

  pros::delay(600);

  // Stop intake from heating up
  bot.intake.stop();

  // Push ring down onto stake
  bot.lady.goToIdle();
  pros::delay(100);

  // Back straight out of stake
  tank(-96, -96, 400, 4);

  // === Intake top ring of stack next to ally stake

  //   const Pose posMogo = {-TILE, -TILE};

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

//   const Pose posSafeStack = {-TILE, -TILE * 2};

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Adjust odometry
  bot.setPose(bot.getPose() + Pose {-5, 0}, 1);

  // Turn towards stack
  bot.swingToPoint(posSafeStack, lemlib::DriveSide::LEFT, 500);
  bot.waitUntilDone();

  // Go towards positive safe stack and intake it
  bot.moveToPose(Pose::fromPolar(
                     -DRIVE_LENGTH / 2 + 3,
                     90 - lemlib::radToDeg(bot.getPose().angle(posSafeStack))) +
                     posSafeStack,
                 2000);
  bot.waitUntilDone();

  // === Yoink ring from corner

  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 - 4, AUDIENCE + 45) +
                     Pose {MIN_X, MIN_Y},
                 1500);

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
