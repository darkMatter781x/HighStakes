#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"
#include "units/units.hpp"

using lemlib::Pose;

using namespace dimensions::all;
using namespace DIR;

using namespace auton::util;

void posSafeFunction(ALLIANCE alliance);

auton::Auton auton::autons::posSafe = {
    .label = (char*)"Neg Safe",
    .run = posSafeFunction,
    .labelForController = (char*)"NEG Neil",
    .description = (char*)"Start on positive side of alliance wall stake, "
                          "facing away from ref. "
                          "Gets solo sig AWP. "
                          "Alliance wall swtake - 2. "
                          "Neg mogo - 1. "
                          "Pos mogo - 1.",
};

void posSafeFunction(ALLIANCE alliance){
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
  // bot states with the lady brown in the collect state
  bot.lady.collect();
  // Start pose of the bot
  const Pose startPose {-TILE * 2 - 13.1 - 1, -TILE + 11.2 - 1,
                        (REFEREE - 53.36 + 20.f)};
  bot.setPose(startPose);

  const Pose safeGoal = { - TILE, - TILE };
  const Pose safeStack = { - TILE, -TILE * 2 };
  const Pose posCorner = { MIN_X, MIN_Y };
  const Pose centerRing {0 + RING_DIAM/2, 0 - RING_DIAM/2};

  // === scores on the alliance stake and get the mogo
  // flips the lb and scores on the alliance stake
  bot.lady.flip();
  pros::delay(300);
  // backs away from the alliance stake and moves to the mobile goal
  bot.moveToPoint(safeGoal - Pose { -4, 0 }, 1000, {.forwards = false, .maxSpeed = 96});
  // resets the lady brown
  bot.waitUntil(3);
  bot.lady.goToIdle();
  bot.waitUntilDone();
  bot.tank(-32, -32);
  pros::delay(500);
  bot.mogoClamp.extend();
  pros::delay(250);



  // === gets the center ring
  // calculates the target pose needed to doinked the ring
  Pose targetCenterRing { Pose::fromPolar(-12.5, BLUE_STATION ) + centerRing + Pose { 0 , - DRIVE_WIDTH / 2}};
  // turns toward the center ring
  bot.turnToPoint(targetCenterRing, 1000);
  bot.waitUntilDone();
  // moves the intake lift out of the way
  bot.intakeLift.extend();
  // moves towards the center ring
  bot.moveToPoint(targetCenterRing.withTheta(BLUE_STATION), 1000);
  bot.waitUntilDone();
  // deploys the doinker depending on the alliance color
  switch (bot.getColor()) {
    case ALLIANCE::BLUE: bot.leftDoinker.extend();
    case ALLIANCE::RED: bot.rightDoinker.extend();
  }
  pros::delay(400);

  return;

  // === backs out and releases the doinked ring
  // backs away from the ladder to clear the stumps
  bot.moveToPoint(Pose { - TILE / 2, -8 }, 1000, {.forwards = false, .minSpeed = 72});
  bot.waitUntilDone();
  // puts down the intak lift
  bot.intakeLift.retract();
  // curves towards the target position to make sure the ring ends in its target position
  bot.moveToPose(Pose::fromPolar(-12.5, BLUE_STATION) + Pose { - TILE, - TILE + DRIVE_WIDTH /2}, 1000, {.forwards = false});
  bot.waitUntilDone();
  // retracts the doinkers
  bot.leftDoinker.retract();
  bot.rightDoinker.retract();

  // === swing turns to intake the doinked ring and the corner stack
  const Pose andrewAte { -2.5 * TILE, -2.5 * TILE, RED_STATION };

  //starts the intake
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  // swings toward the safe stack to intake the doinked ring
  bot.swingToPoint(safeStack, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 100});
  bot.waitUntilDone();
  // moves to the save stake and intakes the ring
  bot.moveToPose(safeStack, 1000, {.minSpeed = 1000});
  bot.waitUntilDone();
  //motions to move to the corner
  bot.moveToPose(andrewAte, 1000, {.maxSpeed = 96, .minSpeed = 72});
  bot.waitUntilDone();
  bot.moveToPoint(posCorner, 2000, {.maxSpeed = 72});
  



}