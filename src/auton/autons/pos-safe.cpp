#include "auton/autons.h"
#include "auton/transform.h"
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

  // Configure coordinate space to be the same as the safe awp
  bot.setTransform(auton::ChainedTransform::make(
      bot.m_transform,
      std::make_shared<auton::SignTransform>(auton::SignTransform::POSITIVE,
                                             auton::SignTransform::NEGATIVE)));

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

  // Start Lady Brown in collect
  bot.lady.collect();

  const Pose startPose {-TILE * 2 - 13.1, TILE - 11.2,
                        180 - (REFEREE - 53.36 + 20.f)};
  bot.setPose(startPose);

  pros::delay(2000);

  // === Score preload on ally stake
  bot.lady.flip();
  pros::delay(400);

  // Back straight out of stake
  tank(-96, -96, 400, 4);
  // Reset LB
  bot.lady.goToIdle();

  // === Intake top ring of stack next to ally stake

  const Pose negMogo = {-TILE, TILE + 1};
  const Pose allySafeStack {-TILE * 2, 0};

  // Turn to ally stake
  bot.turnToPoint(allySafeStack, 500);
  bot.waitUntilDone();

  // Intake stack
  bot.intakeLift.extend();
  const Pose allySafeStackTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2 - 1, bot.getPose().theta) +
      allySafeStack;

  bot.moveToPoint(allySafeStackTarget, 1000);
  bot.intake.intakeBothTo(Intake::MOGO);

  // Once on top of stack, drop intake to intake the ring
  waitUntilDistToPose(allySafeStack, DRIVE_LENGTH / 2 + 2, 0, true);
  bot.intakeLift.retract();
  bot.waitUntilDone();

  // Give time to pickup
  pros::delay(500);
  // Back up for 200ms
  tank(-32, -32, 0, 0);
  waitUntil([&] { return bot.intake.getSensedRing() == bot.getColor(); }, 0,
            200);
  if (bot.intake.getSensedRing() == bot.getColor()) bot.intake.stop();

  // Push back of bot into mogo and index it
  bot.moveToPoint(negMogo, 1500, {.forwards = false, .maxSpeed = 72});

  // Wait until red ring is sensed
  waitUntil(
      [&] {
        return bot.intake.getSensedRing() == bot.getColor() ||
               !bot.isInMotion() || bot.getPose().distance(negMogo) < 5;
      },
      0);
  // Then stop the intake to hold the ring
  bot.intake.stop();

  // Once near mogo clamp it
  waitUntilDistToPose(negMogo, 5);
  bot.mogoClamp.retract();
  // Reset mogo ring counter
  bot.intake.numOfMogoRings = 0;

  bot.waitUntilDone();

  // Give robot time to clamp
  pros::delay(250);

  // === Score held ring and negative safe stack on mogo

  const Pose negSafeStack = {-TILE, TILE * 2};

  // Score on mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);
  pros::delay(400);

  // Turn towards stack
  bot.turnToPoint(negSafeStack, 500, {.minSpeed = 64, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Go towards negative safe stack and intake it
  bot.moveToPoint(negSafeStack, 1000
                  /* {.minSpeed = 64 , .earlyExitRange = 4} */);
  bot.waitUntilDone();
  // Give time to intake
  pros::delay(750);

  // Reset coordinate space
  bot.setAlliance(alliance);

  // === Yoink ring from corner

  // Go around undesired rings
  const Pose aroundBlueCornerRingTarget {-TILE * 2,
                                         -TILE * 2 - DRIVE_WIDTH / 2};
  bot.swingToPoint(aroundBlueCornerRingTarget, lemlib::DriveSide::LEFT, 500);
  bot.moveToPoint({-TILE * 2, -TILE * 2 - DRIVE_WIDTH / 2}, 750,
                  {.minSpeed = 64});

  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 - 4, AUDIENCE + 45) +
                     Pose {MIN_X, MIN_Y},
                 1500);

  // === Touch ladder

  const Pose ladderTouchTarget {-TILE * .5, -TILE * .5, AUDIENCE + 45};
  // Go to ladder
  bot.moveToPose(ladderTouchTarget, 5000, {.forwards = false, .maxSpeed = 64});

  // Stop intake to prevent it from getting stuck on ladder
  waitUntilDistToPose(ladderTouchTarget, 12);
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
