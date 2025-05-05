#include "auton/autons.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"
#include "units/units.hpp"
#include "lemlib/util.hpp"

using lemlib::Pose;

using namespace dimensions::all;
using namespace DIR;

using namespace auton::util;

static void negAWPFunc(ALLIANCE alliance);
static void posAWPFunc(ALLIANCE alliance);

auton::Auton auton::autons::negAWP = {
    .label = (char*)"Negative AWP",
    .run = negAWPFunc,
    .labelForController = (char*)"Neg AWP",
    .description = (char*)"Start on negative side of alliance wall stake, "
                          "facing towards ally stake. "
                          "Gets solo sig AWP. "
                          "Alliance wall stake - 1. "
                          "Neg mogo - 2  - deposited in pos corner. "
                          "Pos mogo - 1.",
};

auton::Auton auton::autons::posAWP = {
    .label = (char*)"Positive AWP",
    .run = posAWPFunc,
    .labelForController = (char*)"Pos AWP",
    .description = (char*)"Start on positive side of alliance wall stake, "
                          "facing towards ally stake. "
                          "Gets solo sig AWP. "
                          "Alliance wall stake - 1. "
                          "Neg mogo - 1. "
                          "Pos mogo - 2. - deposited in neg corner",
};

auto clamp(auto val, auto min, auto max) {
  return std::max(min, std::min(val, max));
}

/**
 * @brief Autonomous for solo AWP
 *
 * @param alliance Color of robot
 * @param corner Sign of the corner the robot starts on
 */
static void genericAWP(ALLIANCE alliance, auton::SignTransform::SIGN corner) {
  bot.setAlliance(alliance);
  bot.setTransform(auton::ChainedTransform::make(
      bot.m_transform, std::make_shared<auton::SignTransform>(
                           corner, auton::SignTransform::NEGATIVE)));

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

  // === Clamp mogo on negative side

  // // Turn towards mogo
  // bot.swingToPoint(negMogo, lemlib::DriveSide::LEFT, 500,
  //                  {.forwards = false, .minSpeed = 64, .earlyExitRange =
  //                  15});
  // waitUntil(
  //     [&] {
  //       return bot.intake.getSensedRing() == bot.getColor() ||
  //              !bot.isInMotion();
  //     },
  //     0);
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
  pros::delay(300);

  // Turn towards stack
  bot.turnToPoint(negSafeStack, 500, {.minSpeed = 64, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Go towards negative safe stack and intake it
  bot.moveToPoint(negSafeStack, 1000
                  /* {.minSpeed = 64 , .earlyExitRange = 4} */);
  bot.waitUntilDone();
  // Give time to intake
  pros::delay(750);

  // === Deposit mogo in actually positive corner and push alliance partner bot
  const Pose goAroundAllyStackTarget {-TILE * 1.5, 0};

  // Turn towards target first, hopefully reducing potential for burn outs
  bot.swingToPoint(goAroundAllyStackTarget, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false, .minSpeed = 48, .earlyExitRange = 20});

  // Go around ally stake rings
  bot.moveToPoint({-TILE * 1.5, 0}, 1200,
                  {.forwards = false, .minSpeed = 96, .earlyExitRange = 7});
  bot.waitUntilDone();

  // Release mogo to make sure mogo hitting rings doesn't stop us
  bot.mogoClamp.extend();
  // Stop intake to prevent hooking the mogo with hooks
  bot.intake.stop();

  // Deposit mogo
  bot.moveToPoint({-TILE * 2.2, -TILE * 1.75}, 2500, {.forwards = false});
  bot.waitUntilDone();

  const Pose posMogo = {-TILE, -TILE};

  // === Intake Safe stack on positive side

  // Adjust odom pose
  bot.setPose(bot.getPose() + Pose {-3, 2});

  const Pose posSafeStack = {-TILE, -TILE * 2};

  // Move towards safe stack, giving a wide berth for alliance partner
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 6, AUDIENCE - 60) +
                     posSafeStack,
                 3000,
                 {
                     .horizontalDrift = 5,
                     .lead = .5,
                 });
  // Once near stack, intake
  bot.waitUntil(9);
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);
  waitUntil([&] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion();
  });
  bot.intake.stop();

  // // Get out of corner
  // bot.moveToPoint({-TILE * 2.5, -TILE}, 1000,
  //                 {.minSpeed = 48, .earlyExitRange = 5});

  // // Turn towards mogo
  // bot.swingToPoint(posMogo, lemlib::DriveSide::RIGHT, 750,
  //                  {.forwards = false, .minSpeed = 32, .earlyExitRange =
  //                  15});

  // // Go pick up mogo
  // bot.moveToPoint(posMogo, 1500, {.forwards = false});

  // // Wait until near mogo to clamp it
  // waitUntilDistToPose(posMogo, 5, 0, true);
  // bot.mogoClamp.retract();
  // // Reset mogo ring counter
  // bot.intake.numOfMogoRings = 0;

  // pros::delay(250);
  // bot.waitUntilDone();

  // === Score ring on pos mogo

  bot.moveToPoint(posMogo, 1000, {.forwards = false, .maxSpeed = 80});
  // Wait until ring is sensed
  waitUntil([&] {
    return bot.intake.getSensedRing() == bot.getColor() || !bot.isInMotion() ||
           bot.getPose().distance(posMogo) < 5;
  });
  // Then stop the intake to hold the ring
  bot.intake.stop();

  // Once near mogo, clamp it
  waitUntilDistToPose(posMogo, 5, 0, true);
  bot.mogoClamp.retract();
  // Give time to clamp
  pros::delay(250);
  bot.waitUntilDone();

  // Turn to be perpendicular to ladder rung
  bot.turnToHeading(AUDIENCE + 45, 500);
  bot.waitUntilDone();

  // Back away from ladder
  tank(40, 40, 100);
  // Score on mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);
  // Give time to score and get away from ladder
  tank(40, 40, 500);

  // === Touch ladder

  // Raise LB to go over ladder rung
  bot.lady.goToIdleScore();
  // Stop instascore to prevent it from hitting ladder
  bot.intake.stop();

  // Give time to raise LB
  pros::delay(250);

  // /** Target position of the point that will touch the ladder.  */
  // const Pose ladderTouchTarget = [&] {
  //   const Pose desired =
  //       (Pose {1, -1} * (bot.getPose().x - bot.getPose().y) - Pose {24, 24})
  //       / 2;
  //   // Prevent from running into ladder post
  //   return Pose {clamp(desired.x, -18.f, -6.f), clamp(desired.y, -6.f,
  //   -18.f)};
  // }();
  /** Target position of the point that will touch the ladder.  */
  const Pose ladderTouchTarget =
      (Pose {1, -1} * (bot.getPose().x - bot.getPose().y) - Pose {24, 24}) / 2;

  const Pose ladderTarget =
      ladderTouchTarget + Pose::fromPolar(-DRIVE_LENGTH / 2 + 8, REFEREE + 45);

  // Go backwards towards ladder
  bot.moveToPoint(ladderTarget, 1000, {.forwards = false});
  bot.waitUntilDone();
  // tank(-64, -64, 300, 0);
  // stop();

  // // Go to ladder
  // const Pose ladderTouchTarget {-TILE * .5 + 1, -TILE * .5 - 1, AUDIENCE +
  // 45}; bot.moveToPose(ladderTouchTarget, 2000,
  //                {.forwards = false, .minSpeed = 127, .earlyExitRange = 6});
  // // Stop intake to prevent it from getting stuck on ladder
  // bot.waitUntil(12);
  // bot.intake.stop();
  // waitUntilDistToPose(ladderTouchTarget, 8, 0, true);
  // bot.cancelMotion();

  // Push hang up into ladder
  bot.lady.flip();
}

static void negAWPFunc(ALLIANCE alliance) {
  genericAWP(alliance, auton::SignTransform::NEGATIVE);
}

static void posAWPFunc(ALLIANCE alliance) {
  genericAWP(alliance, auton::SignTransform::POSITIVE);
}