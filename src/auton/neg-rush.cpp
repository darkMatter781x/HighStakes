#include "lemlib/chassis/chassis.hpp"
#include "lemlib/exitcondition.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "dimensions.h"
#include "subsystems/intake.h"
#include "lemlib/util.hpp"
#include "auton.h"
#include <optional>

using namespace dimensions::all;
using namespace DIR;

using namespace auton::util;

void negativeRushFunc(ALLIANCE alliance);

auton::Auton auton::autons::negativeRush = {
    .label = (char*)"Negative Rush",
    .run = negativeRushFunc,
    .labelForController = (char*)"neg rush",
    .description =
        (char*)"Rushes mid rings and scores 2 on mogo and 3 on alliance stake.",
};

// Autonomous for negative rush (side of the field with negative corners)
void negativeRushFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  const size_t startingLiftRings = bot.intake.numOfLiftRings;
  const size_t startingMogoRings = bot.intake.numOfMogoRings;

  // Programmed from the perspective of the red alliance.
  // Robot facing blue alliance station.
  // Neil knows set
  bot.setPose(
      {MIN_X + TILE - 3.75, TILE * 2 - 4.5 - DRIVE_WIDTH / 2, BLUE_STATION});
  const lemlib::Pose preloadRing = {MIN_X + TILE, TILE + 4};

  // Deploy basket
  bot.lift.goToMiddle();
  bot.intake.outtake();

  // Move past mogo, and first stack of rings
  bot.moveToPoint({-TILE * .75, bot.getPose().y}, 2000, {.minSpeed = 64});
  pros::delay(350);

  // Reset intake and lift
  bot.lift.goToBottom();
  bot.intake.stop();

  waitUntil(
      [] { return bot.getPose().x > MIN_X + TILE * 2 || !bot.isInMotion(); });
  bot.cancelMotion();

  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.intake.intakeTo(bot.getColor(), Intake::LIFT);

  const lemlib::Pose firstMidRing = {-3.5, TILE * 2 - 3.5};
  // turn towards first stack of rings
  bot.swingToPoint({firstMidRing.x, firstMidRing.y}, lemlib::DriveSide::LEFT,
                   500, {.minSpeed = 64, .earlyExitRange = 20});
  bot.waitUntilDone();

  // Prevent stopping motion immediately after getting to point, preventing
  // coasting
  bot.lateralLargeExit = lemlib::ExitCondition {0, INT_MAX};
  bot.lateralSmallExit = lemlib::ExitCondition {0, INT_MAX};

  // move to first stack of rings
  bot.moveToPoint({-8, TILE * 2 - 10}, 1000, {.maxSpeed = 64});
  /** the number of rings robot starts with */
  bool hasSensedFirstRing = false;
  // let motion start, before waiting for it to finish
  pros::delay(50);

  // Wait until first ring is sensed by current
  waitUntil(
      [startingLiftRings, &hasSensedFirstRing] {
        hasSensedFirstRing |=
            bot.intake.getSensedRing() == bot.getColor() ||
            (bot.m_config.motors.intake.get_current_draw() / 1000) /*amps*/ *
                    12 /*volts*/
                < 9 /*watts*/;
        return hasSensedFirstRing;
      },
      50, 1000, true);
  pros::delay(400);
  bot.cancelMotion();

  // Reset exit conditions
  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;

  // Go pick up mogo
  const lemlib::Pose mogoPose = {-TILE, TILE, REF + 45};
  // Back up away from mid to prevent intaking more rings, and get mogo.
  // Go quickly till away from mid.
  bot.moveToPoint(mogoPose, 400,
                  {.forwards = false, /* .lead = .7 , */ .earlyExitRange = 12});

  // Then slow down
  bot.moveToPoint(
      mogoPose, 750,
      {.forwards = false, /* .lead = .7 , */ .maxSpeed = 48, .minSpeed = 32});
  // Wait until hit mogo, then clamp
  waitUntilDistToPose(mogoPose, 5, 500, true);
  // Clamp mogo, and continue moving to keep the momentum
  bot.mogo.close();
  bot.waitUntilDone();

  // Give mogo time to settle
  pros::delay(600);

  // Intake second red ring
  // Prevent stopping motion immediately after getting to point, preventing
  // coasting
  bot.lateralLargeExit = lemlib::ExitCondition {0, INT_MAX};
  bot.lateralSmallExit = lemlib::ExitCondition {0, INT_MAX};

  // Prevent robot from stopping too quickly and hitting the tiles with the
  // intake ramp.
  bot.lateralPID.kD = defaultLatPID.kD / 4;

  // Move to second red ring
  bot.moveToPose({-8, TILE * 2 - 1, 60}, 1500,
                 {
                     .lead = .45,
                     .earlyExitRange = 6,
                 });

  // Wait until first ring is fully intaked to lift
  if (startingLiftRings == bot.intake.numOfLiftRings)
    waitUntil(
        [startingLiftRings] {
          return bot.intake.numOfLiftRings > startingLiftRings;
        },
        0, 500);

  // Intake second ring into mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  bot.waitUntilDone();

  // Reset exit conditions and PID
  bot.lateralLargeExit = defaultLatLargeExit;
  bot.lateralSmallExit = defaultLatSmallExit;
  bot.lateralPID = defaultLatPID;

  // ~~~~~~~~~~~~
  // Back away from mid and face third ring stack
  // ~~~~~~~~~~~~
  const lemlib::Pose thirdRingStack = {-TILE + 4, TILE * 2 - 3};
  lemlib::Pose backAwayFromMidTarget = {-TILE * .75, TILE};

  // Find angle to point towards third ring stack from backAwayFromMidTarget.
  backAwayFromMidTarget.theta =
      trigAngleToHeading(backAwayFromMidTarget.angle(thirdRingStack));

  // Back away from mid and point towards third ring stack
  bot.moveToPose(backAwayFromMidTarget, 1000,
                 {
                     .forwards = false,
                     .lead = 0.4,
                 });
  // Wait until second ring fully intaked
  waitUntil(
      [startingMogoRings] {
        return startingMogoRings < bot.intake.numOfMogoRings;
      },
      0, 500);

  // // Prevent stopping motion immediately after getting to point, preventing
  // // coasting.
  // bot.lateralLargeExit = lemlib::ExitCondition {0, INT_MAX};
  // bot.lateralSmallExit = lemlib::ExitCondition {0, INT_MAX};

  // Move to third ring stack
  bot.moveToPoint(thirdRingStack, 1300);
  // Intake third ring into mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Face towards preload, scooping up third ring if its still not there
  bot.swingToPoint(preloadRing, lemlib::DriveSide::LEFT, 750,
                   {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
                    .maxSpeed = 64});
  // Wait until third ring fully intaked
  waitUntil(
      [startingMogoRings] {
        return (startingMogoRings + 1) < bot.intake.numOfMogoRings;
      },
      0, 1000);
  bot.cancelMotion();

  // Once third ring is intaked, intake preload/4th ring
  bot.moveToPose({preloadRing.x, preloadRing.y - 5, STANDS}, 1000);
  // Wait for third ring to be fully intaked into mogo
  pros::delay(500);
  // Intake preload to lift
  bot.intake.intakeTo(bot.getColor(), Intake::LIFT);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  // Wait until ring is intaked
  waitUntil(
      [startingLiftRings] {
        return (startingLiftRings + 1) < bot.intake.numOfMogoRings;
      },
      0, 1600);
  const lemlib::Pose allianceWallStake {MIN_X, 0};
  const lemlib::Pose scoreOnWallStake =
      (allianceWallStake +
       lemlib::Pose {float(sqrt(2) / 2), float(sqrt(2) / 2)} * 12)
          .withTheta(STANDS + 45);
  bot.waitUntilDone();
  pros::delay(500);
  // Jostle ring into lift
  tank(-127, -127, 100, 0);
  tank(127, 127, 100, 0);
  stop();
  pros::delay(300);

  // Go to wall stake scoring pose
  bot.moveToPose(scoreOnWallStake, 2000, {.maxSpeed = 48});
  // Give ring a moment to settle in intake
  pros::delay(500);

  // Position lift above wall stake.
  bot.lift.goToMiddle();
  bot.waitUntilDone();
  pros::delay(500);

  // Score on wall stake
  bot.lift.goToBottom();
  pros::delay(500);

  // Touch ladder
  bot.moveToPose({-TILE * .75, TILE * .75, REF}, 2000, {.forwards = false});

  // const lemlib::Pose fourthStack {
  //     -TILE * 2, 0 + 9 /* distance from center of robot to intake */};
  // // Get near fourth ring stack
  // bot.moveToPose(fourthStack.x, fourthStack.y + 12, STANDS, 1000,
  //                {.maxSpeed = 48});
  // // Let ring fully intake
  // pros::delay(300);
  // // Intake red ring that is on top of stack
  // bot.lift.goToMiddle();
  // pros::delay(300);
  // // intake ring
  // bot.moveToPose(fourthStack.x, fourthStack.y, STANDS, 1000, {.maxSpeed =
  // 64});

  // // put rings into lift
  // pros::delay(300);
  // bot.intake.intakeTo(Intake::RED, Intake::LIFT);
  // bot.intake.intakeTo(Intake::BLUE, Intake::KICK);

  // return;
  // // Wait until ring is sensed by current
  // waitUntil(
  //     [startingLiftRings, &hasSensedFirstRing] {
  //       hasSensedFirstRing |=
  //           bot.intake.getSensedRing() == Intake::RED ||
  //           (bot.m_config.motors.intake.get_current_draw() / 1000) /*amps*/ *
  //                   12 /*volts*/
  //               < 9 /*watts*/;
  //       return hasSensedFirstRing;
  //     },
  //     50, 500, true);
  // pros::delay(250);
  // bot.moveToPose(MIN_X - TILE * 1.5, 0, RED, 1000, {.forwards = false});

  // // wait until fully intaked
  // waitUntil(
  //     [startingLiftRings] {
  //       return (startingLiftRings + 1) < bot.intake.numOfLiftRings;
  //     },
  //     0, 1300);
  // pros::delay(500);
  // bot.cancelMotion();
  // tank(-64, -64, 200, 0);
  // stop();
  // // Lift up lift and align with alliance stake
  // bot.lift.goToMiddle();
  // pros::delay(800);
  // bot.moveToPose(MIN_X - 15, 0, RED, 600);
  // bot.waitUntilDone();

  // // Stop intake from goofying things up
  // bot.intake.stop();

  // // Put the rings on stake
  // bot.lift.goToBottom();
  // pros::delay(400);

  // // Back away from stake
  // tank(-48, -48, 200, 0);
}
