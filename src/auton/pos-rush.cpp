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

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  const size_t startingLiftRings = bot.intake.numOfLiftRings;
  const size_t startingMogoRings = bot.intake.numOfMogoRings;

  bot.setPose({-TILE * 2 - 2.75, -TILE * 2 - 13.25, RED_STATION});

  // Deploy basket
  bot.lift.goToMiddle();
  bot.intake.outtake();

  const Pose midMogoTarget =
      Pose {0, -TILE * 2 + 2, RED_STATION - 30} +
      Pose {float(-sqrt(3) / 2), -1. / 2} * (DRIVE_LENGTH / 2 + 3);

  // Travel past first stack of rings
  bot.moveToPose(midMogoTarget, 1500,
                 {
                     .forwards = false,
                     .lead = 0.5,
                     .minSpeed = 127,
                 });
  // Keep preload close to top
  pros::delay(200);
  bot.intake.intakeBothTo(Intake::MOGO);

  // Reset intake and lift
  pros::delay(200);
  bot.intake.stop();
  bot.lift.goToBottom();

  // Wait until past first stack of rings
  waitUntil([] { return bot.getPose().x > -TILE - 3 || !bot.isInMotion(); });
  bot.cancelMotion();

  // Finish motion as quick as possible
  bot.lateralSmallExit.range = 3;
  bot.lateralSmallExit.time = 50;

  // Once past rings, go towards mogo slowly and clamp mogo
  bot.moveToPose(midMogoTarget, INT_MAX, {.forwards = false, .lead = 0.5});
  waitUntilDistToPose(midMogoTarget, 3.5, 50, true);
  printf("isInMotion: %i\n", (int)bot.isInMotion());
  bot.cancelMotion();
  printf("x: %f, y: %f\n", bot.getPose().x, bot.getPose().y);
  printf("theta: %f\n", bot.getPose().theta);

  // reset exit conditions
  bot.lateralLargeExit = defaultLatLargeExit;

  // Clamp mogo
  bot.mogo.close();
  // Intake preload into mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  // Give clamp time to clamp
  pros::delay(100);

  // Get second ring and get out of mid
  const Pose secondRingTarget {-TILE + 3, -TILE * 2, RED_STATION + 15};
  bot.moveToPose(secondRingTarget, 300);
  bot.moveToPose(secondRingTarget, 1000, {.maxSpeed = 48});
  // Raise lift to prevent it hitting the blue ring
  bot.lift.goToMiddle();
  pros::delay(300);
  // Drop lift to intake
  bot.lift.goToBottom();
  // Wait until first ring is intaked
  const size_t startWaitingForSecondRing = pros::millis();
  waitUntil(
      [startingMogoRings] {
        return bot.intake.numOfMogoRings >= (startingMogoRings + 1);
      },
      0, 1200);
  printf("%3.3f: first ring sensed\n", float(pros::millis()) / 1000.);

  // Intake second ring to lift
  bot.intake.intakeTo(bot.getColor(), Intake::LIFT);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  // Wait until second ring is sensed
  waitUntil(
      [startWaitingForSecondRing] {
        return bot.intake.getSensedRing() == bot.getColor() ||
               (pros::millis() - startWaitingForSecondRing) > 1700;
      },
      0, 1700);
  printf("%3.3f: second ring sensed\n", float(pros::millis()) / 1000.);

  // Stop going towards second ring
  bot.cancelMotion();

  // Go towards second mogo, throwing the mogo
  Pose secondMogo {-TILE - 1, -TILE - DRIVE_LENGTH / 2 + 1, STANDS};
  bot.turnToPoint(secondMogo, 500,
                  {
                      .forwards = false,
                      .direction = lemlib::AngularDirection::CW_CLOCKWISE,
                      .minSpeed = 64,
                      .earlyExitRange = 20,
                  });

  // Throw the first mogo, once it has the momentum to do so
  pros::delay(50);
  bot.mogo.open();

  bot.moveToPose(secondMogo, 1500, {.forwards = false, .lead = .7});
  bot.waitUntilDone();

  // Clamp second mogo
  bot.mogo.close();
  // Give time to clamp
  pros::delay(100);

  // Intake second ring into luft
  bot.intake.intakeTo(bot.getColor(), Intake::LIFT);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);

  const Pose thirdRingStack {-TILE * 2 - 1.5, 0 - DRIVE_LENGTH / 2 - 4.5, REF};

  // Turn in preparation for third ring stack
  bot.swingToPoint(
      thirdRingStack, lemlib::DriveSide::RIGHT, 1000,
      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
       .maxSpeed = 127,
       .minSpeed = 127,
       .earlyExitRange = 20});

  // Go towards third ring stack
  bot.moveToPose(thirdRingStack, 2000);

  // Wait until second ring is intaked to mogo
  waitUntil(
      [startingLiftRings] {
        return bot.intake.numOfLiftRings >= (startingLiftRings + 1);
      },
      0, 500);
  // Lift up intake in order to intake top ring of third ring stack
  bot.lift.goToMiddle();

  // Intake third ring to mogo
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.waitUntilDone();

  bot.lift.goToBottom();
  // Give intake time to get grip on third ring
  pros::delay(400);

  // Drop intake onto top ring
  // Back away from third ring stack
  bot.moveToPoint(bot.getPose() - Pose {0, 4}, 500, {.forwards = false});

  // Wait until third ring is in mogo
  waitUntil(
      [startingMogoRings] {
        return bot.intake.numOfMogoRings >= startingMogoRings + 2;
      },
      0, 1500);

  // If the ring is in basket, score it on alliance stake
  if (/* bot.intake.numOfLiftRings >= startingLiftRings + 1 */ true) {
    printf("third ring sensed\n");

    // Raise lift to score on alliance stake
    bot.lift.goToMiddle();

    const Pose allianceWallStake {MIN_X, -1.5};
    const Pose scoreOnWallStake =
        (allianceWallStake + Pose {float(sqrt(3) / 2), float(-1. / 2)} * 11)
            .withTheta(REF - 60);

    // Go to wall stake
    bot.moveToPose(scoreOnWallStake, 1300);
    bot.waitUntilDone();
    // Score on wall stake
    bot.lift.goToBottom();
    // Give time for ring to score
    pros::delay(600);

    // Make sure trapdoor doesn't get stuck in intake
    bot.lift.goToMiddle();
    // Back away from wall stake
    tank(-127, -127, 300, 0);
    // Reset lift
    bot.lift.goToBottom();
  }

  // Touch ladder
  bot.moveToPose({-TILE * .75, -TILE * .75 + .5, RED_STATION}, 2000,
                 {.forwards = false});
}
