#include "auton/autons.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <cmath>
#include <cstddef>
#include <optional>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::SignTransform;

void skillsFunc(ALLIANCE alliance);

auton::Auton auton::autons::skills = {
    .label = (char*)"Skills",
    .run = skillsFunc,
    .labelForController = (char*)"skills",
    .description = (char*)"Bouta go crazy fr fr",
};

/** @returns Whether a ring is detected in the lady brown by using intake
 * velocity. */
bool isRingPushedIntoLadyBrown() {
  // const int current = bot.m_config.motors.intake.get_current_draw();
  // /* if (pros::millis() % 30 < 10)  */ printf("intake current: %imA\n",
  //                                             current);
  // return current > 2000;
  const float velo = bot.m_config.motors.intake.get_actual_velocity();
  // printf("intake velo: %f\n", velo);
  return std::abs(velo) < 0.5;
}

void pulseIntake() {
  // bot.intake.stop();
  // pros::delay(50);
  // bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // pros::delay(150);
  // // Stop intake to prevent intake from cooking itself
  // bot.intake.stop();
}

void waitUntilRingIsInLB(bool& ringSensed, bool& ringCollected,
                         bool checkMotionRunning = false,
                         size_t timeout = INT_MAX) {
  waitUntil(
      [&ringSensed, &ringCollected, checkMotionRunning] {
        ringSensed |= bot.intake.getSensedRing() != std::nullopt;
        ringCollected |= ringSensed && isRingPushedIntoLadyBrown();
        return ringCollected || (!bot.isInMotion() && checkMotionRunning);
      },
      0, timeout);
}

/**
 * @brief Since the 2 red corners are symmetrical, this does one of the
 * corners. Assumes mogo clamped.
 *
 * @param corner Which of the two corners to do
 */
void redQuadrant(SignTransform::SIGN sign) {
  // Change coordinate system based on sign of quadrant
  bot.setTransform(
      std::make_shared<SignTransform>(sign, SignTransform::POSITIVE));

  bot.intake.numOfMogoRings = 0;

  // === Intake 2 mogo rings

  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // 1st ring
  const Pose ring1 {-TILE, -TILE};
  bot.swingToPoint(ring1, lemlib::DriveSide::RIGHT, 300,
                   {.minSpeed = 127, .earlyExitRange = 45});
  bot.moveToPoint(ring1, 1250,
                  {
                      .minSpeed = 64,
                      .earlyExitRange = 6,
                  });
  waitUntilDistToPose(ring1, DRIVE_LENGTH / 2 + 3, 0, true);
  bot.cancelMotion();

  // Go around ladder
  bot.moveToPoint({0, -TILE - 4.5 - DRIVE_WIDTH / 2 - 5}, 800,
                  {.minSpeed = 64, .earlyExitRange = 6});

  // 2nd ring
  const Pose ring2 {TILE, -TILE * 2};
  bot.moveToPoint(ring2, 1500,
                  {
                      .minSpeed = 64,
                      .earlyExitRange = 5,
                  });
  waitUntilDistToPose(ring2, DRIVE_LENGTH / 2 + 3, 0, true);
  bot.cancelMotion();

  // === Fill lady brown with 1 ring

  // Go to 1st LB ring
  const Pose lbRing1 = [&] {
    switch (sign) {
      case SignTransform::SIGN::POSITIVE:
        return Pose {TILE * 2, -TILE * 2, BLUE_STATION};
      case SignTransform::SIGN::NEGATIVE:
        return Pose {TILE * 2, -TILE * 2.5, BLUE_STATION + 30};
    }
  }();
  bot.moveToPoint(
      Pose::fromPolar(-DRIVE_LENGTH / 2 + 1, lbRing1.theta) + lbRing1, 2500);

  // Wait until all mogo rings should be intaked, before lifting LB
  bot.waitUntilDone();
  bot.lady.collect();
  pros::delay(300);
  const size_t timeLadyBrownFirstRaised = pros::millis();

  // Save number of mogo rings, as LB may mess up count
  const size_t numOfMogoRingsBeforeLB = bot.intake.numOfMogoRings;

  bool firstLBRingSensed = false;
  bool firstLBRingCollected = false;

  // === Score 1st ring on wall stake

  const Pose wallStake {0, MIN_Y};
  /** Ring in front of wall stake */
  const Pose wallStakeRing {0, -TILE * 2.5};

  // Position in front of wall stake
  bot.moveToPoint(
      wallStakeRing +
          Pose {
              0,
              RING_DIAM / 2 + (float)hypot(DRIVE_LENGTH / 2, DRIVE_WIDTH / 2) +
                  4 + (sign == SignTransform::SIGN::NEGATIVE ? 1 : 0),
          },
      2000,
      {
          .forwards = false,
      });

  // Wait until 1st LB ring is intaked
  waitUntil(
      [&] {
        firstLBRingSensed |= bot.intake.getSensedRing() != std::nullopt;
        return firstLBRingSensed || !bot.isInMotion();
      },
      0, 1000);
  // Stop ring in intake and wait until LB is ready
  bot.intake.stop();

  // Once LB is ready, intake to it
  bot.waitUntil(18);
  bot.intake.intakeBothTo(Intake::MOGO);
  // Let intake accelerate
  if (firstLBRingSensed) pros::delay(50);

  // Wait until ring is sensed and collected
  waitUntilRingIsInLB(firstLBRingSensed, firstLBRingCollected, true, 2000);
  printf("First LB ring sensed?: %i\n", firstLBRingSensed);
  if (firstLBRingCollected) bot.intake.stop();

  // Turn to wall stake
  bot.turnToPoint(wallStake, 500);
  // Record whether the first ring was sensed and collected
  waitUntilRingIsInLB(firstLBRingSensed, firstLBRingCollected, true, 500);
  printf("First LB ring sensed?: %i\n", firstLBRingSensed);
  if (firstLBRingCollected) bot.intake.stop();
  bot.waitUntilDone();

  // Go straight forwards into wall stake, regardless of
  // whether we're aligned
  bot.moveToPose(bot.getPose()
                     .withY(wallStake.y + DRIVE_LENGTH / 2 + 1.5)
                     .withTheta(AUDIENCE),
                 1000);
  waitUntilRingIsInLB(firstLBRingSensed, firstLBRingCollected, false, 1000);
  printf("First LB ring sensed?: %i\n", firstLBRingSensed);
  if (firstLBRingCollected) bot.intake.stop();

  // Pulse x2 intake to shove into the LB
  pulseIntake();
  pulseIntake();

  bot.waitUntilDone();

  // Move hook out of the way
  bot.intake.outtake();
  pros::delay(30);
  // Raise LB to score
  bot.lady.score();

  const size_t timeToScoreOnWallStake = 750;
  const size_t timeToMoveHooksOutOfTheWay = 200;

  // Push bot into wall stake
  tank(32, 32, 0, 0);
  pros::delay(timeToMoveHooksOutOfTheWay);

  // Intake ring under wall stake until sensed
  {
    bot.intake.intakeBothTo(Intake::MOGO);
    // Wait until ring under wall stake is sensed
    const size_t start = pros::millis();
    const size_t timeout = timeToScoreOnWallStake - timeToMoveHooksOutOfTheWay;
    waitUntil([&] { return bot.intake.getSensedRing() == COLOR::RED; }, 0,
              timeout);
    // Hold ring in intake till the first ring is scored on
    // wall stake
    bot.intake.stop();
    // Wait for the rest of the timeout to give LB to score
    const size_t duration = pros::millis() - start;
    if (duration < timeout) pros::delay(timeout - duration);
  }

  // === Score 2nd ring

  // Back out of wall stake to realign
  tank(-48, -48, 0, 0);

  // Prepare to pick up ring
  bot.lady.collect();

  // Wait until LB is settled
  pros::delay(700);

  // Push bot into wall stake to realign
  tank(48, 48, 0, 0);

  // Intake ring to LB
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // Give intake time to accelerate
  pros::delay(100);

  // Wait until ring is pushed into LB
  waitUntil([&] { return isRingPushedIntoLadyBrown(); }, 0, 800);

  // Pulse intake x2 to shove into the LB
  pulseIntake();

  // Score 2nd ring
  bot.lady.score();

  // Move hook out of the way
  bot.intake.outtake();
  pros::delay(30);
  bot.intake.stop();

  // Give LB time to score
  pros::delay(750);

  // Back out
  bot.moveToPoint(bot.getPose().withY(-TILE * 2), 1000,
                  {.forwards = false, .minSpeed = 127});
  bot.waitUntilDone();

  // Retract LB
  bot.lady.goToIdle();

  // Reset number of mogo rings
  bot.intake.numOfMogoRings = numOfMogoRingsBeforeLB;

  // === Intake 4 more mogo rings

  const Pose ring3 {-TILE, -TILE * 2};
  const Pose ring4 {-TILE * 2, -TILE * 2};
  const Pose ring5 {-TILE * 2.5, -TILE * 2};
  const Pose ring6 {-TILE * 2, -TILE * 2.5};

  // Face ring3
  bot.turnToPoint(ring3, 500, {.minSpeed = 127, .earlyExitRange = 30});
  // Outtake to prevent mogo from being clamped when
  // hitting ladder post
  bot.intake.outtake();
  bot.waitUntilDone();

  // Intake 3rd ring
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::MOGO);
  bot.moveToPoint(ring3, 2000, {.minSpeed = 48, .earlyExitRange = 6});

  // Intake 4th and 5th rings
  // bot.moveToPoint(ring4 + Pose {DRIVE_LENGTH / 2 - 2,
  // 1}, 2000);
  bot.moveToPoint(ring5 + Pose {DRIVE_LENGTH / 2, 1}, 2000,
                  {.minSpeed = 48, .earlyExitRange = 6});
  bot.waitUntilDone();

  // Wait for ring to go into mogo
  pros::delay(300);

  // Intake 6th ring
  bot.swingToPoint(ring6, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});
  // Scoop up by going parallel and next to wall
  bot.moveToPose(
      {ring6.x + DRIVE_LENGTH / 2, MIN_Y + DRIVE_WIDTH / 2 + 3, BLUE_STATION},
      1500, {.minSpeed = 72, .earlyExitRange = 6});

  bot.waitUntilDone();

  // === Deposit Mogo

  const Pose depositMogoCorner {MIN_X, MIN_Y};

  // Turn towards corner
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::RIGHT, 500,
                   {.forwards = false, .minSpeed = 127, .earlyExitRange = 30});

  // Go to corner
  const Pose depositMogoTarget =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 10, REFEREE + 60) + depositMogoCorner;
  bot.moveToPose(depositMogoTarget, 3000, {.forwards = false, .minSpeed = 72});

  // Wait until mogo is full
  waitUntil([&] { return bot.intake.numOfMogoRings >= 6; }, 0, 3000);
  pros::delay(100);

  // Once near corner, release mogo
  bot.mogoClamp.extend();
  pros::delay(200);

  bot.waitUntilDone();

  // Prevent mogo from being held by hooks
  bot.intake.outtake();

  // Reset coordinate system
  bot.setAlliance(ALLIANCE::RED);
}

/** @brief Fills the third mogo. Assumes function is
 * started in red negative corner. (-x,+y) */
void fillThirdMogo() {
  // Reset number of mogo rings
  bot.intake.numOfMogoRings = 0;

  // === Intake ring for mogo and clamp

  const Pose ring1 {TILE, TILE};
  // Go to 1st ring
  bot.moveToPose(ring1.withTheta(BLUE_STATION + 45), 2500,
                 {.minSpeed = 72, .earlyExitRange = 6});

  // Intake 1st ring once clear of mogo
  bot.waitUntil(12);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);

  // Wait until we are near enough to ring to intake it
  // (don't spin around if we intake a ring that isn't
  // ring1)
  waitUntilDistToPose(ring1, 9, 0, true);
  // Then wait until ring is sensed
  waitUntil([] {
    return bot.intake.getSensedRing() == COLOR::RED ||
           // If bot is not moving, then we might have
           // missed it
           !bot.isInMotion();
  });
  // Hold the ring in the intake
  bot.intake.stop();
  // Move on
  bot.cancelMotion();

  // Go to mogo
  const Pose mogo3 {TILE * 2, 0};
  bot.turnToPoint(mogo3, 300,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 30});
  bot.moveToPose(mogo3.withTheta(RED_STATION + 30), 2000, {.forwards = false});

  // Clamp mogo
  bot.waitUntilDone();
  bot.mogoClamp.retract();
  // Give mogo time to clamp
  pros::delay(150);

  // Score 1st ring on mogo
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // === Intake 2 rings from negative corner

  // Go around mogo with blue ring
  bot.moveToPose({TILE * 2 - DRIVE_WIDTH / 2 - 3, TILE + 3, REFEREE}, 800,
                 {.minSpeed = 96, .earlyExitRange = 6});
  // Go to 2nd ring (1st of corner)
  const Pose ring2 {TILE * 2, TILE * 2};
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 1, REFEREE + 45) + ring2,
                 2000, {.lead = 0.5});

  const Pose centerRing {0, 0};

  // Back out of corner
  bot.swingToPoint(centerRing, lemlib::DriveSide::LEFT, 300,
                   {.direction = AngDir::CW_CLOCKWISE,
                    .minSpeed = 127,
                    .earlyExitRange = 45});
  bot.turnToPoint(centerRing, 500,
                  {.direction = AngDir::CW_CLOCKWISE, .earlyExitRange = 20});

  // === Grab center ring

  // Go to center ring
  const Pose intakeCenterRingTarget =
      Pose::fromPolar(-DRIVE_LENGTH / 2, AUDIENCE + 45) + centerRing;
  bot.moveToPose(intakeCenterRingTarget, 2000,
                 {.lead = .4, .minSpeed = 72, .earlyExitRange = 6});

  // Once near ladder, stop intake
  bot.waitUntil(12);
  bot.intake.stop();

  // Once near center ring, intake it
  waitUntilDistToPose(centerRing, 12 * sqrt(2) - 3, 0, true);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // Don't wait to settling
  waitUntilDistToPose(intakeCenterRingTarget, 2, 0, true);
  bot.cancelMotion();

  const Pose ring4 {TILE, -TILE};

  // Swing to scoop center ring up
  bot.swingToPoint(ring4, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});

  // Wait until ring is scored on mogo, to prevent it
  // from being kicked
  const size_t numOfRingsBeforeCenter = bot.intake.numOfMogoRings;
  waitUntil(
      [&numOfRingsBeforeCenter] {
        return bot.intake.numOfMogoRings >= numOfRingsBeforeCenter + 1;
      },
      0, 750);

  // === Intake 4th ring

  // Go to 4th ring
  bot.moveToPoint(ring4, 2000, {.minSpeed = 72, .earlyExitRange = 6});

  // When going under ladder stop intake
  bot.led.autonMessage(0x00FF00, 500);
  // Once near 4th ring, intake it
  bot.intake.stop();

  // Once near 4th ring, intake it
  waitUntilDistToPose(ring4, 9, 0, true);
  bot.led.autonMessage(0xFF00FF, 500);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  bot.waitUntilDone();

  // === Score 1 final ring

  bot.moveToPose({TILE * 2 - 2, -TILE * 2.5, BLUE_STATION}, 2000,
                 {.minSpeed = 32, .earlyExitRange = 6});
  // Wait until rings are in mogo, before depositing mogo
  waitUntil([] { return bot.intake.numOfMogoRings >= 5; }, 0, 1000);

  // === Deposit mogo

  // Face corner
  const Pose depositMogoCorner {MAX_X, MIN_Y};
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false,
                    .direction = AngDir::CCW_COUNTERCLOCKWISE,
                    .minSpeed = 48,
                    .earlyExitRange = 30});

  // Once facing corner, release mogo
  bot.waitUntilDone();
  bot.mogoClamp.extend();
  // Prevent mogo from being held by hooks
  bot.intake.outtake();

  // Go to corner
  const Pose depositMogoTarget =
      depositMogoCorner.withTheta(REFEREE - 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 3, REFEREE - 45);
  bot.moveToPose(depositMogoTarget, 2000,
                 {
                     .forwards = false,
                     .lead = .5,
                     .minSpeed = 48,
                 });

  bot.waitUntilDone();
}

/** @brief assumed to be starting in blue positive corner
 * (+x,-y) */
void scoreOnBlueStake() {
  // === Pick up ring

  const Pose ring {TILE * 2.5, -TILE * 2};
  // Turn to ring
  bot.swingToPoint(ring, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});
  // Go to ring
  bot.moveToPoint(ring, 1500);

  // Once out of corner, intake
  bot.waitUntil(9);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // === Go around mogo

  bot.moveToPose({TILE, -TILE, RED_STATION}, 2000,
                 {.minSpeed = 72, .earlyExitRange = 6});

  // Wait until we have acquired the ring
  waitUntil([] {
    return bot.intake.getSensedRing() == COLOR::RED ||
           // If bot is not moving, then we might have
           // missed it
           !bot.isInMotion();
  });
  // Hold the ring in the intake
  bot.intake.stop();

  bot.waitUntilDone();

  // === Score on blue stake

  bot.moveToPose(
      {
          TILE * 2 - 5.75,
          0,
          BLUE_STATION,
      },
      2000);

  bot.waitUntilDone();

  // Intake into blue stake
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  pros::delay(500);

  bot.intake.stop();
}

/** @brief Pushes the mogo with the blue ring into the
 * blue negative corner. Assumes that the function is
 * called from blue ally stake. */
void pushBlueRingMogo() {
  // === Go around rings and mogo

  // // Go around rings and turn around
  // bot.moveToPose({TILE, -TILE * 2.5, RED_STATION},
  // 1000,
  //                {.minSpeed = 72, .earlyExitRange =
  //                6});

  // waitUntil([] { return bot.getPose().x < TILE * 1.6
  // && bot.isInMotion();
  // }); bot.cancelMotion();

  // // Go around other mogo
  // bot.moveToPose({TILE * 1.5, -TILE, RED_STATION +
  // 45}, 1500,
  //                {.minSpeed = 72, .earlyExitRange =
  //                6});

  // === Push mogo into corner
  const Pose mogo {TILE * 2.5, TILE * 2};

  // Go to mogo
  bot.moveToPoint(mogo, 1000,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});

  // Go to corner
  const Pose depositMogoCorner {MAX_X, MAX_Y};
  const Pose depositMogoTarget =
      depositMogoCorner + Pose {-DRIVE_WIDTH / 2 + 4, -DRIVE_LENGTH / 2 - 4};
  //   depositMogoCorner.withTheta(AUDIENCE + 45) +
  //   Pose::fromPolar(DRIVE_LENGTH / 2 + 5, AUDIENCE +
  //   45);
  bot.moveToPoint(depositMogoTarget, 3000, {.forwards = false, .minSpeed = 72});
  bot.waitUntilDone();

  // === Escape
  tank(64, 64, 300);
  stop();
}

// Autonomous for skills rush (always will be red)
void skillsFunc(ALLIANCE _alliance) {
  bot.setAlliance(ALLIANCE::RED);

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  bot.intake.numOfMogoRings = 0;
  bot.intake.numOfLiftRings = 0;

  bot.setPose({-TILE * 2 - 13.25, 0, BLUE_STATION});

  // === Score on red ally stake

  // Prevent Lady brown from goofing up and lifting
  bot.lady.setState(LadyBrown::COAST);

  // Score ring on ally stake
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intakeLift.retract();
  pros::delay(500);

  // === Clamp mogo

  const Pose mogo1 = {-TILE * 2, -TILE, REF};

  // Get out of alliance stake
  bot.moveToPoint(bot.getPose().withX(mogo1.x), 1000, {.earlyExitRange = 2});
  bot.waitUntilDone();

  // Deploy wall stake indexer
  bot.lady.goToIdleScore();

  // Face mogo
  bot.turnToPoint(mogo1, 500,
                  {.forwards = false, .minSpeed = 32, .earlyExitRange = 20});
  bot.waitUntilDone();

  // Go to mogo
  bot.moveToPoint(mogo1 + Pose {0, DRIVE_LENGTH / 2 - 3.5}, 2500,
                  {
                      .forwards = false,
                      .maxSpeed = 96,
                      .minSpeed = 24,
                      .earlyExitRange = 3,
                  });
  bot.waitUntilDone();

  // Reset lady brown after deploying indexer
  bot.lady.goToIdle();

  // Push into mogo
  tank(-32, -32, 0, 0);
  pros::delay(400);

  // Grab mogo
  bot.mogoClamp.retract();
  // Give mogo time to clamp
  pros::delay(300);

  // === Do positive red quadrant

  redQuadrant(SignTransform::SIGN::POSITIVE);

  // === Clamp negative mogo

  const Pose mogo2 {-TILE * 2, TILE};

  // Get out of corner
  bot.moveToPoint({-TILE * 2, -TILE * 2}, 500,
                  {.minSpeed = 127, .earlyExitRange = 7});
  bot.turnToPoint(mogo2, 500, {.forwards = false});

  // Go to mogo
  bot.moveToPoint(mogo2, 3000,
                  {.forwards = false, .minSpeed = 24, .earlyExitRange = 15});

  bot.waitUntilDone();

  // Once near mogo, coast into it
  tank(-24, -24, 250);
  // Then clamp
  bot.mogoClamp.retract();

  // Coast a little longer while clamp clamps
  tank(-24, -24, 450);

  // === Do negative red quadrant

  redQuadrant(SignTransform::SIGN::NEGATIVE);

  // === Fill third mogo

  fillThirdMogo();

  // === Score final ring on blue ally stake

  scoreOnBlueStake();

  // === Push blue ring mogo

  pushBlueRingMogo();
}
