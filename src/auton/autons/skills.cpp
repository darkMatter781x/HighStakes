#include "auton/autons.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "robot.h"
#include "subsystems/ladybrown.h"

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

/**
 * @brief Since the 2 red corners are symmetrical, this does one of the corners.
 * Assumes mogo clamped.
 *
 * @param corner Which of the two corners to do
 */
void redQuadrant(SignTransform::SIGN sign) {
  // Change coordinate system based on sign of quadrant
  bot.setTransform(
      std::make_shared<SignTransform>(sign, SignTransform::POSITIVE));

  bot.intake.numOfMogoRings = 0;

  // === Intake 3 mogo rings

  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // 1st ring
  const Pose ring1 {-TILE, -TILE};
  bot.swingToPoint(ring1, lemlib::DriveSide::RIGHT, 300,
                   {.minSpeed = 127, .earlyExitRange = 45});
  bot.moveToPose(
      Pose::fromPolar(-DRIVE_LENGTH / 2 - 2, BLUE_STATION + 30) + ring1, 1250,
      {
          .lead = .5,
          .minSpeed = 64,
          .earlyExitRange = 5,
      });
  waitUntilDistToPose(ring1, DRIVE_LENGTH / 2 + 3, 0, true);
  bot.cancelMotion();

  // 2nd ring
  const Pose ring2 {-TILE, -TILE * 2};
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2, AUDIENCE) + ring2, 1500,
                 {.minSpeed = 64, .earlyExitRange = 5});
  waitUntilDistToPose(ring2, DRIVE_LENGTH / 2 + 3, 0, true);
  bot.cancelMotion();

  // 3rd ring
  const Pose ring3 {0, -TILE * 2.5};
  bot.moveToPose((ring3 + Pose {0, +3}).withTheta(BLUE_STATION), 1750,
                 {
                     .lead = 0.5,
                     .minSpeed = 64,
                     .earlyExitRange = 6,
                 });
  waitUntilDistToPose(ring3, 5, 0, true);
  bot.cancelMotion();

  // === Fill lady brown with 1 ring

  // Go to 1st LB ring
  const Pose lbRing1 {TILE, -TILE * 2};
  bot.moveToPoint(lbRing1, 2500);

  // Wait until all mogo rings should be intaked, before lifting LB
  waitUntil([lbRing1] {
    return bot.intake.numOfMogoRings >= 3 ||
           // If close to LB ring, raise LB to grab it
           lbRing1.distance(bot.getPose()) < DRIVE_LENGTH / 2 ||
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });
  bot.lady.collect();

  // Save number of mogo rings, as LB may mess up count
  const size_t numOfMogoRingsBeforeLB = bot.intake.numOfMogoRings;

  bool firstLBRingSensed = false;
  bool firstLBRingCollected = false;

  // Wait until 1st LB ring is intaked
  waitUntil(
      [&firstLBRingSensed, &firstLBRingCollected] {
        firstLBRingSensed |= bot.intake.getSensedRing() == COLOR::RED;
        firstLBRingCollected |=
            firstLBRingSensed && isRingPushedIntoLadyBrown();
        return firstLBRingCollected;
      },
      0, 2000);
  printf("First LB ring sensed?: %i\n", firstLBRingSensed);

  // Intake longer to push into the LB
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  pros::delay(200);
  // Stop intake to prevent intake from cooking itself
  bot.intake.stop();

  bot.waitUntilDone();

  // === Score on wall stake

  const Pose wallStake {sign == SignTransform::NEGATIVE ? 1.f : 0.f, MIN_Y};

  // Position in front of wall stake
  bot.moveToPose(wallStake.withY(-TILE * 1.75).withTheta(AUDIENCE), 2000,
                 {
                     .forwards = false,
                     .lead = .5,
                     .minSpeed = 64,
                     .earlyExitRange = 5,
                 });
  bot.waitUntilDone();

  // Go forwards into wall stake
  bot.moveToPose(
      (wallStake + Pose {0, DRIVE_LENGTH / 2 + 4.5}).withTheta(AUDIENCE), 2000);
  bot.waitUntilDone();

  // Raise LB to score
  bot.lady.score();
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

  // === Intake 3 more mogo rings

  const Pose ring4 {-TILE * 2, -TILE * 2};
  const Pose ring5 {-TILE * 2.5, -TILE * 2};
  const Pose ring6 {-TILE * 2, -TILE * 2.5};

  // Face ring4
  bot.turnToPoint(ring4, 500, {.minSpeed = 127, .earlyExitRange = 30});
  // Outtake to prevent mogo from being clamped
  bot.intake.outtake();
  bot.waitUntilDone();

  // Intake 4th and 5th rings
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::MOGO);
  bot.moveToPoint(ring4 + Pose {DRIVE_LENGTH / 2 - 2, 1}, 3000);
  bot.moveToPoint(ring5 + Pose {DRIVE_LENGTH / 2, 1}, 1000,
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
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false, .minSpeed = 127, .earlyExitRange = 30});

  // Go to corner
  const Pose depositMogoTarget =
      depositMogoCorner.withTheta(REFEREE + 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 3, REFEREE + 45);
  bot.moveToPose(depositMogoTarget, 2000, {.forwards = false, .minSpeed = 72});

  // Wait until mogo is full
  waitUntil([&] { return bot.intake.numOfMogoRings >= 6; }, 0, 2000);

  // Once on corner, release mogo
  bot.waitUntilDone();
  bot.mogoClamp.extend();
  // Prevent mogo from being held by hooks
  bot.intake.outtake();

  // Reset coordinate system
  bot.setAlliance(ALLIANCE::RED);
}

/** @brief Fills the third mogo. Assumes function is started in red negative
 * corner. (-x,+y) */
void fillThirdMogo() {
  // Reset number of mogo rings
  bot.intake.numOfMogoRings = 0;

  // === Intake ring for mogo and clamp

  // Go to 1st ring
  bot.moveToPose({TILE, TILE, BLUE_STATION + 45}, 2500,
                 {.minSpeed = 72, .earlyExitRange = 6});

  // Intake 1st ring once clear of mogo
  bot.waitUntil(12);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);

  // Wait until we have acquired the ring
  waitUntil([] {
    return bot.intake.getSensedRing() == COLOR::RED ||
           // If bot is not moving, then we might have missed it
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

  // === Intake 1 ring from negative corner

  // Go around mogo with blue ring
  bot.moveToPose({TILE * 2 - DRIVE_WIDTH / 2 - 3, TILE + 3, REFEREE}, 800,
                 {.minSpeed = 96, .earlyExitRange = 6});
  // Go to 2nd ring (1st and only of corner)
  const Pose ring2 {TILE * 2.5, TILE * 2};
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 1, REFEREE) + ring2 +
                     Pose {2, 0},
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

  // Wait until ring is scored on mogo, to prevent it from being kicked
  const size_t numOfRingsBeforeCenter = bot.intake.numOfMogoRings;
  waitUntil(
      [&numOfRingsBeforeCenter] {
        return bot.intake.numOfMogoRings >= numOfRingsBeforeCenter + 1;
      },
      1000);

  // === Intake 4th ring

  // Go to 4th ring
  bot.moveToPoint(ring4, 2000, {.minSpeed = 72, .earlyExitRange = 6});

  // When under ladder stop intake
  bot.waitUntil(9);
  bot.intake.stop();

  // Once near 4th ring, intake it
  waitUntilDistToPose(ring4, 9, 0, true);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  bot.waitUntilDone();

  // === Score 1 final ring

  bot.moveToPose({TILE * 2 - 2, -TILE * 2.5, REFEREE}, 2000,
                 {.minSpeed = 32, .earlyExitRange = 6});

  // === Deposit mogo

  // Go further forward
  // bot.moveToPoint(bot.getPose().withY(MIN_Y + DRIVE_WIDTH + 3), 750,
  //                 {.minSpeed = 72, .earlyExitRange = 6});
  // Face corner
  const Pose depositMogoCorner {MAX_X, MIN_Y};
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false,
                    .direction = AngDir::CCW_COUNTERCLOCKWISE,
                    .minSpeed = 48,
                    .earlyExitRange = 30});

  // Once ring is in mogo go into corner
  waitUntil([] { return bot.intake.numOfMogoRings >= 5; }, 0, 1000);

  // Once facing corner, release mogo
  bot.waitUntilDone();
  bot.mogoClamp.extend();
  // Prevent mogo from being held by hooks
  bot.intake.outtake();

  // Go to corner
  const Pose depositMogoTarget =
      depositMogoCorner.withTheta(REFEREE - 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 3, REFEREE - 45);
  bot.moveToPose(depositMogoTarget, 2000, {.forwards = false, .minSpeed = 72});
  bot.waitUntilDone();
}

/** @brief assumed to be starting in blue positive corner (+x,-y) */
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
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });
  // Hold the ring in the intake
  bot.intake.stop();

  bot.waitUntilDone();

  // === Score on blue stake

  bot.moveToPose(
      {
          MAX_X - DRIVE_LENGTH / 2 - 2,
          0,
          RED_STATION,
      },
      2000,
      {
          .forwards = false,
      });

  bot.waitUntilDone();

  // Intake into blue stake
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  pros::delay(500);

  bot.intake.stop();
}

/** @brief Pushes the mogo with the blue ring into the blue negative corner.
 * ~~Assumes that the function is called from blue ally stake~~
 * Assumes that the function is called from blue positive corner (+x,-y) */
void pushBlueRingMogo() {
  // === Go around rings and mogo

  // Go around rings and turn around
  bot.moveToPose({TILE, -TILE * 2.5, RED_STATION}, 1000,
                 {.minSpeed = 72, .earlyExitRange = 6});

  // Go around other mogo
  bot.moveToPose({TILE * 1.5, -TILE, RED_STATION + 45}, 1500,
                 {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});

  // === Push mogo into corner
  const Pose mogo {TILE * 2.5, TILE * 2};

  // Go to mogo
  bot.moveToPoint(mogo, 2000,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});

  // Go to corner
  const Pose depositMogoCorner {MAX_X, MAX_Y};
  const Pose depositMogoTarget =
      depositMogoCorner.withTheta(AUDIENCE + 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 5, AUDIENCE + 45);
  bot.moveToPose(depositMogoTarget, 2000,
                 {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});
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

  // === Score on ally score

  // Prevent Lady brown from goofing up and lifting
  bot.lady.setState(LadyBrown::COAST);

  // Score ring on ally stake
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intakeLift.retract();
  pros::delay(500);

  // === Clamp mogo

  const Pose mogo1 = {-TILE * 2, -TILE, REF};

  // Get out of alliance stake
  bot.moveToPoint(bot.getPose().withX(mogo1.x - 1) + Pose {-TRACK_WIDTH / 2, 0},
                  1000, {.earlyExitRange = 2});
  // Face mogo
  bot.swingToPoint(mogo1, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false, .minSpeed = 48, .earlyExitRange = 15});

  // Cancel motion once back of bot is facing mogo
  waitUntil([&mogo1] {
    const float targetHeading =
        (90 - lemlib::radToDeg(bot.getPose().angle(mogo1))) - 180;
    // printf("goal angle: %f\n", targetHeading);
    return robotAngDist(targetHeading) < 15 || !bot.isInMotion();
  });
  bot.cancelMotion();

  // Go to mogo
  bot.moveToPoint(mogo1 + Pose {0, DRIVE_LENGTH / 2 - 2}, 2500,
                  {
                      .forwards = false,
                      .minSpeed = 24,
                      .earlyExitRange = 4,
                  });
  bot.waitUntilDone();
  // Grab mogo
  bot.mogoClamp.retract();
  // Give mogo time to clamp
  pros::delay(150);

  // === Do positive red quadrant

  redQuadrant(SignTransform::SIGN::POSITIVE);

  // === Clamp negative mogo

  // Get out of corner
  const Pose mogo2 {-TILE * 2, TILE, AUDIENCE};
  bot.swingToPoint(mogo2, lemlib::DriveSide::LEFT, 500, {.forwards = false});

  // Go to mogo
  bot.moveToPose(mogo2, 3000, {.forwards = false});

  // Once near mogo, clamp it
  waitUntilDistToPose(mogo2, 2, 0, true);
  bot.mogoClamp.retract();

  bot.waitUntilDone();

  // Give mogo time to clamp
  pros::delay(150);

  // === Do negative red quadrant

  redQuadrant(SignTransform::SIGN::NEGATIVE);

  // === Fill third mogo

  fillThirdMogo();

  // // === Score final ring on blue ally stake

  // scoreOnBlueStake();

  // === Push blue ring mogo

  pushBlueRingMogo();
}
