#include "auton/autons.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "subsystems/ladybrown.h"
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

  // 2nd ring (next to wall stake)
  const Pose ring2 {0, -TILE * 2.5};
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2 + 3, AUDIENCE) + ring2, 1500,
                 {.lead = .3});
  bot.waitUntilDone();

  // === Intake 4 more mogo rings

  const Pose ring3 {-TILE, -TILE * 2};
  const Pose ring4 {-TILE * 2, -TILE * 2};
  const Pose ring5 {-TILE * 2.5, -TILE * 2};
  const Pose ring6 {-TILE * 2, -TILE * 2.5};

  // Face ring3
  bot.swingToPoint(ring3, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 127, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Intake 3rd ring
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::MOGO);
  bot.moveToPoint(ring3, 2000,
                  {.maxSpeed = 96, .minSpeed = 48, .earlyExitRange = 6});

  // Intake 4th and 5th rings
  bot.moveToPoint(ring5 + Pose {DRIVE_LENGTH / 2, 1}, 2000,
                  {.maxSpeed = 48, .minSpeed = 24, .earlyExitRange = 6});
  bot.waitUntilDone();

  // Wait for ring to go into mogo
  pros::delay(300);

  // Intake 6th ring
  bot.swingToPoint(ring6, lemlib::DriveSide::LEFT, 500,
                   {.maxSpeed = 100, .minSpeed = 72, .earlyExitRange = 30});
  // Scoop up by going parallel and next to wall
  bot.moveToPose({ring6.x + DRIVE_LENGTH / 2 + 2, MIN_Y + DRIVE_WIDTH / 2 + 3,
                  BLUE_STATION},
                 1500, {.maxSpeed = 100, .minSpeed = 72, .earlyExitRange = 6});

  bot.waitUntilDone();

  // === Deposit Mogo

  const Pose depositMogoCorner {MIN_X, MIN_Y};

  // Wait a little before turning
  pros::delay(300);

  // Turn towards corner
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::RIGHT, 800,
                   {.forwards = false});

  // Go to corner
  const Pose depositMogoTarget =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 8, REFEREE + 45) + depositMogoCorner;
  bot.moveToPoint(depositMogoTarget, 3000, {.forwards = false});

  // Wait until mogo is full
  waitUntil([&] { return bot.intake.numOfMogoRings >= 6; }, 0, 1750);
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

/** @brief Fills the third mogo. Assumes function is started after blue alliance
 * wall stake with empty mogo clamped. */
void fillThirdMogo() {
  // Reset number of mogo rings
  bot.intake.numOfMogoRings = 0;

  // === Intake ring for mogo and clamp

  // Score 1st ring on mogo
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // === Intake 1 ring from negative corner

  // // Go around mogo with blue ring
  // bot.moveToPose({TILE * 2 - DRIVE_WIDTH / 2 - 3, TILE + 3, REFEREE}, 800,
  //                {.minSpeed = 96, .earlyExitRange = 6});
  // bot.waitUntilDone();

  // Retract LB
  bot.lady.goToIdle();

  // Go to 1st ring (1st of corner)
  const Pose ring1 {TILE * 2, TILE * 2};
  bot.moveToPose(Pose::fromPolar(/* -DRIVE_LENGTH / 2 + 4 */ 0, REFEREE + 45) +
                     ring1,
                 2000);

  const Pose ring2 {TILE, TILE * 2};
  const Pose centerRing {0, 0};

  // Back out of corner
  bot.swingToPoint(ring2, lemlib::DriveSide::LEFT, 300,
                   {.minSpeed = 127, .earlyExitRange = 45});
  bot.turnToPoint(ring2, 500, {.earlyExitRange = 20});
  // Go to ring2
  bot.moveToPoint(ring2, 1000);

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

  // === Intake 5th ring
  bot.moveToPoint({TILE, -TILE * 2}, 1000,
                  {.minSpeed = 72, .earlyExitRange = 6});

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

  // Escape corner
  tank(127, 127, 200, 0);
}

/** @brief Pushes the mogo with the blue ring into the
 * blue negative corner. Assumes that the function is
 * called from blue ally stake. */
void pushBlueRingMogo() {
  // === Go around rings and mogo

  // Go around rings and turn around
  bot.moveToPose({TILE, -TILE * 2.5, RED_STATION}, 1000,
                 {.minSpeed = 72, .earlyExitRange = 6});

  waitUntil([] { return bot.getPose().x < TILE * 1.6 || !bot.isInMotion(); });
  bot.cancelMotion();

  // Go around other mogo
  bot.moveToPose({TILE * 1.5, -TILE, RED_STATION + 45}, 1500,
                 {.minSpeed = 72, .earlyExitRange = 6});

  // === Push mogo into corner
  const Pose mogo {TILE * 2.5, TILE * 2};

  // Go to mogo
  bot.moveToPoint(mogo, 1000,
                  {.forwards = false, .minSpeed = 72, .earlyExitRange = 6});

  // Go to corner
  const Pose depositMogoCorner {MAX_X, MAX_Y};
  const Pose depositMogoTarget =
      // depositMogoCorner + Pose {-DRIVE_WIDTH / 2 + 4, -DRIVE_LENGTH / 2 - 4};
      Pose::fromPolar(DRIVE_LENGTH / 2 + 10, REFEREE + 60) + depositMogoCorner;
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

  // Face mogo
  bot.turnToPoint(mogo1, 500,
                  {.forwards = false, .minSpeed = 32, .earlyExitRange = 20});
  bot.waitUntilDone();

  // Go to mogo
  bot.moveToPoint(mogo1 + Pose {0, DRIVE_LENGTH / 2 - 4.5}, 2500,
                  {
                      .forwards = false,
                      .maxSpeed = 64,
                      .minSpeed = 24,
                      .earlyExitRange = 3,
                  });
  bot.waitUntilDone();

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

  const Pose mogo2 {-TILE * 2, TILE - 0.75};

  // Get out of corner
  bot.moveToPoint({-TILE * 2, -TILE * 2}, 750);
  bot.turnToPoint(mogo2, 500, {.forwards = false});

  // Go to mogo
  bot.moveToPoint(mogo2 + Pose {0, -DRIVE_LENGTH / 2 + 4.5}, 3000,
                  {.forwards = false,
                   .maxSpeed = 64,
                   .minSpeed = 24,
                   .earlyExitRange = 18});
  waitUntilDistToPose(mogo2, 18, 0, true);
  bot.cancelMotion();

  // Go slower to mogo
  bot.moveToPoint(mogo2 + Pose {0, -DRIVE_LENGTH / 2 + 4.5}, 2500,
                  {
                      .forwards = false,
                      .maxSpeed = 24,
                      .minSpeed = 24,
                      .earlyExitRange = 3,
                  });
  bot.waitUntilDone();

  // Push into mogo
  tank(-32, -32, 0, 0);
  pros::delay(400);

  // Grab mogo
  bot.mogoClamp.retract();
  // Give mogo time to clamp
  pros::delay(300);

  // === Do negative red quadrant

  redQuadrant(SignTransform::SIGN::NEGATIVE);

  // === Score on Blue wall stake
  {
    const Pose ring {TILE, TILE};
    // Go to 1st ring
    bot.moveToPose(ring.withTheta(BLUE_STATION + 45), 2500,
                   {.minSpeed = 72, .earlyExitRange = 6});

    // Intake 1st ring once clear of mogo
    bot.waitUntil(12);
    bot.intake.intakeTo(COLOR::RED, Intake::MOGO);

    // Wait until we are near enough to ring to intake it
    // (don't spin around if we intake a ring that isn't ring1)
    waitUntilDistToPose(ring, 12, 0, true);

    // Then wait until ring is sensed
    waitUntil([] {
      return bot.intake.getSensedRing() == COLOR::RED ||
             // If bot is not moving, then we might have
             // missed it
             !bot.isInMotion();
    });
    // Stop ring in intake
    bot.intake.stop();
    // Move on
    bot.cancelMotion();

    // Go to mogo
    const Pose mogo3 {TILE * 2 + 2, 0 + 2};
    bot.turnToPoint(mogo3, 500,
                    {.forwards = false, .minSpeed = 72, .earlyExitRange = 30});

    bot.moveToPoint(
        Pose::fromPolar(-DRIVE_LENGTH / 2 + 4.5, BLUE_STATION + 60) + mogo3,
        2500,
        {
            .forwards = false,
            .maxSpeed = 56,
            .minSpeed = 24,
            .earlyExitRange = 3,
        });
    bot.waitUntilDone();

    // Push into mogo
    tank(-32, -32, 0, 0);
    pros::delay(400);

    // Grab mogo
    bot.mogoClamp.retract();
    // Give mogo time to clamp
    pros::delay(300);

    // Clamp mogo
    bot.waitUntilDone();
    bot.mogoClamp.retract();
    // Give mogo time to clamp
    pros::delay(150);
  }

  // === Fill third mogo

  fillThirdMogo();

  // === Push blue ring mogo

  pushBlueRingMogo();

  // === Hang on ladder

  // Don't hit hooks on ladder
  bot.intake.stop();

  const Pose ladder {0, 0};

  // Turn to ladder
  bot.turnToPoint(ladder, 800,
                  {.forwards = false, .direction = AngDir::CW_CLOCKWISE});
  // Ram into ladder
  bot.moveToPose(ladder.withTheta(REFEREE + 45), 2000,
                 {.forwards = false, .minSpeed = 96});
  // Raise hang
  bot.waitUntil(12);
  bot.lady.flip();

  // Retract LB to improve center of mass
  bot.waitUntilDone();
  bot.lady.goToIdle();
  pros::delay(750);
  bot.lady.setState(LadyBrown::COAST);
}
