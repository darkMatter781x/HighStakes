#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"

using namespace dimensions::all;
using namespace DIR;
using lemlib::Pose;

using namespace auton::util;

void skillsFunc(ALLIANCE alliance);

auton::Auton auton::autons::skills = {
    .label = (char*)"Skills",
    .run = skillsFunc,
    .labelForController = (char*)"skills",
    .description = (char*)"Bouta go crazy fr fr",
};

/** @returns Whether a ring is detected in the lady brown by using intake
 * current. */
bool isRingPushedIntoLadyBrown() {
  const size_t current = bot.m_config.motors.intake.get_current_draw();
  if (pros::millis() % 50 < 10) printf("intake current: %i\n", current);
  return current > 2000;
}

enum class CORNER { NEGATIVE, POSITIVE };

/**
 * @brief Since the 2 red corners are symmetrical, this does one of the corners.
 * Assumes mogo clamped.
 *
 * @param corner Which of the two corners to do
 */
void redQuadrant(CORNER corner) {
  // Programmed for positive corner

  bot.intake.numOfMogoRings = 0;

  // === Intake 3 mogo rings

  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // 1st ring
  bot.moveToPoint({-TILE, -TILE}, 1250, {.minSpeed = 72, .earlyExitRange = 6});
  // 2nd ring
  bot.moveToPoint({-TILE, -TILE * 2}, 1000,
                  {.minSpeed = 72, .earlyExitRange = 6});
  // 3rd ring
  bot.moveToPose({0, -TILE * 2.5, BLUE_STATION}, 1750,
                 {.minSpeed = 72, .earlyExitRange = 6});

  bot.waitUntilDone();

  // === Fill lady brown with 2 rings

  // Go to 1st LB ring
  bot.moveToPoint({TILE, -TILE * 2}, 1250,
                  {.minSpeed = 72, .earlyExitRange = 6});

  // Wait until 2nd mogo ring should be intaked
  bot.waitUntil(12);
  bot.lady.collect();

  // Save number of mogo rings, as LB may mess up count
  const size_t numOfMogoRingsBeforeLB = bot.intake.numOfMogoRings;

  bool firstLBRingIntaked = false;

  // Wait until 1st LB ring is intaked
  waitUntil([&firstLBRingIntaked] {
    firstLBRingIntaked |= isRingPushedIntoLadyBrown();
    return firstLBRingIntaked ||
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });
  bot.waitUntilDone();

  // Go to 2nd LB ring
  bot.moveToPose({TILE * 2, -TILE * 2.5 - 2, BLUE_STATION}, 1250,
                 {.minSpeed = 72, .earlyExitRange = 6});
  // If first LB ring is not in LB
  if (!firstLBRingIntaked) {
    // Wait until 1st LB ring is intaked
    waitUntil([&firstLBRingIntaked] {
      firstLBRingIntaked |= isRingPushedIntoLadyBrown();
      return firstLBRingIntaked ||
             // If bot is not moving, then we might have missed it
             !bot.isInMotion();
    });
  }

  // Once first ring is in LB, raise LB to collect 2nd ring
  bot.lady.collectSecond();

  // Stop intake momentarily to raise LB
  bot.intake.stop();
  pros::delay(100);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);

  bot.waitUntilDone();

  // === Score on wall stake

  // Position in front of wall stake
  bot.moveToPose({0, -TILE * 1.5, AUDIENCE}, 2000,
                 {
                     .forwards = false,
                     .minSpeed = 56,
                 });

  // Wait until 2nd LB ring is collected
  waitUntil([] {
    return isRingPushedIntoLadyBrown() ||
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });

  // Then raise LB to idle score, and stop intake
  bot.lady.goToIdleScore();
  bot.intake.stop();

  bot.waitUntilDone();

  // Go forwards into wall stake
  bot.moveToPose({0, MIN_Y + DRIVE_LENGTH / 2 + 3, REFEREE}, 1500,
                 {.minSpeed = 56});
  // Raise LB to score
  bot.lady.score();
  bot.waitUntilDone();

  // Back out
  tank(-127, -127, 200, 12);
  // Retract LB
  bot.lady.goToIdle();
  // Reset number of mogo rings
  bot.intake.numOfMogoRings = numOfMogoRingsBeforeLB;

  // === Intake 4 more mogo rings

  // const Pose ring3 {-TILE, -TILE * 2};
  const Pose ring4 {-TILE * 2, -TILE * 2.5};
  const Pose ring5 {-TILE * 2, -TILE * 2};
  const Pose ring6 {-TILE * 2.5, -TILE * 2};

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::MOGO);

  bot.swingToPoint(ring4, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 127, .earlyExitRange = 30});
  // // Go to 3rd ring and face 4th ring
  // bot.moveToPose(ring3.withTheta(lemlib::radToDeg(M_PI_2 -
  // ring3.angle(ring4))),
  //                2000, {.minSpeed = 72, .earlyExitRange = 6});

  // Intake 4-6
  bot.moveToPoint(ring4, 1000, {.minSpeed = 72, .earlyExitRange = 6});
  bot.moveToPoint(ring5, 1000, {.minSpeed = 72, .earlyExitRange = 6});
  bot.moveToPoint(ring6, 1000, {.minSpeed = 72, .earlyExitRange = 6});

  bot.waitUntilDone();

  // === Deposit Mogo

  const Pose depositMogoCorner {MIN_X, MIN_Y};

  // Turn towards corner
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false, .minSpeed = 127, .earlyExitRange = 30});

  // Go to corner
  const Pose depositMogoTarget =
      depositMogoCorner.withTheta(RED_STATION + 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 5, RED_STATION + 45);
  bot.moveToPose(depositMogoTarget, 2000, {.forwards = false, .minSpeed = 72});

  // Once near corner, release mogo
  waitUntilDistToPose(depositMogoTarget, 5, 0, true);
  bot.mogoClamp.retract();
  // Prevent mogo from being held by hooks
  bot.intake.outtake();

  bot.waitUntilDone();
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

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intakeLift.retract();
  pros::delay(500);

  // === Clamp mogo

  const Pose grabMogo1 = {-TILE * 2, -TILE, REF};
  // Get out of alliance stake, and face mogo
  bot.moveToPose(grabMogo1 + Pose {0, 40}, 500,
                 {.minSpeed = 64, .earlyExitRange = 5});

  // Go to mogo
  bot.moveToPose(grabMogo1, 2500, {.forwards = false, .maxSpeed = 64});
  bot.waitUntilDone();
  // Grab mogo
  bot.mogoClamp.retract();
  // Give mogo time to clamp
  pros::delay(300);

  // === Complete red quadrant
  redQuadrant(CORNER::POSITIVE);
}
