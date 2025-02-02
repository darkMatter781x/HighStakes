#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"

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
  printf("intake velo: %f\n", velo);
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
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // 1st ring
  const Pose ring1 {-TILE, -TILE};
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2, AUDIENCE - 45) + ring1,
                 1250,
                 {
                     .lead = .5,
                     .minSpeed = 64,
                     .earlyExitRange = 5,
                 });
  waitUntilDistToPose(ring1, 6, 0, true);
  bot.cancelMotion();

  // 2nd ring
  const Pose ring2 {-TILE, -TILE * 2};
  bot.moveToPose(Pose::fromPolar(-DRIVE_LENGTH / 2, AUDIENCE) + ring2, 1500,
                 {.minSpeed = 64, .earlyExitRange = 5});
  waitUntilDistToPose(ring2, 6, 0, true);
  bot.cancelMotion();

  // 3rd ring
  const Pose ring3 {0, -TILE * 2.5};
  bot.moveToPose((ring3 + Pose {-DRIVE_LENGTH / 2, +2}).withTheta(BLUE_STATION),
                 1750,
                 {
                     .lead = 0.5,
                     .minSpeed = 64,
                     .earlyExitRange = 6,
                 });
  waitUntilDistToPose(ring3, 6, 0, true);
  bot.cancelMotion();

  // === Fill lady brown with 2 rings

  // Go to 1st LB ring
  const Pose lbRing1 {TILE, -TILE * 2};
  const Pose lbRing1Target =
      Pose::fromPolar(-DRIVE_LENGTH / 2 + 3, BLUE_STATION + 15) + lbRing1;
  bot.moveToPose(lbRing1Target, 2500, {.minSpeed = 32, .earlyExitRange = 5});

  // Wait until all mogo rings should be intaked, before lifting LB
  waitUntil([lbRing1] {
    return bot.intake.numOfMogoRings >= 3 ||
           // If close to LB ring, raise LB to grab it
           lbRing1.distance(bot.getPose()) < (DRIVE_LENGTH / 2 + 4) ||
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });
  bot.lady.collect();

  // Save number of mogo rings, as LB may mess up count
  const size_t numOfMogoRingsBeforeLB = bot.intake.numOfMogoRings;

  bool firstLBRingSensed = false;
  bool firstLBRingCollected = false;

  // Wait until 1st LB ring is intaked
  waitUntil([&firstLBRingSensed, &firstLBRingCollected] {
    firstLBRingSensed |= bot.intake.getSensedRing() == COLOR::RED;
    firstLBRingCollected |= firstLBRingSensed && isRingPushedIntoLadyBrown();
    return firstLBRingCollected ||
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });
  // Move on
  bot.cancelMotion();

  // Go to 2nd LB ring
  bot.moveToPose({TILE * 2 - DRIVE_LENGTH / 2 + 1, -TILE * 2.5, BLUE_STATION},
                 2500);
  // If first LB ring is not in LB
  if (!firstLBRingCollected) {
    // Wait until 1st LB ring is intaked
    waitUntil([&firstLBRingSensed, &firstLBRingCollected] {
      firstLBRingSensed |= bot.intake.getSensedRing() == COLOR::RED;
      firstLBRingCollected |= firstLBRingSensed && isRingPushedIntoLadyBrown();
      return firstLBRingCollected ||
             // If bot is not moving, then we might have missed it
             !bot.isInMotion();
    });
  }

  // Once first ring is in LB, raise LB to collect 2nd ring
  bot.lady.collectSecond();
  // Stop intake momentarily to raise LB
  bot.intake.stop();
  // Give LB time to raise
  pros::delay(300);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);

  bot.waitUntilDone();

  // === Score on wall stake

  // Position in front of wall stake
  bot.moveToPose({0, -TILE * 1.75, AUDIENCE}, 2000,
                 {
                     .forwards = false,
                     .minSpeed = 56,
                 });
  // Give time for bot to settle
  pros::delay(100);
  // Wait until 2nd LB ring is collected
  waitUntil([] {
    return isRingPushedIntoLadyBrown() ||
           // If bot is not moving, then we might have missed it
           !bot.isInMotion();
  });

  // Let intake push into LB
  pros::delay(200);
  // Then raise LB to idle score, and stop intake
  bot.lady.goToIdleScore();
  bot.intake.stop();

  bot.waitUntilDone();

  // Go forwards into wall stake
  bot.moveToPose({bot.getPose().x, MIN_Y + DRIVE_LENGTH / 2 + 4.5, AUDIENCE},
                 2000);
  bot.waitUntilDone();

  // Raise LB to score
  bot.lady.score();
  // Give LB time to score
  pros::delay(750);

  // Back out
  tank(-127, -127, 400, 12);
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
  bot.waitUntilDone();

  // Intake 4-6
  bot.moveToPoint(ring4 + Pose {DRIVE_LENGTH / 2, 0}, 3000);
  bot.swingToPoint(ring5, lemlib::DriveSide::LEFT, 500);
  bot.moveToPoint(ring5, 1500);
  bot.moveToPoint(ring6, 1500);

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

  // === Intake negative corner rings

  const Pose centerRing {0, 0};

  // Go to 2nd ring (1st of corner)
  bot.moveToPose(
      {TILE * 2 + DRIVE_LENGTH / 2 + 1, TILE * 2 - DRIVE_WIDTH / 2, REFEREE},
      2000, {.lead = 0.5, .minSpeed = 72, .earlyExitRange = 6});

  // Swing to grab 3rd ring
  bot.swingToPoint(centerRing, lemlib::DriveSide::LEFT, 750,
                   {.direction = AngDir::CCW_COUNTERCLOCKWISE,
                    .minSpeed = 72,
                    .earlyExitRange = 15});

  // === Grab center ring

  // Go to center ring
  const Pose intakeCenterRingTarget = centerRing + Pose {DRIVE_LENGTH / 2, 0};
  bot.moveToPose(intakeCenterRingTarget, 2000,
                 {.minSpeed = 72, .earlyExitRange = 6});

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

  const Pose ring5 {TILE, -TILE};

  // Swing to scoop center ring up
  bot.swingToPoint(ring5, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 72, .earlyExitRange = 30});

  // === Intake 5th ring

  // Go to 5th ring
  bot.moveToPoint(ring5, 2000, {.minSpeed = 72, .earlyExitRange = 6});

  // When under ladder stop intake
  bot.waitUntil(9);
  bot.intake.stop();

  // Once near 5th ring, intake it
  waitUntilDistToPose(ring5, 9, 0, true);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  bot.waitUntilDone();

  // === Score final ring

  bot.moveToPose({TILE * 2 - 3, -TILE * 2, AUDIENCE}, 2000,
                 {.minSpeed = 72, .earlyExitRange = 6});
  bot.waitUntilDone();

  // === Deposit mogo

  // Go further forward
  bot.moveToPoint(bot.getPose().withY(MIN_Y + DRIVE_WIDTH + 3), 750,
                  {.minSpeed = 72, .earlyExitRange = 6});
  // Face corner
  const Pose depositMogoCorner {MAX_X, MIN_Y};
  bot.swingToPoint(depositMogoCorner, lemlib::DriveSide::LEFT, 500,
                   {.forwards = false, .minSpeed = 127, .earlyExitRange = 30});

  // Go to corner
  const Pose depositMogoTarget =
      depositMogoCorner.withTheta(REFEREE - 45) +
      Pose::fromPolar(DRIVE_LENGTH / 2 + 5, REFEREE - 45);
  bot.moveToPose(depositMogoTarget, 2000, {.forwards = false, .minSpeed = 72});

  // Once in corner, release mogo
  bot.waitUntilDone();
  bot.mogoClamp.extend();
  // Prevent mogo from being held by hooks
  bot.intake.outtake();
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
  bot.moveToPose(grabMogo1, 2500,
                 {
                     .forwards = false,
                     .minSpeed = 24,
                     .earlyExitRange = 3,
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
  bot.waitUntilDone();

  // Grab mogo
  bot.mogoClamp.retract();
  // Give mogo time to clamp
  pros::delay(150);

  // === Do negative red quadrant

  redQuadrant(SignTransform::SIGN::NEGATIVE);

  // === Fill third mogo

  fillThirdMogo();

  // === Score final ring on blue ally stake

  scoreOnBlueStake();
}
