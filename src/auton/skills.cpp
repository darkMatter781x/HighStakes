#include "color.h"
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

void skillsFunc(ALLIANCE alliance);

auton::Auton auton::autons::skills = {
    .label = (char*)"Skills",
    .run = skillsFunc,
    .labelForController = (char*)"skills",
    .description = (char*)"Bouta go crazy fr fr",
};

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

  // Score preload on alliance stake
  bot.intake.intakeBothTo(Intake::MOGO);
  pros::delay(220);
  bot.intake.stop();

  // Reset count of rings in mogo (dont include preload)
  bot.intake.numOfMogoRings = 0;

  // Wait until ring is scored
  pros::delay(200);

  const Pose grabMogo1 = {-TILE * 2, -TILE + DRIVE_LENGTH / 2, REF};
  // Get out of alliance stake, and face mogo
  bot.moveToPose(grabMogo1 + Pose {0, 40}, 500,
                 {.minSpeed = 96, .earlyExitRange = 5});
  // Deploy basket
  bot.intake.outtake();
  bot.lift.goToMiddle();
  pros::delay(400);

  // Reset lift and intake
  bot.lift.goToBottom();
  bot.intake.stop();

  // Go to mogo
  bot.moveToPose(grabMogo1, 1500, {.forwards = false, .lead = .7});
  // Once near mogo, use more consistent moveToPoint
  waitUntilDistToPose(grabMogo1, 12, 0, true);
  bot.cancelMotion();
  bot.moveToPoint(grabMogo1, 1000, {.forwards = false});

  // Once next to mogo, grab it
  waitUntil([grabMogo1] {
    return bot.getPose().y < grabMogo1.y + 3 || !bot.isInMotion();
  });
  bot.mogo.close();
  bot.waitUntilDone();

  // Turn to next ring, but make sure to continue backing into mogo, to ensure
  // clamping
  bot.swingToHeading(REF + 60, lemlib::DriveSide::LEFT, 400,
                     {.minSpeed = 127, .earlyExitRange = 20});

  // Move towards first ring, and intake it
  const Pose ring1 = {-TILE, -TILE, BLUE_STATION + 30};
  bot.moveToPose(ring1, 1500, {.minSpeed = 72, .earlyExitRange = 6});
  // Turn on intake
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // Wait until close, then go to next ring
  waitUntilDistToPose(ring1, 6, 0, true);
  bot.cancelMotion();

  // Move to second ring, next to wall stake
  const Pose ring2 {-0, -TILE * 2.5 /*  + DRIVE_LENGTH / 2 - 4 */, STANDS};
  bot.moveToPoint(ring2, 2000);

  // Wait until close, then go slowly to next ring
  waitUntilDistToPose(ring2, 24);
  bot.cancelMotion();
  bot.moveToPoint(ring2, 2000, {.maxSpeed = 48});

  // Wait until close enough to intake, then move on
  waitUntilDistToPose(ring2, DRIVE_LENGTH / 2 - 3);
  bot.cancelMotion();

  // Back away from wall stake
  tank(-64, -64, 200, 0);

  // First of line of 3 rings
  const Pose ring3 {-TILE + DRIVE_LENGTH / 2, -TILE * 2 + 2, RED_STATION};
  const Pose ring4 {-TILE * 2 + DRIVE_LENGTH / 2, -TILE * 2 + 2, RED_STATION};
  const Pose ring5 {-TILE * 2.5 + 4, -TILE * 2 + 2, RED_STATION};

  // Turn towards line of rings
  bot.moveToPose(ring3 - Pose {-36, 0}, 500,
                 {
                     .forwards = false,
                     .minSpeed = 127,
                 });
  bot.moveToPose(ring3, 1200);
  bot.moveToPose(ring4, 1000);

  // Move to intake first ring, quickly at first
  // bot.moveToPose(ring5, 1000,
  //                {.maxSpeed = 96, .minSpeed = 96, .earlyExitRange = 4});
  // waitUntilDistToPose(ring3, 4, 0, true);
  // bot.cancelMotion();

  // // Move to intake second ring, slowly at first
  // bot.moveToPose(ring5, 1000,
  //                {.maxSpeed = 56, .minSpeed = 56, .earlyExitRange = 8});
  // waitUntilDistToPose(ring4, 8, 0, true);
  // // bot.cancelMotion();

  // // Then move quickly through the last ring
  // bot.moveToPose(ring5, 900,
  //                {.maxSpeed = 96, .minSpeed = 96, .earlyExitRange = 12});
  // waitUntilDistToPose(ring5, 12, 0, true);
  // bot.cancelMotion();

  // Slow down and intake all 3 rings
  bot.moveToPose(ring5, 4000, {.maxSpeed = 32});

  /** The ring to be scored on the first tall wall stake */
  const Pose stakeRing2 {-TILE * 2, -TILE * 2.5, BLUE_STATION + 45};
  // Turn towards ring for stake
  bot.turnToPoint(stakeRing2, 500, {.minSpeed = 96, .earlyExitRange = 45});
  RobotConfig::LEDs::leds.leftUnderGlow->buf().setAll(0xFF0000);

  // Move towards ring for stake ring
  bot.moveToPoint(stakeRing2, 1500, {.earlyExitRange = 4});
  RobotConfig::LEDs::leds.leftUnderGlow->buf().setAll(0x00FF00);

  bot.waitUntilDone();
  RobotConfig::LEDs::leds.leftUnderGlow->buf().setAll(0x0000FF);

  const Pose depositMogo1 =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 6, REF + 45) + Pose {
                                                            MIN_X,
                                                            MIN_Y,
                                                        };

  // Intake rest of rings to lift (should be 1)
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // Swing towards corner to deposit mogo
  bot.swingToPoint(depositMogo1, lemlib::DriveSide::RIGHT, 500,
                   {.forwards = false, .earlyExitRange = 30});
  bot.waitUntilDone();
  // Push mogo into corner
  bot.moveToPose(depositMogo1, 1200, {.forwards = false});
  waitUntil([] { return bot.intake.numOfMogoRings >= 6; }, 0, 2000);

  bot.mogo.open();
  bot.intake.stop();

  pros::delay(500);

  // Wait until close to corner
  // waitUntilDistToPose(depositMogo1, 6, 0, true);
  // Then, ensure intake doesn't hold onto mogo

  // Release mogo
  // waitUntilDistToPose(depositMogo1, 3, 0, true);
  // bot.cancelMotion();

  bot.waitUntilDone();
  // Reset number of mogo rings
  bot.intake.numOfMogoRings = 0;

  // const Pose scoreOnTallStake1 {0, MIN_Y + DRIVE_LENGTH / 2 + 5, STANDS};

  // // Go to second wall stake
  // bot.moveToPose(scoreOnTallStake1 + Pose {0, 5}, 2500, {.lead = .6});
  // pros::delay(200);
  // bot.intake.intakeTo(COLOR::RED, Intake::LIFT);

  // // Wait until ring has been intaked to lift
  // waitUntil([] { return bot.intake.numOfLiftRings >= 1 || !bot.isInMotion();
  // });
  // // If ring was not intaked to lift
  // if (bot.intake.numOfLiftRings < 1) {
  //   // Wait 500ms longer
  //   waitUntil([] { return bot.intake.numOfLiftRings >= 1; }, 0, 500);
  // }
  // // If ring was intaked to lift
  // if (/* bot.intake.numOfLiftRings >= 1 */ true) {
  //   // Wait until ring in lift
  //   pros::delay(800);
  //   bot.lift.goToTop();
  //   RobotConfig::LEDs::leds.leftUnderGlow->buf().setAll(0xFF0000);
  //   // Give lift to raise
  //   pros::delay(2000);
  //   bot.moveToPose(scoreOnTallStake1, 500);
  // }

  // bot.waitUntilDone();
  // // Score ring on second wall stake
  // bot.lift.goToBottom();
  // // Give lift time to score
  // pros::delay(400);
  // // Reset number of lift rings
  // bot.intake.numOfLiftRings = 0;
  // tank(-127, -127, 200, 0);
  /*
     //-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-\\
    //                                                               \\
    ||                    BEGIN 2nd QUARTER                          ||
    \\                                                               //
     \\-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-//
  */

  // Position to score on blue stake
  const Pose getNearBlueStake =
      Pose::fromPolar(-DRIVE_LENGTH / 2 - 12, BLUE_STATION - 30) + Pose {
                                                                       MAX_X,
                                                                       0,
                                                                   };

  // Ring for third stake
  const Pose stakeRing3 = {TILE, -TILE * 2, BLUE_STATION - 30};
  // Turn towards third ring for stake
  bot.turnToPoint(stakeRing3, /* lemlib::DriveSide::RIGHT, */ 500,
                  {.minSpeed = 127, .earlyExitRange = 45});

  // Intake second lift ring
  bot.moveToPoint(stakeRing3, 1800, {.minSpeed = 127, .earlyExitRange = 4});
  bot.intake.intakeTo(COLOR::RED, Intake::LIFT);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // Go to blue alliance stake
  bot.moveToPose(getNearBlueStake, 3500, {.lead = .4});
  waitUntil([] { return bot.intake.numOfLiftRings >= 1 || !bot.isInMotion(); });
  // Give ring time to get in lift
  pros::delay(1200);
  // Raise lift to score
  bot.lift.goToMiddle();
  pros::delay(500);

  const Pose scoreOnBlueStake =
      Pose::fromPolar(-DRIVE_LENGTH / 2 - 6, BLUE_STATION - 30) + Pose {
                                                                      MAX_X,
                                                                      0,
                                                                  };
  bot.moveToPose(scoreOnBlueStake, 1000);
  bot.waitUntilDone();
  // Score ring on blue stake
  bot.lift.goToBottom();
  // Give lift time to score
  pros::delay(500);

  // Location of mogo with blue ring to be put in corner
  const Pose blueMogo = {TILE * 2.5, -TILE, REF};

  // Get out of stake and go to mogo
  bot.moveToPose(blueMogo, 2500,
                 {.forwards = false, .minSpeed = 64, .earlyExitRange = 5});
  waitUntilDistToPose(blueMogo, 3, 0, true);
  bot.cancelMotion();

  const Pose pushBlueMogoToCorner =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 4, REF - 30) + Pose {
                                                            MAX_X,
                                                            MIN_Y,
                                                        };
  // Push mogo into corner
  bot.moveToPose(pushBlueMogoToCorner, 2500,
                 {.forwards = false, .minSpeed = 64});
  waitUntilDistToPose(pushBlueMogoToCorner, 5, 0, true);
  bot.cancelMotion();

  // intake rings
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  bot.moveToPose({TILE * 2, -TILE * 2, REF - 45}, 1000,
                 {.minSpeed = 64, .earlyExitRange = 4});
  const Pose mogo2 {TILE * 2, -DRIVE_LENGTH / 2, STANDS};

  bot.swingToPoint(mogo2, lemlib::DriveSide::LEFT, 800,
                   {.forwards = false, .minSpeed = 64, .earlyExitRange = 30});
  bot.moveToPose(mogo2, 2000, {.forwards = false, .earlyExitRange = 4});

  waitUntil([] {
    return bot.intake.getSensedRing() == COLOR::RED || !bot.isInMotion();
  });
  bot.intake.stop();

  bot.waitUntilDone();
  bot.mogo.close();

  pros::delay(500);
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);
  pros::delay(500);

  const Pose depositMogo2 {MAX_X, MAX_Y, STANDS + 45};

  bot.swingToPoint(depositMogo2, lemlib::DriveSide::RIGHT, 600,
                   {.forwards = false});
  bot.moveToPose(depositMogo2, 2500, {.forwards = false});
  bot.waitUntilDone();

  bot.mogo.open();
  bot.intake.stop();
  pros::delay(500);
}