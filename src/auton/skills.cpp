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
  bot.intakeLift.retract();

  // Score preload on alliance stake
  bot.intake.intakeBothTo(Intake::MOGO);
  pros::delay(220);

  // Reset count of rings in mogo (dont include preload)
  bot.intake.numOfMogoRings = 0;

  // Wait until ring is scored
  pros::delay(200);

  const Pose grabMogo1 = {-TILE * 2 - 4, -TILE + 2, REF};
  // Get out of alliance stake, and face mogo
  bot.moveToPose(grabMogo1 + Pose {0, 40}, 500,
                 {.minSpeed = 64, .earlyExitRange = 5});
  // Deploy basket
  // bot.intake.outtake();
  // pros::delay(400);

  // Reset lift and intake
  // bot.intake.stop();

  // Go to mogo
  bot.moveToPose(grabMogo1, 2500,
                 {.forwards = false, .lead = .7, .maxSpeed = 64});
  // Once near mogo, use more consistent moveToPoint
  // waitUntilDistToPose(grabMogo1, 12, 0, true);
  // bot.cancelMotion();
  // bot.moveToPoint(grabMogo1, 1000, {.forwards = false});

  // Once next to mogo, grab it
  waitUntil([grabMogo1] {
    return bot.getPose().y < grabMogo1.y + 3 || !bot.isInMotion();
  });
  bot.mogoClamp.retract();
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
  const Pose ring2 {-0, -TILE * 2.4 /*  + DRIVE_LENGTH / 2 - 4 */, STANDS};
  bot.moveToPoint(ring2, 2000, {.maxSpeed = 48});

  // Wait until close, then go slowly to next ring
  waitUntilDistToPose(ring2, 24, 0, true);
  bot.cancelMotion();
  bot.setPose(bot.getPose() + lemlib::Pose {0, 3});
  bot.moveToPoint(ring2, 2000, {.maxSpeed = 48});

  // Wait until close enough to intake, then move on
  waitUntilDistToPose(ring2, DRIVE_LENGTH / 2 - 3, 0, true);
  bot.cancelMotion();

  // Back away from wall stake
  tank(-64, -64, 200, 0);

  // First of line of 3 rings
  const Pose ring3 {-TILE + DRIVE_LENGTH / 2, -TILE * 2 + 2, RED_STATION};
  const Pose ring4 {-TILE * 2 + DRIVE_LENGTH / 2, -TILE * 2 + 2, RED_STATION};
  const Pose ring5 {-TILE * 2.5 + 4, -TILE * 2 + 2, RED_STATION};

  // Turn towards line of rings
  // bot.moveToPose(ring3 - Pose {-36, 0}, 500,
  //                {
  //                    .forwards = false,
  //                    .minSpeed = 127,
  //                });
  // bot.moveToPose(ring3, 1200);
  // bot.moveToPose(ring4, 1000);

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
  bot.moveToPose(ring5, 6000, {.maxSpeed = 40});

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
      Pose::fromPolar(DRIVE_LENGTH / 2 + 4, REF + 45) + Pose {
                                                            MIN_X,
                                                            MIN_Y,
                                                        };

  // Intake rest of rings to lift (should be 1)
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // Swing towards corner to deposit mogo
  bot.swingToPoint(depositMogo1, lemlib::DriveSide::RIGHT, 500,
                   {.forwards = false, .earlyExitRange = 30});
  bot.waitUntilDone();
  pros::delay(500); // Give more time to intake
  // Push mogo into corner
  bot.moveToPose(depositMogo1, 1200, {.forwards = false});
  waitUntil([] { return bot.intake.numOfMogoRings >= 6; }, 0, 2000);

  // Release mogo and make sure intake doesn't hold onto it
  bot.mogoClamp.extend();
  bot.intake.outtake();
  pros::delay(100);
  bot.intake.stop();
  // Reset number of mogo rings
  bot.intake.numOfMogoRings = 0;

  bot.waitUntilDone();
  // Adjust odom
  bot.setPose(bot.getPose() + Pose {7, 0, 0});

  // pass auton line
  bot.moveToPose({0, -TILE * 1.75, BLUE_STATION}, 3000, {.maxSpeed = 72});
  // prepare to push mogo
  bot.moveToPoint({TILE * 2.5, 0, REFEREE}, 3000, {.maxSpeed = 72});

  const Pose mogo2 {TILE * 2.5, -TILE, REFEREE};
  bot.turnToPoint(mogo2, 700,
                  {
                      .forwards = false,
                      .maxSpeed = 64,
                  });

  // go to mogo
  bot.moveToPose(mogo2, 1000, {.forwards = false, .maxSpeed = 72});
  // Deposit mogo
  const Pose depositMogo2 =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 4, REFEREE - 45) + Pose {
                                                                MAX_X,
                                                                MIN_Y,
                                                            };
  bot.moveToPose(depositMogo2, 4000, {.forwards = false, .maxSpeed = 72});

  // Go to next mogo
  bot.moveToPose({TILE * 2.5, TILE, REFEREE}, 4000);

  // Deposit mogo
  const Pose depositMogo3 =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 4, AUDIENCE + 45) + Pose {
                                                                 MAX_X,
                                                                 MAX_Y,
                                                             };
  bot.moveToPose(depositMogo3, 4000, {.forwards = false, .maxSpeed = 72});

  // pass auton line
  bot.moveToPose({0, TILE * 1.75, RED_STATION}, 3000, {.maxSpeed = 72});

  // Get in front of next mogo
  bot.moveToPose({-TILE, TILE}, 3000, {.maxSpeed = 72});
  bot.moveToPoint({-TILE * 2, 0}, 2000, {.maxSpeed = 72});

  // Deposit mogo
  const Pose depositMogo4 =
      Pose::fromPolar(DRIVE_LENGTH / 2 + 4, AUDIENCE - 45) + Pose {
                                                                 MIN_X,
                                                                 MAX_Y,
                                                             };
  bot.moveToPose(depositMogo4, 4000, {.forwards = false, .maxSpeed = 72});
  bot.waitUntilDone();
  return;

  // /*
  //    //-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-\\
  //   //                                                               \\
  //   ||                    BEGIN 2nd QUARTER                          ||
  //   \\                                                               //
  //    \\-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-+-.-+-//
  // */

  // // First ring for second mogo
  // const Pose ring2_1 = {TILE - 3, -TILE * 2, BLUE_STATION - 30};
  // // Turn towards third ring for stake
  // bot.turnToPoint(ring2_1, 500, {.minSpeed = 32, .earlyExitRange = 45});

  // // Intake second lift ring
  // bot.moveToPoint(ring2_1, 1800, {.earlyExitRange = 3});
  // // bot.intake.intakeTo(COLOR::RED, Intake::LIFT);
  // bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // // // Against wall between alliance stake and mogo
  // // const Pose reset1 {MAX_X + 3, -TILE + DRIVE_WIDTH / 2 + 5, RED_STATION};

  // // // Turn to face reset position
  // // bot.turnToPoint(reset1, /* lemlib::DriveSide::RIGHT, */ 500,
  // //                 {.forwards = false, .minSpeed = 96, .earlyExitRange =
  // 30});
  // // // Go to wall to reset position
  // // bot.moveToPose(reset1, 2000, {.forwards = false, .lead = .7});

  // // // Reset at wall
  // // bot.waitUntilDone();
  // // std::cout << format_as(bot.getPose()) << std::endl;
  // // bot.setPose(
  // //     {/* MAX_X - DRIVE_LENGTH / 2 + 1 */65.5, bot.getPose().y,
  // //     bot.getPose().theta});
  // // return;

  // const Pose mogo2 {TILE * 2, -DRIVE_LENGTH / 2 + 3, STANDS};

  // // Turn to face mogo
  // bot.turnToPoint(mogo2, /* lemlib::DriveSide::RIGHT, */ 500,
  //                 {.forwards = false, .minSpeed = 127, .earlyExitRange =
  //                 30});
  // // Go to grab mogo
  // bot.moveToPose(mogo2, 2200, {.forwards = false, .lead = .4});

  // // Stop intake if a ring is sensed
  // waitUntil([] {
  //   return bot.intake.getSensedRing() == COLOR::RED || !bot.isInMotion();
  // });
  // bot.intake.stop();
  // bot.waitUntilDone();
  // // Clamp mogo
  // bot.mogoClamp.retract();

  // // Second ring for second mogo
  // const Pose ring2_2 = {TILE, TILE, RED + 30};

  // // Back up to face second ring
  // bot.moveToPose(ring2_2 + Pose::fromPolar(-30, ring2_2.theta), 2000,
  //                {.forwards = false, .lead = .4, .minSpeed = 64});
  // pros::delay(50);

  // // Score first ring onto mogo
  // bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // // Wait until facing ring
  // waitUntil([ring2_2] {
  //   return robotAngDist(ring2_2.theta) < 20 || !bot.isInMotion();
  // });
  // bot.cancelMotion();

  // // Move to and intake second ring
  // bot.moveToPoint(ring2_2, 1400, {.minSpeed = 32});

  // const Pose ring2_3 = {0, 0, STANDS};
  // // Move to and intake third ring
  // bot.moveToPose(ring2_3, 1300,
  //                {
  //                    .lead = .4,
  //                    .minSpeed = 32,
  //                });

  // const Pose ring2_4 = {TILE, -TILE, STANDS - 45};

  // // Move to and intake fourth ring
  // bot.moveToPose(ring2_4, 1300, {.minSpeed = 32});

  // // Move to intake last ring
  // const Pose ring2_5 =
  //     Pose::fromPolar(-DRIVE_LENGTH / 2 + 8, STANDS - 45) + Pose {
  //                                                               TILE * 2,
  //                                                               -TILE * 2,
  //                                                           };

  // // Move to and intake fifth ring
  // bot.moveToPose(ring2_5, 1300);

  // // Wait until all rings intaked
  // waitUntil([] { return bot.intake.numOfMogoRings >= 5; }, 0, 2000);

  // const Pose depositMogo2 =
  //     Pose::fromPolar(DRIVE_LENGTH / 2 - 2, RED_STATION + 45) +
  //     Pose {MAX_X, MIN_Y};

  // // get rings away
  // bot.turnToHeading(BLUE_STATION, 500, {.minSpeed = 127, .earlyExitRange =
  // 30}); bot.turnToHeading(STANDS, 500, {.minSpeed = 127, .earlyExitRange =
  // 30}); bot.turnToHeading(BLUE_STATION, 500, {.minSpeed = 127,
  // .earlyExitRange = 30});
  // // // Go into corner
  // // bot.moveToPose(depositMogo2.withTheta(depositMogo2.theta + 180), 800,
  // //                {.forwards = true});

  // // Turn to corner
  // bot.turnToPoint(depositMogo2, 800,
  //                 {.forwards = false, .minSpeed = 64, .earlyExitRange = 30});

  // // Move to corner
  // bot.moveToPose(depositMogo2, 1300, {.forwards = false});

  // // Unclamp and dont clamp w/ intake
  // bot.mogoClamp.extend();
  // bot.intake.outtake();
  // pros::delay(100);
  // bot.intake.stop();

  // // Reset number of mogo rings
  // bot.intake.numOfMogoRings = 0;

  // bot.waitUntilDone();
  // tank(127, 127, 300, 0);
  // // Prepare for blue mogo
  // const Pose prepareForBlueMogo {TILE * 1.25, -TILE * 1.25, RED_STATION};
  // bot.moveToPose(prepareForBlueMogo, 2000,
  //                {.forwards = false, .minSpeed = 127, .earlyExitRange = 10});
  // waitUntilDistToPose(prepareForBlueMogo, 12);
  // bot.cancelMotion();

  // // Go to blue mogo
  // const Pose blueMogo {TILE * 2.5, TILE - DRIVE_LENGTH / 2, STANDS};
  // bot.moveToPose(blueMogo, 3000, {.forwards = false});

  // bot.waitUntilDone();
  // // Clamp mogo
  // bot.mogoClamp.retract();

  // // Line up for deposit mogo
  // bot.moveToPose({MAX_X - DRIVE_WIDTH / 2 - 3, TILE * 1.75, REF}, 2000,
  //                {.forwards = false, .minSpeed = 32});

  // // Deposit blue mogo
  // bot.moveToPose(
  //     {MAX_X - DRIVE_WIDTH / 2 - 3, MAX_X - DRIVE_LENGTH / 2 - 3, REF}, 2000,
  //     {.forwards = false});

  // // Unclamp and dont clamp w/ intake
  // bot.mogoClamp.extend();
  // bot.intake.outtake();
  // pros::delay(100);
  // bot.intake.stop();

  // // Reset number of mogo rings
  // bot.intake.numOfMogoRings = 0;

  // // Go around stacks of rings
  // bot.moveToPose({TILE * 2, TILE * 1.5, RED_STATION}, 2000,
  //                {.minSpeed = 64, .earlyExitRange = 6});

  // // Intake 1st ring for new mogo
  // bot.moveToPose({TILE, TILE * 2, REF - 30}, 1500,
  //                {.minSpeed = 64, .earlyExitRange = 4});
  // bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // // Grab mogo
  // bot.moveToPose(Pose::fromPolar(DRIVE_LENGTH / 2, BLUE - 30) +
  //                    Pose {-TILE * 2, TILE},
  //                2500, {.forwards = false, .lead = .4});
  // // Pause intake once sensed
  // waitUntil([] {
  //   return bot.intake.getSensedRing() == COLOR::RED || !bot.isInMotion();
  // });
  // bot.intake.stop();
  // bot.waitUntilDone();

  // // Clamp mogo
  // bot.mogoClamp.retract();
  // pros::delay(300);

  // const Pose depositMogo3 =
  //     Pose::fromPolar(DRIVE_LENGTH / 2 - 4, STANDS - 45) + Pose {MIN_X,
  //     MAX_Y};
  // bot.turnToPoint(depositMogo3, 800, {.forwards = false});
  // bot.moveToPose(depositMogo3, 2000, {.forwards = false, .lead = .8});

  // bot.waitUntilDone();
  // // Unclamp and dont clamp w/ intake
  // bot.mogoClamp.retract();
  // bot.intake.outtake();
  // pros::delay(100);
  // bot.intake.stop();

  // bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  // bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // bot.moveToPose({-TILE * .75, TILE * .75, BLUE + 45}, 3000, {.minSpeed =
  // 96}); bot.intake.stop();

  // tank(127, 127, 400, 0);
  // pros::delay(500);
  // pros::delay(1000);
  // stop();
}
