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

  // Deploy basket
  bot.lift.goToMiddle();
  bot.intake.outtake();
  pros::delay(200);

  // Score preload on alliance stake
  bot.intake.intakeBothTo(Intake::MOGO);

  // Reset lift and intake
  pros::delay(280);
  bot.lift.goToBottom();
  bot.intake.stop();

  // Reset count of rings in mogo (dont include preload)
  bot.intake.numOfMogoRings = 0;

  // Wait until ring is scored
  pros::delay(150);

  // Get out of alliance stake
  bot.swingToHeading(REF, lemlib::DriveSide::LEFT, 500,
                     {.minSpeed = 127, .earlyExitRange = 30});

  // Go to mogo
  const Pose grabMogo1 = {-TILE * 2, -TILE + DRIVE_LENGTH / 2, REF};
  bot.moveToPose(grabMogo1, 1500, {.forwards = false});

  // Once next to mogo, grab it
  waitUntil([grabMogo1] {
    return bot.getPose().y < grabMogo1.y + 2 || !bot.isInMotion();
  });
  bot.mogo.close();
  bot.waitUntilDone();

  // Turn to next ring, but make sure to continue backing into mogo, to ensure
  // clamping
  bot.swingToHeading(REF + 60, lemlib::DriveSide::LEFT, 400,
                     {.minSpeed = 127, .earlyExitRange = 20});

  // Move towards first ring, and intake it
  const Pose ring1 = {-TILE, -TILE, BLUE_STATION + 30};
  bot.moveToPose(ring1, 1500, {.minSpeed = 64, .earlyExitRange = 5});

  // Turn on intake
  bot.intake.intakeTo(COLOR::RED, Intake::MOGO);
  bot.intake.intakeTo(COLOR::BLUE, Intake::KICK);

  // Move to second ring, next to wall stake

  bot.moveToPose({-0, -TILE * 2.5 + DRIVE_LENGTH / 2 - 2, STANDS}, 2000, {.lead=.5});
  // First of line of 3 rings
  const Pose ring3 = {-TILE, -TILE * 2 + DRIVE_LENGTH / 2, RED_STATION};

  // Turn towards line of rings
  bot.swingToPoint(ring3, lemlib::DriveSide::LEFT, 400,
                   {.minSpeed = 127, .earlyExitRange = 30});

  // Move to intake first ring, quickly at first
  bot.moveToPose(ring3, 700, {.minSpeed = 127, .earlyExitRange = 18});
  // Slow down to intake
  bot.moveToPose(ring3, 500,
                 {.maxSpeed = 64, .minSpeed = 24, .earlyExitRange = 3});

  // Intake next ring
  bot.moveToPose({-TILE * 2 + DRIVE_LENGTH / 2, -TILE * 2, RED_STATION}, 2500,
                 {.maxSpeed = 64, .earlyExitRange = 3});
  // Intake next next ring
  bot.moveToPose({-TILE * 2.5 - DRIVE_LENGTH / 2, -TILE * 2, RED_STATION}, 2500,
                 {.maxSpeed = 64, .earlyExitRange = 3});

  /** The ring to be scored on the first full wall stake */
  const Pose stakeRing2 {-TILE * 2.5, -TILE * 2, BLUE_STATION + 30};
  // Turn towards ring for stake
  bot.swingToPoint(stakeRing2, lemlib::DriveSide::LEFT, 500,
                   {.minSpeed = 127, .earlyExitRange = 45});

  // Move towards ring for stake ring
  bot.moveToPose(stakeRing2, 1500, {.minSpeed = 64, .earlyExitRange = 5});

  // Go to corner to deposit mogo
  bot.moveToPose(
      Pose {
          MIN_X,
          MIN_Y,
      } + Pose {1 / (float)sqrt(2), 1 / (float)sqrt(2)} *
              (DRIVE_LENGTH / 2 + 6),
      800, {.forwards = false, .minSpeed = 64, .earlyExitRange = 5});

  // Wait until all mogo rings are scored to intake to lift
  waitUntil([] { return bot.intake.numOfMogoRings >= 5 || !bot.isInMotion(); });
  bot.intake.intakeTo(COLOR::RED, Intake::LIFT);

  bot.waitUntilDone();
  // Deposit mogo in corner
  bot.mogo.open();

  // Reset number of mogo
  bot.intake.numOfMogoRings = 0;

  bot.moveToPose({0, MIN_Y - DRIVE_LENGTH / 2, STANDS}, 2500);
  bot.waitUntil(48);
  bot.lift.goToTop();

  bot.waitUntilDone();
  bot.lift.goToBottom();
}