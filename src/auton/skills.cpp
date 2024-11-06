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

// Autonomous for skills rush (as red)
void skills() {
  bot.setAlliance(ALLIANCE::RED);

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  const size_t startingLiftRings = bot.intake.numOfLiftRings;
  const size_t startingMogoRings = bot.intake.numOfMogoRings;

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

  // Wait until ring is scored
  pros::delay(150);

  // Get out of alliance stake
  bot.swingToHeading(REF, lemlib::DriveSide::LEFT, 500,
                     {.minSpeed = 127, .earlyExitRange = 30});

  bot.moveToPose({-TILE * 2, -TILE + DRIVE_LENGTH / 2, REF}, 1000,
                 {.forwards = false});
  bot.waitUntilDone();

  bot.mogo.close();
}

void autonomous() { skills(); }