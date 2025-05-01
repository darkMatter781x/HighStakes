#include "auton/autons.h"
#include "auton/util.h"
#include "color.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "units/units.hpp"

using namespace dimensions::all;
using namespace DIR;
using lemlib::Pose;

using namespace auton::util;

void positiveRushFunc(ALLIANCE alliance);

auton::Auton auton::autons::posRush = {
    .label = (char*)"Positive Rush",
    .run = positiveRushFunc,
    .labelForController = (char*)"pos rush",
    .description = (char*)"Gets the goal and makes someone cry :(",
};

// Autonomous for positive rush (side of the field with positive corners)
void positiveRushFunc(ALLIANCE alliance) {
  bot.setAlliance(alliance);
  bot.localization.enabled = false;

  const auto defaultAngPID = bot.angularPID;
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngLargeExit = bot.angularLargeExit;
  const auto defaultAngSmallExit = bot.angularSmallExit;
  const auto defaultLatLargeExit = bot.lateralLargeExit;
  const auto defaultLatSmallExit = bot.lateralSmallExit;

  bot.intake.numOfMogoRings = 0;

  const Time startTime = pros::millis() * msec;

  const Pose startPose {-TILE * 2 - 27. / 2. / 2. - 0.2, (TILE * 1 + DRIVE_WIDTH / 2 + 1.9) * -1, BLUE_STATION + 9.5};
  bot.setPose(startPose);

  const Pose rushGoal {0, TILE * -2};
  const Pose safeGoal {- TILE, - TILE};
  const Pose safeStack { - TILE, -TILE * 2, AUDIENCE};
  const Pose centerRing {0 + RING_DIAM/2, 0 - RING_DIAM/2};
  const Pose posCorner { MIN_X, MIN_Y };
  const Pose targetRushGoal {Pose::fromPolar(-13, bot.getPose().theta) + Pose {0 , + DRIVE_WIDTH / 2 } + rushGoal};

  // === moves towards the rush goal and doinks it
  bot.moveToPoint(targetRushGoal.withTheta(BLUE_STATION + 9.5), 1000);
  waitUntilDistToPose(targetRushGoal, 6, 0, true);
  switch (bot.getColor()) {
    case ALLIANCE::BLUE:  bot.leftDoinker.extend();
    case ALLIANCE::RED: bot.rightDoinker.extend();
  }  
  bot.waitUntilDone();
  const Pose moveBack {Pose::fromPolar(-12, bot.getPose().theta) + targetRushGoal};
  bot.moveToPose(moveBack.withTheta(BLUE_STATION + 9.5), 1000, {.forwards = false});
  bot.waitUntilDone();
  pros::delay(100);
  bot.leftDoinker.retract();
  bot.rightDoinker.retract();
  pros::delay(250);
  bot.moveToPose(moveBack.withTheta(BLUE_STATION + 9.5) + Pose::fromPolar(-7, bot.getPose().theta), 1000, {.forwards = false});
  bot.waitUntilDone();


  // === clamps the safe goal and intakes the preload
  bot.moveToPose(safeGoal.withTheta(AUDIENCE), 1000, {  .forwards = false, .lead = 0.2, .minSpeed = 32 });
  bot.waitUntilDone();
  tank(-32, -32, 0, 0);
  pros::delay(300);
  bot.mogoClamp.retract();
  stop();
  tank(32, 32, 0,0);
  pros::delay(450);
  stop();
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  pros::delay(500);

  
  // === doinks the center ring
  Pose targetCenterRing { Pose::fromPolar(-12.5, BLUE_STATION + 5) + centerRing + Pose { -1 , - DRIVE_WIDTH / 2 + 2}};
  bot.turnToPoint(targetCenterRing, 1000);
  bot.waitUntilDone();
  bot.intake.stop();
  bot.intakeLift.extend();
  bot.moveToPoint(targetCenterRing.withTheta(BLUE_STATION + 5), 1000);
  bot.waitUntilDone();
  switch (bot.getColor()) {
    case ALLIANCE::BLUE: bot.leftDoinker.extend();
    case ALLIANCE::RED: bot.rightDoinker.extend();
  }
  pros::delay(400);

  // == backs out from the center, un - doinks the ring, and intaks the doinked ring 
  bot.moveToPoint(Pose { - TILE / 2, -8 }, 1000, {.forwards = false, .minSpeed = 72});
  bot.waitUntilDone();
  bot.intakeLift.retract();
  bot.moveToPose(Pose::fromPolar(-12.5, BLUE_STATION) + Pose { - TILE, - TILE + DRIVE_WIDTH /2}, 1000, {.forwards = false});
  bot.waitUntilDone();
  bot.leftDoinker.retract();
  bot.rightDoinker.retract();
  

  // === swing turns to intake the doinked ring and the corner stack
  const Pose andrewAte { -2.5 * TILE, -2.5 * TILE, RED_STATION };

  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.swingToPoint(posCorner, lemlib::DriveSide::RIGHT, 4000, {.minSpeed = 72});
  bot.moveToPose(safeStack, 1000, {.minSpeed = 72});
  bot.waitUntilDone();
  bot.moveToPose(andrewAte, 1000, {.maxSpeed = 96, .minSpeed = 72});
  bot.waitUntilDone();
  bot.moveToPoint(posCorner, 2000, {.maxSpeed = 72});
  waitUntil([&] {
    return bot.intake.numOfMogoRings == 4 || !bot.isInMotion();
  });
  tank(-32, -32, 0 , 0);
  pros::delay(300);
  stop();

  // === intakes the stake ring
  const Pose allianceStack ( -2 * TILE, 0, REFEREE );
  bot.moveToPose(allianceStack - Pose::fromPolar(- DRIVE_LENGTH / 2, REFEREE), 1000);
  bot.waitUntil(30);
  bot.intakeLift.extend();

  

  //scraped auto
  /*
  // === moves towards the rush goal, and scores the ring using the lady brown
  //lady brown starting position should be collect
  bot.lady.collect();
  //moves bot to the target mobile goal and scores on it with lady brown
  bot.moveToPoint(Pose::fromPolar(-15, bot.getPose().theta) + rushGoal, 2000);
  bot.waitUntil(22);
  bot.lady.flip();
  pros::delay(700);

  // === removes the ring/goal from the lady brown and clamps onto a goal
  bot.moveToPose(safeGoal.withTheta(AUDIENCE) + Pose::fromPolar(-6, bot.getPose().theta), 3000, {.forwards = false, .earlyExitRange = 10});
  bot.waitUntil(4);
  bot.lady.score();
  bot.turnToHeading(AUDIENCE, 1000);
  bot.waitUntilDone();
  bot.moveToPose(safeGoal.withTheta(AUDIENCE) + Pose::fromPolar(-10, bot.getPose().theta), 1000, {.forwards = false});
  bot.waitUntil(60);
  bot.mogoClamp.retract();
  bot.waitUntilDone();
  pros::delay(250);
  

  // === gets the center ring with the doinker
  bot.turnToPoint(Pose::fromPolar(7.5, BLUE_STATION + 10) + centerRing, 1000);
  bot.waitUntilDone();
  bot.moveToPoint(centerRing, 1000, {.maxSpeed = 42});
  bot.waitUntilDone();
  switch(bot.getColor()){
    case COLOR::BLUE: bot.leftDoinker.extend(); break;
    case COLOR::RED: bot.rightDoinker.extend(); break;
  }

  //moves bot to the target doinked ring pose and intakes the ring
  const Pose targetDoinkedRing {Pose::fromPolar(20, bot.getPose().theta) + Pose { - TILE, - TILE }};
  const Pose doinkedRing { - TILE, - TILE, AUDIENCE};
  bot.moveToPose(targetDoinkedRing, 1000, {.forwards = false});
  bot.intake.intakeTo(bot.getColor(), Intake::MOGO);
  bot.intake.intakeTo(!bot.getColor(), Intake::KICK);
  bot.moveToPose(doinkedRing, 1000, {.minSpeed = 72});
  bot.waitUntilDone();
  */







}
