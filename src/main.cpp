#include "main.h"
#include "config.h"
#include "led.h"
#include "pros/rtos.hpp"
#include "robot.h"

void screen() {
  while (1) {
    pros::lcd::print(0, "target\tang\t=err");
    pros::lcd::print(1, "%4.2f\t-%4.2f\t=%4.2f", bot.lift.getTargetAngle(),
                     bot.lift.calcLiftAngle(), bot.lift.calcError());
    pros::lcd::print(3, "x:\t%fin", bot.getPose().x);
    pros::lcd::print(4, "y:\t%fin", bot.getPose().y);
    pros::lcd::print(5, "theta:\t%fdeg", bot.getPose().theta);
    pros::delay(50);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();

  // ensure robot is initialized
  Robot::get();
  bot.calibrate();
  // // LED Testing
  // LedStrip leftStrip {RobotConfig::LEDs::leds.leftUnderGlow};
  // LedStrip rightStrip {RobotConfig::LEDs::leds.rightUnderGlow};
  // pros::delay(500);
  // leftStrip.clear();
  // pros::delay(500);
  // rightStrip.clear();
  // // static color
  // leftStrip.setAll(0x550055);
  // rightStrip.setAll(0x550055);

  // // rotating gradient
  // leftStrip.setGradient(0x220022, 0x220000);
  // rightStrip.setGradient(0x220022, 0x220000);
  // bool a;
  // while (1) {
  //   a = !a;
  //   if (a) leftStrip.shift();
  //   else rightStrip.shift();
  //   pros::delay(30);
  // }

  new pros::Task {screen};
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}