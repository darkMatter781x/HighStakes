#include "main.h"
#include "robot.h"

namespace ControllerMapping {
const pros::controller_analog_e_t LEFT_DRIVE =
    pros::E_CONTROLLER_ANALOG_LEFT_Y;
const pros::controller_analog_e_t RIGHT_DRIVE =
    pros::E_CONTROLLER_ANALOG_RIGHT_Y;
const pros::controller_digital_e_t INTAKE = pros::E_CONTROLLER_DIGITAL_L1;
const pros::controller_digital_e_t OUTTAKE = pros::E_CONTROLLER_DIGITAL_L2;
const pros::controller_digital_e_t MOGO = pros::E_CONTROLLER_DIGITAL_R1;
}; // namespace ControllerMapping
namespace map = ControllerMapping;

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    if (master.get_digital(map::INTAKE)) bot.intake.intake();
    else if (master.get_digital(map::OUTTAKE)) bot.intake.outtake();
    else bot.intake.stop();

    if (master.get_digital_new_press(map::MOGO)) bot.mogo.toggle();

    pros::delay(20); // Run for 20 ms then update
  }
}