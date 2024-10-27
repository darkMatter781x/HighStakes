#include "main.h"
#include "pros/misc.h"
#include "robot.h"
#include "tuning.h"

namespace controller_mapping {
const pros::controller_analog_e_t LEFT_DRIVE = pros::E_CONTROLLER_ANALOG_LEFT_Y;
const pros::controller_analog_e_t RIGHT_DRIVE =
    pros::E_CONTROLLER_ANALOG_RIGHT_Y;
const pros::controller_digital_e_t INTAKE = pros::E_CONTROLLER_DIGITAL_L1;
const pros::controller_digital_e_t INTAKE_FILTER_TO_LIFT =
    pros::E_CONTROLLER_DIGITAL_R2;
const pros::controller_digital_e_t INTAKE_FILTER_COLOR_CHANGE =
    pros::E_CONTROLLER_DIGITAL_A;
const pros::controller_digital_e_t OUTTAKE = pros::E_CONTROLLER_DIGITAL_L2;
const pros::controller_digital_e_t MOGO = pros::E_CONTROLLER_DIGITAL_R1;
const pros::controller_digital_e_t LIFT_UP = pros::E_CONTROLLER_DIGITAL_UP;
const pros::controller_digital_e_t LIFT_DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;

}; // namespace controller_mapping
namespace map = controller_mapping;

/** @brief Returns the opposite color. */
inline Intake::COLOR operator!(Intake::COLOR color) {
  switch (color) {
    case Intake::COLOR::RED: return Intake::COLOR::BLUE;
    case Intake::COLOR::BLUE: return Intake::COLOR::RED;
  }
}

// TODO: deduplicate this with the one in intake.cpp
template <class... Ts> struct overloads : Ts... {
    using Ts::operator()...;
};
template <class... Ts> overloads(Ts...) -> overloads<Ts...>;

std::string destToStrOp(Intake::DESTINATION dest) {
  switch (dest) {
    case Intake::MOGO: return "MOGO";
    case Intake::KICK: return "KICK";
    case Intake::LIFT: return "LIFT";
  }
}

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
  /** The color of ring we are directing to the destination with the filter. */
  Intake::COLOR filterColor = Intake::COLOR::RED;
  auto intakeStateToStr = overloads(
      [](Intake::FilteringState& state) { return destToStrOp(state.dest); },
      [filterColor](Intake::Filtering& state) {
        return destToStrOp(filterColor == Intake::BLUE ? state.blue : state.red);
      },
      [](Intake::BaseState& state) { return std::string("none"); });
  bool prevIntakeBtn = false;

  while (true) {
    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    // Intake control
    const bool intakeBtn = master.get_digital(map::INTAKE);

    if (intakeBtn) {
      if (!prevIntakeBtn) {
        // Rising edge of Intake button

        // Intake current color to mogo, other color to kicker
        bot.intake.intakeTo(filterColor, Intake::MOGO);
        bot.intake.intakeTo(!filterColor, Intake::KICK);
      }
      if (master.get_digital(map::INTAKE_FILTER_TO_LIFT)) {
        // Selected color to lift
        bot.intake.intakeTo(filterColor, Intake::LIFT);
      }
    } else if (master.get_digital(map::OUTTAKE)) bot.intake.outtake();
    else bot.intake.stop();

    prevIntakeBtn = intakeBtn;

    // Mogo control
    if (master.get_digital_new_press(map::MOGO)) bot.mogo.toggle();

    // Lift control
    if (master.get_digital_new_press(map::LIFT_UP)) bot.lift.goUp();
    if (master.get_digital_new_press(map::LIFT_DOWN)) bot.lift.goDown();

    // Toggle filtering color
    if (master.get_digital_new_press(map::INTAKE_FILTER_COLOR_CHANGE))
      filterColor = !filterColor;
    auto intakeState = bot.intake.getState();
    master.print(1, 0, "%s->%s", filterColor == Intake::COLOR::RED ? "R" : "B",
                 std::visit(intakeStateToStr, intakeState));
#ifdef TUNING
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
      tuningCLI();
#endif

    pros::delay(20); // Run every 20ms (refresh rate of the controller)
  }
}