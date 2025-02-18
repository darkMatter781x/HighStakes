#include "lemlib/util.hpp"
#include "pros/misc.h"
#include <optional>
#include <string>
#include "pros/rtos.hpp"
#include "subsystems/intake.h"
#include "leds/functions.h"
#include "auton/util.h"
#include "robot.h"
#include "main.h"

namespace controller_mapping {
typedef pros::controller_analog_e_t axis_t;
typedef pros::controller_digital_e_t button_t;

constexpr axis_t LEFT_DRIVE = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr axis_t RIGHT_DRIVE = pros::E_CONTROLLER_ANALOG_RIGHT_Y;

constexpr button_t INTAKE = pros::E_CONTROLLER_DIGITAL_L1;
constexpr button_t INTAKE_FILTER_COLOR_CHANGE = pros::E_CONTROLLER_DIGITAL_A;
constexpr button_t OUTTAKE = pros::E_CONTROLLER_DIGITAL_L2;

constexpr button_t NEXT_LIFT_STATE = pros::E_CONTROLLER_DIGITAL_R2;

// Pneumatic controls
constexpr button_t MOGO_CLAMP_TOGGLE = pros::E_CONTROLLER_DIGITAL_R1;
constexpr button_t INTAKE_LIFT_TOGGLE = pros::E_CONTROLLER_DIGITAL_X;
constexpr button_t DOINKER_TOGGLE = pros::E_CONTROLLER_DIGITAL_Y;
}; // namespace controller_mapping
namespace map = controller_mapping;

// TODO: deduplicate this with the one in intake.cpp
template <class... Ts> struct overloads : Ts... {
    using Ts::operator()...;
};
template <class... Ts> overloads(Ts...) -> overloads<Ts...>;

std::string destToStrOp(Intake::DESTINATION dest) {
  switch (dest) {
    case Intake::MOGO: return "MOGO";
    case Intake::KICK: return "KICK";
    case Intake::LB: return "LIFT";
  }
}

void skillsMacro() {}

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
  // Drop intake automatically
  bot.intakeLift.retract();

  /** The color of ring we are directing to the destination with the filter. */
  COLOR filterColor = bot.selector.getAlliance();
  auto intakeStateToStr = overloads(
      [](Intake::FilteringState& state) { return destToStrOp(state.dest); },
      [filterColor](Intake::Filtering& state) {
        return destToStrOp(filterColor == COLOR::BLUE ? state.blue : state.red);
      },
      [](Intake::BaseState& state) { return std::string("none"); });
  bool prevIntakeBtn = false;

  if (strcmp(bot.selector.getAuton().label, "Skills") == 0) {
    auton::util::waitUntil(
        [&master] { return std::abs(master.get_analog(map::LEFT_DRIVE)) > 8; });
    skillsMacro();
  }

  /** Duration of hold for a hold */
  constexpr uint32_t longPressDuration = 200;
  std::optional<uint32_t> holdStartTime = std::nullopt;

  int time_since_press = INT_MAX - 1000;
  int startTime = INT_MAX - 1000;
  while (true) {
    if (bot.lady.getState() == LadyBrown::COAST &&
        lemlib::angleError(bot.lady.calcLBAngle(), bot.lady.m_config.idle,
                           false) < 10) {
      bot.lady.setState(LadyBrown::IDLE);
    }

    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    // TODO: Move this responsibility to led code
    // LED control

    if (/* bot.led.getState() == LEDs::FILTER_COLOR */ true) {
      // if (pros::millis() % 100 < 10)printf("%s\n", filterColor == COLOR::RED
      // ? "RED" : "BLUE");
      bot.led.displayFilterColor(filterColor);
      // startTime = pros::millis();
      /*printf("change color\n");
      if (pros::millis() > startTime + 100) {
        printf("other strip\n");
        bot.led.rightDisplayFilterColor(filterColor);
      } */
      // pros::delay(100);
      // bot.led.rightDisplayFilterColor(filterColor);
    }

    // Intake control
    if (master.get_digital(map::INTAKE)) {
      // Rising edge of Intake button

      // Intake current color to mogo, other color to kicker
      if (bot.lady.getState() == LadyBrown::COLLECT) {
        bot.intake.intakeTo(filterColor, Intake::LB);
        bot.intake.intakeTo(!filterColor, Intake::KICK);
      } else {
        bot.intake.intakeTo(filterColor, Intake::MOGO);
        bot.intake.intakeTo(!filterColor, Intake::KICK);
      }
    } else if (master.get_digital(map::OUTTAKE)) bot.intake.outtake();
    else bot.intake.stop();

    // Mogo control
    if (master.get_digital_new_press(map::MOGO_CLAMP_TOGGLE))
      bot.mogoClamp.toggle();
    // Intake lift control
    if (master.get_digital_new_press(map::INTAKE_LIFT_TOGGLE))
      bot.intakeLift.toggle();
    // Doinker control
    if (master.get_digital_new_press(map::DOINKER_TOGGLE)) bot.doinker.toggle();

    // Lift control

    if (master.get_digital(map::NEXT_LIFT_STATE)) {
      if (auto& start = holdStartTime) {
        const uint32_t duration = pros::millis() - *start;
        // If this is a long press, go to score
        if (duration > longPressDuration) { bot.lady.flip(); }
      } else { // Button was not pressed before (new press)
        holdStartTime = pros::millis();
        bot.lady.nextState();
      }
    } else { // If lift button not pressed
      if (auto& start = holdStartTime) {
        const uint32_t duration = pros::millis() - *start;
        if (duration > longPressDuration) { bot.lady.goToIdle(); }
      }
      holdStartTime = std::nullopt;
    }

    if (bot.lady.getState() == LadyBrown::COLLECT &&
        (std::holds_alternative<Intake::IntakingToKick>(
             bot.intake.getState()) ||
         std::holds_alternative<Intake::KickingRing>(bot.intake.getState()))) {
      bot.lady.goToIdle();
    }

    // Toggle filtering color
    if (master.get_digital_new_press(map::INTAKE_FILTER_COLOR_CHANGE)) {
      filterColor = !filterColor;
    }
    auto intakeState = bot.intake.getState();
    master.print(1, 0, "%s->%s", filterColor == COLOR::RED ? "R" : "B",
                 std::visit(intakeStateToStr, intakeState));
    // #ifdef TUNING
    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
    //         master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    //       tuningCLI();
    // #endif
    pros::delay(5); // Run every 20ms (refresh rate of the controller)
  }
}