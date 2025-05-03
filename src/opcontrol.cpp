#include "dimensions.h"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"
#include "pros/misc.h"
#include <optional>
#include <string>
#include "pros/rtos.hpp"
#include "subsystems/intake.h"
#include "leds/functions.h"
#include "auton/util.h"
#include "robot.h"
#include "main.h"
#include "tuning.h"

namespace controller_mapping {
typedef pros::controller_analog_e_t axis_t;
typedef pros::controller_digital_e_t button_t;

constexpr axis_t LEFT_DRIVE = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr axis_t RIGHT_DRIVE = pros::E_CONTROLLER_ANALOG_RIGHT_Y;

constexpr button_t INTAKE = pros::E_CONTROLLER_DIGITAL_L1;
constexpr button_t OUTTAKE = pros::E_CONTROLLER_DIGITAL_L2;
constexpr button_t INTAKE_FILTER_COLOR_CHANGE = pros::E_CONTROLLER_DIGITAL_A;

constexpr button_t NEXT_LIFT_STATE = pros::E_CONTROLLER_DIGITAL_R2;

// Pneumatic controls
constexpr button_t MOGO_CLAMP_TOGGLE = pros::E_CONTROLLER_DIGITAL_R1;
constexpr button_t INTAKE_LIFT_TOGGLE = pros::E_CONTROLLER_DIGITAL_X;
constexpr button_t LEFT_DOINKER_TOGGLE = pros::E_CONTROLLER_DIGITAL_RIGHT;
constexpr button_t RIGHT_DOINKER_TOGGLE = pros::E_CONTROLLER_DIGITAL_Y;
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
  COLOR filterColor = bot.selector.getAlliance();
  auto intakeStateToStr = overloads(
      [](Intake::FilteringState& state) { return destToStrOp(state.dest); },
      [filterColor](Intake::Filtering& state) {
        return destToStrOp(filterColor == COLOR::BLUE ? state.blue : state.red);
      },
      [](Intake::BaseState& state) { return std::string("none"); });

  pros::Controller master(pros::E_CONTROLLER_MASTER);
  // Drop intake automatically
  bot.intakeLift.retract();

  /** The color of ring we are directing to the destination with the filter. */
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

  // Timer to release the ring from the intake
  lemlib::Timer releaseLBRingTimer {30};
  releaseLBRingTimer.pause();

  while (true) {
    if (bot.lady.getState() == LadyBrown::COAST &&
        lemlib::angleError(bot.lady.calcLBAngle(), bot.lady.m_config.idle,
                           false) < 10) {
      bot.lady.setState(LadyBrown::IDLE);
    }

    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    // Mogo control
    if (master.get_digital_new_press(map::MOGO_CLAMP_TOGGLE))
      bot.mogoClamp.toggle();
    // Intake lift control
    if (master.get_digital_new_press(map::INTAKE_LIFT_TOGGLE))
      bot.intakeLift.toggle();
    // Doinker control
    if (master.get_digital_new_press(map::LEFT_DOINKER_TOGGLE))
      bot.leftDoinker.toggle();
    if (master.get_digital_new_press(map::RIGHT_DOINKER_TOGGLE))
      bot.rightDoinker.toggle();

    // Lift control
    if (master.get_digital(map::NEXT_LIFT_STATE)) {
      if (auto& start = holdStartTime) {
        const uint32_t duration = pros::millis() - *start;
        // If this is a long press, go to flip
        if (duration > longPressDuration) { bot.lady.flip(); }
      } else { // Button was not pressed before (new press)
        holdStartTime = pros::millis();

        // If moving on from collect, then outtake a little bit to release ring
        if (bot.lady.getState() == LadyBrown::COLLECT) {
          bot.intake.outtake();
          releaseLBRingTimer.reset();
          releaseLBRingTimer.resume();
        }

        // Should only ever go to idle, collect, or flip as these are the only
        // useful states for driver
        switch (bot.lady.getState()) {
          case LadyBrown::State::IDLE: bot.lady.collect(); break;
          case LadyBrown::State::COLLECT: bot.lady.flip(); break;
          case LadyBrown::State::IDLE_SCORE: bot.lady.flip(); break;
          case LadyBrown::State::RUN_INTO_ALLY: bot.lady.flip(); break;
          case LadyBrown::State::SCORE: bot.lady.flip(); break;
          case LadyBrown::State::FLIP: bot.lady.goToIdle(); break;
          default: bot.lady.goToIdle(); break;
        }
      }
    } else { // If lift button not pressed
      if (auto& start = holdStartTime) {
        const uint32_t duration = pros::millis() - *start;
        if (duration > longPressDuration) { bot.lady.goToIdle(); }
      }
      holdStartTime = std::nullopt;
    }

    if (releaseLBRingTimer.isDone()) {
      bot.intake.stop();
      releaseLBRingTimer.reset();
      releaseLBRingTimer.pause();
    }

    // === Intake control

    if (master.get_digital_new_press(map::INTAKE_FILTER_COLOR_CHANGE)) {
      filterColor = !filterColor;
    }

    // Intake button
    if (master.get_digital(map::INTAKE)) {
      // Rising edge of Intake button

      // Override lb release functionality
      releaseLBRingTimer.reset();
      releaseLBRingTimer.pause();

      bot.intake.intakeTo(filterColor, Intake::LB);
      bot.intake.intakeTo(!filterColor, Intake::KICK);
      // Outtake button
    } else if (master.get_digital(map::OUTTAKE)) {
      // Override lb release functionality
      releaseLBRingTimer.reset();
      releaseLBRingTimer.pause();

      bot.intake.outtake();
    }
    // If intake button is not pressed, and (timer is done or is paused)
    else if (releaseLBRingTimer.isPaused() || releaseLBRingTimer.isDone()) {
      // Then stop outtaking
      releaseLBRingTimer.reset();
      releaseLBRingTimer.pause();
      bot.intake.stop();
    }

    auto intakeState = bot.intake.getState();
#ifdef TUNING
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
      tuningCLI();
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      bot.moveToPoint(
          {-dimensions::field::TILE * 2, -dimensions::field::TILE * 2}, 3000);
      auton::util::waitUntil([&] {
        return master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) ||
               !bot.isInMotion();
      });
    }
#endif
    pros::delay(5); // Run every 20ms (refresh rate of the controller)
  }
}