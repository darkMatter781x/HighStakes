#include "main.h"
#include "pros/misc.h"
#include "robot.h"
#include "tuning.h"
#include "dimensions.h"
#include "auton.h"

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

void skillsMacro() {
  using namespace dimensions::all;
  using namespace DIR;
  using lemlib::Pose;
  using namespace auton::util;

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
                 {.minSpeed = 64, .earlyExitRange = 5});
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

  while (true) {
    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    // Intake control
    if (master.get_digital(map::INTAKE_FILTER_TO_LIFT)) {
      // Selected color to lift
      bot.intake.intakeTo(filterColor, Intake::LIFT);
      bot.intake.intakeTo(!filterColor, Intake::KICK);
    } else if (master.get_digital(map::INTAKE)) {
      // Rising edge of Intake button

      // Intake current color to mogo, other color to kicker
      bot.intake.intakeTo(filterColor, Intake::MOGO);
      bot.intake.intakeTo(!filterColor, Intake::KICK);
    } else if (master.get_digital(map::OUTTAKE)) bot.intake.outtake();
    else bot.intake.stop();

    // Mogo control
    if (master.get_digital_new_press(map::MOGO)) bot.mogo.toggle();

    // Lift control
    if (master.get_digital_new_press(map::LIFT_UP)) bot.lift.goUp();
    if (master.get_digital_new_press(map::LIFT_DOWN)) bot.lift.goDown();

    // Toggle filtering color
    if (master.get_digital_new_press(map::INTAKE_FILTER_COLOR_CHANGE))
      filterColor = !filterColor;
    auto intakeState = bot.intake.getState();
    master.print(1, 0, "%s->%s", filterColor == COLOR::RED ? "R" : "B",
                 std::visit(intakeStateToStr, intakeState));
#ifdef TUNING
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
      tuningCLI();
#endif

    pros::delay(20); // Run every 20ms (refresh rate of the controller)
  }
}