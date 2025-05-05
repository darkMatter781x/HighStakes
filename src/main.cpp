#include "main.h"
#include "color.h"
#include "config.h"
#include "dimensions.h"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include "pros/apix.h"
#include "units/units.hpp"
#include "units/Temperature.hpp"
#include <iterator>
#include <sstream>

std::vector<std::int8_t> getAllMotors() {
  std::vector<std::int8_t> ports;

  // Combine multiple vectors into the ports vector
  auto leftPorts = bot.m_config.motors.left.get_port_all();
  auto rightPorts = bot.m_config.motors.right.get_port_all();
  auto intakePorts = bot.m_config.motors.intake.get_port_all();
  auto ladyPorts = bot.m_config.motors.LadyBrown.get_port_all();

  // Insert all ports into the ports vector
  ports.insert(ports.end(), leftPorts.begin(), leftPorts.end());
  ports.insert(ports.end(), rightPorts.begin(), rightPorts.end());
  ports.insert(ports.end(), intakePorts.begin(), intakePorts.end());
  ports.insert(ports.end(), ladyPorts.begin(), ladyPorts.end());

  return ports;
}

struct MotorTemp {
    std::int8_t port;
    Temperature temp;
};

/** @returns motor ports that are so hot that vexos limits current */
std::vector<MotorTemp> getHotMotors() {
  /** Temperature above which the motors are limited */
  constexpr Temperature limitedTemp = 55.0_celsius; // degrees celsius
  static const std::vector<std::int8_t> allMotors = getAllMotors();
  std::vector<MotorTemp> motorTemps;
  std::transform(
      allMotors.begin(), allMotors.end(), std::back_inserter(motorTemps),
      [](std::int8_t port) -> MotorTemp {
        return {port,
                units::from_celsius(pros::c::motor_get_temperature(port))};
      });
  std::vector<MotorTemp> hotMotors;

  // for (auto& mTemp : motorTemps) {
  //   printf("[MotorTemp] %d: %f\n", mTemp.port,
  //   units::to_celsius(mTemp.temp));
  // }

  std::copy_if(motorTemps.begin(), motorTemps.end(),
               std::back_inserter(hotMotors),
               [&](MotorTemp& mTemp) { return mTemp.temp >= limitedTemp; });

  return hotMotors;
}

void screen() {
  while (1) {
    // Only print if auton selection is complete
    if (bot.selector.isLocked()) {
      pros::lcd::print(0, "target\tang\t=err");
      pros::lcd::print(1, "%4.2f\t-%4.2f\t=%4.2f", bot.lady.getTargetAngle(),
                       bot.lady.calcLBAngle(), bot.lady.calcError());
      pros::lcd::print(3, "x:\t%fin", bot.getPose().x);
      pros::lcd::print(4, "y:\t%fin", bot.getPose().y);
      pros::lcd::print(5, "theta:\t%fdeg", bot.getPose().theta);
      pros::lcd::print(6, "intake:\t%1.3frpm",
                       bot.m_config.motors.intake.get_actual_velocity());

      auto hotMotors = getHotMotors();
      if (hotMotors.size() > 0) {
        std::stringstream ss;
        for (size_t i = 0; i < hotMotors.size(); i++) {
          ss << hotMotors[i].port << ", ";
        }
        pros::lcd::print(7, "hot motors: %s", ss.str().c_str());
        printf("hot motors: %s", ss.str().c_str());
        pros::c::controller_print(pros::E_CONTROLLER_MASTER, 3, 1, "HOT!!");
      } else {
        pros::lcd::clear_line(7);
      }
    }
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
#ifdef VEXIDE_SIMULATION
  pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
#endif
  // ensure robot is initialized
  Robot::get();
  // indicate calibration
  pros::delay(250);
  bot.calibrate();

  bot.setAlliance(ALLIANCE::RED);
  bot.setPose({0, 0, dimensions::field::REFEREE}, 72);
  bot.localization.enabled = true;
  pros::delay(250);

  printf("Initialized!\n");
  // // LED Testing
  // LED Testing
  // pros::delay(1000);
  // std::vector<std::shared_ptr<LedStrip>> strips;
  // for (char adiPort = 'A'; adiPort <= 'H'; adiPort++) {
  //   strips.push_back(
  //       LedStrip::create({8, adiPort}, size_t(adiPort - 'A' + 1), 0.1));
  // }
  // int i;
  // while (i++ < 1) {
  //   // for (auto strip : strips) {
  //   //   strip->buf().setAll(0x550000);
  //   //   pros::delay(1000);
  //   // }
  //   // for (char adiPort = 'H'; adiPort >= 'A'; adiPort--) {
  //   //   // pros::adi::LED strip {{8, adiPort}, size_t(adiPort - 'A' + 1)};
  //   //   // strip.set_all(0xFFFFFF);
  //   //   // pros::delay(250);
  //   // }
  //   // RobotConfig::LEDs::leds.rightUnderGlow->buf().setAll(0xFF0000);
  //   // pros::delay(250);
  //   RobotConfig::LEDs::leds.leftUnderGlow->buf().setAll(0x00FF00);
  //   // pros::delay(250);
  //   // RobotConfig::LEDs::leds.leftLift->buf().setAll(0xFF0000);
  //   // // pros::delay(250);
  //   // RobotConfig::LEDs::leds.rightLift->buf().setAll(0xFF0000);
  //   // // pros::delay(250);
  // }

  // RobotConfig::LEDs::leds.leftUnderGlow.set_all(0x550000);
  // RobotConfig::LEDs::leds.rightUnderGlow.set_all(0x005500);
  // RobotConfig::LEDs::leds.lift.set_all(0x000055);
  // pros::delay(1000);
  // RobotConfig::LEDs::leds.leftUnderGlow.set_all(0x550055);
  // RobotConfig::LEDs::leds.rightUnderGlow.set_all(0x550055);
  // RobotConfig::LEDs::leds.lift.set_all(0x550055);
  // pros::delay(1000);
  // RobotConfig::LEDs::leds.leftUnderGlow.set_all(0x555500);
  // RobotConfig::LEDs::leds.rightUnderGlow.set_all(0x555500);
  // RobotConfig::LEDs::leds.lift.set_all(0x555500);
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
void autonomous() { bot.selector.runAuton(); }