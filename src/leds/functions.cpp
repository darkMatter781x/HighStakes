#include "leds/functions.h"
#include "color.h"
#include "leds/colors.h"
#include "leds/strip.h"
#include "config.h"
#include "subsystems/selector.h"
#include "robot.h"

#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <climits>
#include <iterator>
#include <string>

#define RED_HEX 0xFF000F
#define BLUE_HEX 0x0FFFFF

void LEDs::makeStripsLight(HexRGB hex) {
  if (pros::millis() % 50 > 25) {
    leftUnderGlow->buf().setAll(hex);
  } else {
    rightUnderGlow->buf().setAll(hex);
  }
}

void LEDs::displayFilterColor(COLOR color) {
  switch (color) {
    case ALLIANCE::RED: makeStripsLight(RED_HEX); break;
    case ALLIANCE::BLUE: makeStripsLight(BLUE_HEX); break;
    default: makeStripsLight(0xFFFFFF); break;
  }
}

void LEDs::displayDISCONNECT(int port) {
  if (getMotorDisconnect() == 12 || getMotorDisconnect() == 14 ||
      getMotorDisconnect() == 17) {
    // printf("l drive\n");
    leftUnderGlow->buf().setAll(0xFF0000);
    rightUnderGlow->buf().setAll(0x000000);
    leftLift->buf().setAll(0x000000);
    rightLift->buf().setAll(0x000000);
  } else if (getMotorDisconnect() == 13 || getMotorDisconnect() == 15 ||
             getMotorDisconnect() == 16) {
    // printf("r drive\n");
    leftUnderGlow->buf().setAll(0x000000);
    rightUnderGlow->buf().setAll(0xFF0000);
    leftLift->buf().setAll(0x000000);
    rightLift->buf().setAll(0x000000);
  } else if (getMotorDisconnect() == 11) {
    // printf("intake \n");
    leftUnderGlow->buf().setAll(0x000000);
    rightUnderGlow->buf().setAll(0x000000);
    leftLift->buf().setAll(0xFF0000);
    rightLift->buf().setAll(0x000000);
  } else {
    // printf("lb\n");
    leftUnderGlow->buf().setAll(0x000000);
    rightUnderGlow->buf().setAll(0x000000);
    leftLift->buf().setAll(0xFF0000);
    rightLift->buf().setAll(0xFF0000);
  }
}

void LEDs::autonMessage(HexRGB color, int time, int startTime) {
  setState(MESSAGE);
  this->time = time;
  this->startTime = startTime;
  this->color = color;
}

void LEDs::setState(State state) { m_state = state; }

int LEDs::getMotorDisconnect() {
  for (int i = 0; i < std::size(LEDs::motors); i++) {
    pros::Device device(LEDs::motors[i]);
    if (device.is_installed() == false) {
      // printf("not nerd\n");
      return motors[i];
    }
    // printf("nerd\n");
  }
  return 0;
}

LEDs::LEDs(std::shared_ptr<LedStrip> leftLift,
           std::shared_ptr<LedStrip> rightLift,
           std::shared_ptr<LedStrip> leftUnderGlow,
           std::shared_ptr<LedStrip> rightUnderGlow)
  : leftLift(leftLift), rightLift(rightLift), leftUnderGlow(leftUnderGlow),
    rightUnderGlow(rightUnderGlow) {}

LEDs::State LEDs::getState() { return m_state; }

// TODO: deduplicate this with the one in intake.cpp
template <class... Ts> struct overloads : Ts... {
    using Ts::operator()...;
};
template <class... Ts> overloads(Ts...) -> overloads<Ts...>;

/**
 * @brief Get the Filtered Color object
 *
 * @return std::optional<COLOR> Color that is currently going to the mogo.
 * @return std::nullopt If not intaking to mogo
 */
static std::optional<COLOR> getFilteredColor() {
  static auto visitor = overloads(
      [](Intake::FilteringState& state) -> std::optional<COLOR> {
        return state.color;
      },
      [](Intake::Filtering& state) -> std::optional<COLOR> {
        if (bot.getAlliance() == COLOR::RED) {
          if (state.red == Intake::MOGO) return COLOR::RED;
          if (state.blue == Intake::MOGO) return COLOR::BLUE;
        } else if (bot.getAlliance() == COLOR::BLUE) {
          if (state.blue == Intake::MOGO) return COLOR::BLUE;
          if (state.red == Intake::MOGO) return COLOR::RED;
        }
        return std::nullopt;
      },
      [](Intake::BaseState& state) -> std::optional<COLOR> {
        return std::nullopt;
      });
  auto state = bot.intake.getState();
  return std::visit(visitor, state);
}

void LEDs::update() {
  if (pros::competition::is_disabled()) {
    // If selecting auto, display alliance color
    displayFilterColor(bot.selector.getAlliance());
  } else if (pros::competition::is_autonomous()) {
    // If running auton, display
    switch (m_state) {
      case FILTER_COLOR:
        if (auto color = getFilteredColor()) {
          displayFilterColor(*color);
          break;
        };
        // Fall through
      case DISCONNECT:
        // Fall through (doesn't work rn)
      case OFF: makeStripsLight(0x000000); break;
      case MESSAGE:
        // If message is done, then set to filter color
        if (pros::millis() > time + startTime) {
          makeStripsLight(0x000000);
          if (pros::millis() > time + 100 + startTime) {
            LEDs::setState(FILTER_COLOR);
          }
        } else makeStripsLight(color);
        break;
    }
  } else if (!pros::competition::is_autonomous()) {
    // If running opcontrol, let opcontrol handle it
    if (pros::millis() % 100 < 10)
      printf("LEDs::update(): letting opcontrol handle it\n");
  }
}