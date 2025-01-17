#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "config.h"

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "subsystems/intake.h"
#include <iterator>

void LEDs::displayFilterColor(COLOR color) {
  if (color == ALLIANCE::RED) {
    leftLift->buf().setAll(0xFF0000);
    rightLift->buf().setAll(0xFF0000);
    leftUnderGlow->buf().setAll(0xFF0000);
    rightUnderGlow->buf().setAll(0xFF0000);

  } else if (color == ALLIANCE::BLUE) {
    leftLift->buf().setAll(0x0000FF);
    rightLift->buf().setAll(0x0000FF);
    leftUnderGlow->buf().setAll(0x0000FF);
    rightUnderGlow->buf().setAll(0x0000FF);
  } else {
    leftLift->buf().setAll(0xFFFFFF);
    rightLift->buf().setAll(0xFFFFFF);
    leftUnderGlow->buf().setAll(0xFFFFFF);
    rightUnderGlow->buf().setAll(0xFFFFFF);
  }
}

void LEDs::displayDISCONNECT(int port) {}

void LEDs::setState(State state) { m_state = state; }

int LEDs::getMotorDisconect() {
  for (int i = 0; i < std::size(LEDs::motors); i++){
    pros::Device device(LEDs::motors[i]);
    if (!device.is_installed()){
      return motors[i];
    }
  }
  return 0;
}

LEDs::State LEDs::getState() { return m_state; }

void LEDs::update() {
  if (LEDs::getMotorDisconect() != 0){
    LEDs::setState(DISCONNECT);
  }
}