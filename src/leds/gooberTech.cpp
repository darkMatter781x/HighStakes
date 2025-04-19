#include "color.h"
#include "leds/colors.h"
#include "leds/gooberTech.h"
#include "subsystems/intake.h"
#include "robot.h"

#define RED_HEX 0xFF000F
#define BLUE_HEX 0x00FFFF
#define WHITE_HEX 0x000000
#define Default State::FILTER_COLOR

LED::LED(std::shared_ptr<LedStrip> leftLift,
         std::shared_ptr<LedStrip> rightLift,
         std::shared_ptr<LedStrip> leftUnderGlow,
         std::shared_ptr<LedStrip> rightUnderGlow)
  : leftLift(leftLift), rightLift(rightLift), leftUnderGlow(leftUnderGlow),
    rightUnderGlow(rightUnderGlow) {}

void LED::skibidiLEDs(HexRGB hex) {
  if (pros::millis() % 50 > 25) {
    leftUnderGlow->buf().setAll(hex);
  } else {
    rightUnderGlow->buf().setAll(hex);
  }
}

void LED::setState(State state) { m_state = state; }

void LED::filterColor() { m_state = FILTER_COLOR; }

void LED::autonMessage(HexRGB color, int time, int startTime) {
  setState(AUTON_MESSAGE);
  this->time = time;
  this->startTime = startTime;
  this->color = color;
}

void LED::update() {
  switch (m_state) {
    case AUTON_MESSAGE:
      if (pros::millis() > time + 100 + startTime) {
        setState(Default);
      } else {
        skibidiLEDs(color);
      }
      break;

    case OFF: skibidiLEDs(0x0000F); break;

    case FILTER_COLOR:
      if (bot.intake.getDestination(ALLIANCE::RED) ==
          Intake::DESTINATION::MOGO) {
        skibidiLEDs(RED_HEX);
      } else if (bot.intake.getDestination(ALLIANCE::BLUE) ==
                 Intake::DESTINATION::MOGO) {
        skibidiLEDs(BLUE_HEX);
      } else {
        skibidiLEDs(WHITE_HEX);
      }
      break;

    case MOTOR_DISCONNECT:
      // Add your logic here
      break;
  }
}