#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "config.h"

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "subsystems/intake.h"
#include <iostream>
#include <concepts>
#include <iterator>
#include <string>

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

void LEDs::displayDISCONNECT(int port) {
  if (getMotorDisconnect() == 12 || getMotorDisconnect() == 14 || getMotorDisconnect() == 17){
    printf("l drive\n");
    leftUnderGlow->buf().setAll(0xFF0000);
    rightUnderGlow->buf().setAll(0x000000);
    leftLift->buf().setAll(0x000000);
    rightLift->buf().setAll(0x000000);
  } else if ( getMotorDisconnect() == 13 || getMotorDisconnect() == 15 || getMotorDisconnect() == 16){
    printf("r drive\n");
    leftUnderGlow->buf().setAll(0x000000);
    rightUnderGlow->buf().setAll(0xFF0000);
    leftLift->buf().setAll(0x000000);
    rightLift->buf().setAll(0x000000);    
  } else if ( getMotorDisconnect() == 11){
    printf("intake \n");
    leftUnderGlow->buf().setAll(0x000000);
    rightUnderGlow->buf().setAll(0x000000);
    leftLift->buf().setAll(0xFF0000);
    rightLift->buf().setAll(0x000000); 
  } else {
    printf("lb\n");
    leftUnderGlow->buf().setAll(0x000000);
    rightUnderGlow->buf().setAll(0x000000);
    leftLift->buf().setAll(0xFF0000);
    rightLift->buf().setAll(0xFF0000);
  }
}

void LEDs::setState(State state) { m_state = state; }

int LEDs::getMotorDisconnect() {
  for (int i = 0; i < std::size(LEDs::motors); i++){
    pros::Device device(LEDs::motors[i]);
    if (device.is_installed() == false){
      printf("not nerd\n");
      return motors[i];
    }
    printf("nerd\n");
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

void LEDs::update() {
  /*printf("yey\n");
  if (LEDs::getMotorDisconnect() != 0){
    printf("disconnect\n");
    LEDs::setState(DISCONNECT);
  } else if (pros::competition::is_autonomous()){
    printf("auton\n");
    LEDs::setState(MESSAGE);
  } else if (pros::competition::is_connected()){
    printf("filter color\n");
    LEDs::setState(FILTER_COLOR);
  } */
  
  LEDs::setState(FILTER_COLOR);
  
}