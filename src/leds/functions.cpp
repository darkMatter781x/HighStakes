#include "leds/functions.h"
#include "color.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "config.h"

#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "subsystems/intake.h"
#include <climits>
#include <iostream>
#include <concepts>
#include <iterator>
#include <string>

void LEDs::makeStripsLight(HexRGB hex){
  if (pros::millis()%100 > 50){
    leftUnderGlow->buf().setAll(hex);
  } else {
    rightUnderGlow->buf().setAll(hex);
  }
}

//int startTime = INT_MAX;
void LEDs::displayFilterColor(COLOR color) {
  if (color == ALLIANCE::RED) {
    makeStripsLight(0xFF0000);
  } else if (color == ALLIANCE::BLUE) {
    makeStripsLight(0x0000FF);
  } else {
    makeStripsLight(0xFFFFFF);
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

void LEDs::autonMessage(HexRGB color, int time,  int startTime){
  if (m_state == MESSAGE){
    if (pros::millis() < startTime + time){
      makeStripsLight(color);
    } else{
      makeStripsLight(0x000000);
    }
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
  if (pros::competition::is_autonomous()){
    LEDs::setState(MESSAGE);
  } else if(pros::competition::is_connected()){
    LEDs::setState(FILTER_COLOR);
  }
  
}