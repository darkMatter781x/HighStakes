#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "pros/device.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "subsystems.h"
#include <optional>




std::string LED::getAllianceColor(){
  std::string allianceColor;
  
  return allianceColor;
}

int LED::getMotorDisconect(){
  for (int i = 0; i < 8, i++){
    pros::Device device(Motors[i]);
    if (device.is_installed() != true){
      return Motors[i];
    }
  }
}

std::optional<Intake::DESTINATION> LED::getRingColorDestination(Intake::COLOR color){

}

void LED::motorDisconnectDisplay(){
switch (getMotorDisconect()){
  case 16:
    pros::delay(500);
    leftDrive->buf().setAll(0xFF0000);
  case 15:
    pros::delay(500);
    leftDrive->buf().setAll(0xFF0000);
  case 14:
    pros::delay(500);
    leftDrive->buf().setAll(0xFF0000);
  case 13:
    pros::delay(500);
    rightDrive->buf().setAll(0xFF0000);
  case 12:
    pros::delay(500);
    rightDrive->buf().setAll(0xFF0000);
  case 11:
    pros::delay(500);
    rightDrive->buf().setAll(0xFF0000);
  case 17:
    pros::delay(500);
    leftDrive->buf().setAll(0xFF0000);
    rightDrive->buf().setAll(0xFF0000);
  case 9:
    pros::delay(500);
    leftDrive->buf().setAll(0xFF0000);
    rightDrive->buf().setAll(0xFF0000);  
}  
}

void LED::allianceColorDisplay(){
  if (getAllianceColor() == "RED"){
    leftDrive->buf().setAll(0xFF0000);
    rightDrive->buf().setAll(0xFF0000);
    leftLift->buf().setAll(0xFF0000);
    rightLift->buf().setAll(0xFF0000);
  } else if (getAllianceColor() == "BLUE"){
    leftDrive->buf().setAll(0x0000FF);
    rightDrive->buf().setAll(0x0000FF);
    leftLift->buf().setAll(0x0000FF);
    rightLift->buf().setAll(0x0000FF);
  }

  //leftDrive->buf().setAll(0xFF0000);
  // leftDrive->buf().setAll(0x0000FF);
  //leftDrive->buf().getBuffer().at(10) = 0x00FF00;
  //leftDrive->buf().update();

}

void LED::autonWarningDisplay(){
  
}

void LED::ringFilterDisplay(){
  if (filterColor == RED && destination == Goal){

  } else if (filterColor == BLUE && destination == Goal){

  } else if (filterColor == RED && destination == Idle){

  } else if (filterColor == BLUE && destination == Idle){

  } else if (filterColor == RED && destination == Lift){

  } else if (filterColor == BLUE && destination == Lift){

  } else if (filterColor == RED && destination == Filter){

  } else if (filterColor == BLUE && destination == Filter){

  }
}

void LED::ledHandler(){
  if (s_state == motorDisconnect){
    if (pros::competition::is_autonomous()){

    } else if (pros::competition::is_disabled()){
      allianceColorDisplay();
    } else if (pros::competition::is_connected()){
      ringFilterDisplay();
    }
  } else{
    leftDrive->buf().setAll(0x000000);
    rightDrive->buf().setAll(0x000000);
    motorDisconnectDisplay();
  }
}