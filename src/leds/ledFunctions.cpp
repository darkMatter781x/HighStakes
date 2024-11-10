#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "pros/misc.hpp"
#include "subsystems.h"
#include <optional>



std::string LED::getAllianceColor(){
  
}

int LED::getMotorDisconect(){
  
}

std::optional<Intake::DESTINATION> LED::getRingColorDestination(Intake::COLOR color){
  if (color == Intake::BLUE){

  }
}

void LED::motorDisconnectDisplay(){
  switch (getMotorDisconect()) {
    case 16:
    case 15:
    case 14:
    case 13:
    case 12:
    case 11:
    case 17:
    case 9:
  };
}

void LED::allianceColorDisplay(){

  leftDrive->buf().setAll(0xFF0000);
  leftDrive->buf().setAll(0x0000FF);
  leftDrive->buf().getBuffer().at(10) = 0x00FF00;
  leftDrive->buf().update();

}

void LED::ledHandler(){
  // pros::competition::is_autonomous();
  // pros::competition::is_disabled();
  // pros::competition::is_connected();
  if (s_state != motorDisconnect){
    if (pros::competition::is_autonomous()){

    } else if (pros::competition::is_disabled()){
      allianceColorDisplay();
    } else if (pros::competition::is_connected()){
      ringFilterDisplay();
    }
  } else{
    motorDisconnectDisplay();
  }
}