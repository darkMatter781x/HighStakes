#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "subsystems.h"
#include <optional>



std::string LED::getAllianceColor(){
  
}

int LED::getMotorDisconect(){
}

std::optional<Intake::DESTINATION> LED::getRingColorDestination(Intake::COLOR color){
  
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

}

void LED::ringFilterDisplay(){

}

void LED::ledHandler(){
  if (s_state != motorDisconnect){
    switch (m_state){
      case matchState::preAuton:
        allianceColorDisplay();
      case matchState::Auton:
        
      case matchState::Driver:
        ringFilterDisplay(); 
    };
  } else{
    motorDisconnectDisplay();
  }
}