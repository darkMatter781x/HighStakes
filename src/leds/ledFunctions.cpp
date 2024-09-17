#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "pros/colors.h"
#include "subsystems.h"

void AllianceColor::AllianceColorLedDisplay(){
switch (m_state){
case red:
case blue:
}
}

void AutonWarningLedDisplay(){

}

void MotorDisconnectLedDisplay(){

}

void IntakeFilterLedDisplay(){

}

void callFunctions(std::function<void(void)> callback){
  callback();  
}

void LEDs::mainControl(){

switch (m_state) {
case preAuton: callFunctions(&AllianceColor::AllianceColorLedDisplay);
case auton: callFunctions(&AutonWarningLedDisplay);
case driver: callFunctions(&IntakeFilterLedDisplay);
}

}