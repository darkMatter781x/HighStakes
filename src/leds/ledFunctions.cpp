#include "leds/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "subsystems.h"

void AllianceColorLedDisplay(){

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
case preAuton: callFunctions(&AllianceColorLedDisplay);
case auton: callFunctions(&AutonWarningLedDisplay);
case driver: callFunctions(&IntakeFilterLedDisplay);
}

}