#include "leds/functions/functions.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "config.h"

#include "pros/misc.hpp"
#include "subsystems/intake.h"
 

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

void LEDs::displayDISCONNECT(int port){

}

void LEDs::setState(State state){
    m_state = state;
}

int LEDs::getMotorDisconect(){
    
}

void LEDs::update(){
    if (pros::competition::is_autonomous()){
        
    } 
    if (pros::competition::is_field_control()){
        displayFilterColor(filterColorLED);
    }
}