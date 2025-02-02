#pragma once

#include "color.h"
#include "leds/colors.h"
#include "leds/power.h"
#include "leds/strip.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "config.h"
#include "subsystems.h"
#include "subsystems/intake.h"
#include <string>

class LEDs : Subsystem {
  private:
  public:
    std::shared_ptr<LedStrip> leftLift;
    std::shared_ptr<LedStrip> rightLift;
    std::shared_ptr<LedStrip> leftUnderGlow;
    std::shared_ptr<LedStrip> rightUnderGlow;

    enum State {
      /* Flashes LED during auton to give programmer feedback */
      MESSAGE,
      /* Flashes LEDs red if motor disconnect */
      DISCONNECT,
      /* Lights up the Color of the ring color the robot is intaking */
      FILTER_COLOR,
      /* Turns off the LEDs*/
      OFF
    };

    int motors[8] = {12, 14, 17, 13, 15, 16, 11, 18};

    State m_state;
    COLOR filterColorLED;

    void update() override;

    LEDs(std::shared_ptr<LedStrip> leftLift,
         std::shared_ptr<LedStrip> rightLift,
         std::shared_ptr<LedStrip> leftUnderGlow,
         std::shared_ptr<LedStrip> rightUnderGlow);

    /* displays the current color that the intake is not filtering */
    void displayFilterColor(COLOR color);
    //void rightDisplayFilterColor(COLOR color);
    /* displays motor disconnect */
    void displayDISCONNECT(int port);
    /* displays autonomus messages */
    void autonMessage(HexRGB color, int time, int startTime);

    void setState(State state);
    State getState();

    void setToMESSAGE();
    void setToDisconnect();
    void setToFilter_Color();

    void makeStripsLight(HexRGB hex);

    int getMotorDisconnect();
};
