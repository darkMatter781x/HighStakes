#pragma once

#include "leds/colors.h"
#include "leds/strip.h"
#include "pros/rtos.hpp"
#include "subsystems.h"
#include <climits>
#include <memory>

class LED : Subsystem {
  private:
    enum State {
      /* Flashes LED during auton to give programmer feedback */
      AUTON_MESSAGE,
      /* Flashes LEDs red if motor disconnect */
      MOTOR_DISCONNECT,
      /* Lights up the Color of the ring color the robot is intaking */
      FILTER_COLOR,
      /* Turns off the LEDs*/
      OFF
    };

    std::shared_ptr<LedStrip> leftLift;
    std::shared_ptr<LedStrip> rightLift;
    std::shared_ptr<LedStrip> leftUnderGlow;
    std::shared_ptr<LedStrip> rightUnderGlow;

    HexRGB color;
    int time;
    int startTime = INT_MAX;

    State m_state = FILTER_COLOR;
  public:
    LED(std::shared_ptr<LedStrip> leftLift, std::shared_ptr<LedStrip> rightLift,
        std::shared_ptr<LedStrip> leftUnderGlow,
        std::shared_ptr<LedStrip> rightUnderGlow);

    void setState(State state);

    /* update function for the LEDs */
    void update() override;
    /* make both LED strips light up */
    void skibidiLEDs(HexRGB hex);
    /* Auton Message */
    void autonMessage(HexRGB color, int time, int startTime = pros::millis());
    /* Filter Color */
    void filterColor();
};