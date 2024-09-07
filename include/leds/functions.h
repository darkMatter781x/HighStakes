#pragma once

#include "colors.h"
#include "power.h"
#include "strip.h"
#include "subsystems/intake.h"

class LedMaster {
    public:
    /** this function doesn't have a definition */
    virtual void control() =0;
};

class IntakeFilterLedDisplay : LedMaster {
    void control() override;

};

class MotorDisconnectLedDisplay : LedMaster{

};

class AllianceColorLedDisplay : LedMaster{

};

class AutonWarningLedDisplay : LedMaster{

};

class LEDs : public Subsystem{

public: 
enum matchState{ preAuton, auton, driver };
matchState m_state;

void setMaster(LedMaster& callback) {
    ledController = callback;
}
void mainControl();
private:
LedMaster& ledController;
pros::Optical& m_optical;
};