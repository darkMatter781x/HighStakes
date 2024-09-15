#pragma once

#include "colors.h"
#include "power.h"
#include "strip.h"
#include "subsystems/intake.h"
class IntakeFilterLedDisplay{
    

};

class MotorDisconnectLedDisplay{

};

class AllianceColor{

public:

enum allianceColor{ red, blue };
allianceColor m_state;
void AllianceColorLedDisplay();

};

class AutonWarningLedDisplay{

};

class LEDs : public Subsystem{

public: 

enum matchState{ preAuton, auton, driver };
matchState m_state;
void mainControl();

private:

pros::Optical& m_optical;
};