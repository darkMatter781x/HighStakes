#pragma once

#include "colors.h"
#include "power.h"
#include "strip.h"
#include "subsystems.h"
#include "subsystems/intake.h"

class LED : Subsystem{

public:

enum  matchState{ preAuton, Auton, Driver };
matchState m_state;

enum ledState{ motorDisconnect, allianceColor, ringFiltering };
ledState s_state;

//get information functions
std::string getAllianceColor();
int getMotorDisconect();
std::optional<Intake::DESTINATION> getRingColorDestination(Intake::COLOR color);

void motorDisconnectDisplay();
void allianceColorDisplay();
void ringFilterDisplay();

void ledHandler();

};