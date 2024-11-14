#pragma once

#include "colors.h"
#include "power.h"
#include "strip.h"
#include "subsystems.h"
#include "subsystems/intake.h"
#include <array>


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
void autonWarningDisplay();

void ledHandler();

std::shared_ptr<LedStrip> leftDrive; 
std::shared_ptr<LedStrip> rightDrive; 
std::shared_ptr<LedStrip> leftLift; 
std::shared_ptr<LedStrip> rightLift; 

int Motors[8] = {16, 15, 14, 13, 12, 11, 17, 9};
enum Destination{
    Idle,
    Filter,
    Goal,
    Lift
};
Destination destination;
enum FilterColor{
    RED,
    BLUE
};
FilterColor filterColor;

};