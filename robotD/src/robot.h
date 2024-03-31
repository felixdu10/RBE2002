#pragma once

#include <Arduino.h>
#include <Chassis.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_DRIVING,
    ROBOT_STANDOFF, 
    ROBOT_LINING,
    ROBOT_WALLING,
    ROBOT_ONRAMP,
    ROBOT_ONTOP,
    ROBOT_NEARTOP,
};

void handleKeyCode(int16_t keyCode);

void handleNewDistanceReading(float distanceReading);

bool checkLineTimer(void);
void handleLineTimer(void);

bool checkIntersection(void);
void handleIntersection(void);

bool checkBatteryTimer(void);
void handleBatteryTimer(void);
void handlePitchUpdate(float pitchAngle, float predictedAngle, float filteredAngle, float bias);
bool checkIfOnSlope();
void handleIfOnSlope();
void handleIfFlat();
bool checkIfFlat();
void crossBridge();
bool checkIfOnTop();
void handleIfOnTop();
bool checkIfCross();
void handleIfCross();