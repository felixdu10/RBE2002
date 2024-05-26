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
    ROBOT_FOUND_WALL,
    ROBOT_INTERSECTION,
    ROBOT_TURN,
    ROBOT_INCREMENTED,
    ROBOT_ONRAMP,
    ROBOT_ONTOP,
    ROBOT_NEARTOP,
    ROBOT_TAG,
    ROBOT_INPUT_EAST,
    ROBOT_INPUT_WEST,
    ROBOT_WAITING
};

enum HEADING
{
    ROBOT_NORTH,
    ROBOT_SOUTH,
    ROBOT_EAST,
    ROBOT_WEST,
};

void handleKeyCode(int16_t keyCode);

void handleNewDistanceReading(float distanceReading);

bool checkLineTimer(void);
void handleLineTimer(void);

bool checkIntersection(void);
void handleIntersection(void);

bool checkBatteryTimer(void);
void handleBatteryTimer(void);

bool checkWall();
void handleWallLeft();
void handleWallRight();
bool checkIfTurned();
void handleIfTurnedLeft();
void handleIfTurnedRight();
void handlePitchUpdate(float pitchAngle, float predictedAngle, float filteredAngle, float bias);
bool checkIfOnSlope();
void handleIfOnSlope();
void handleIfFlat();
bool checkIfFlat();
bool checkLineCrossed();
void handleLineCrossed();
bool checkForBalance();
void handleBalance();
bool checkMsg();
void handleMsg();
void turnEast();
void turnWest();
bool checkInputTurn();
void handleInputTurnEast();
void handleInputTurnWest();
