#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <event_timer.h>

// TODO: Choose your pins
const uint8_t LEFT_LINE_SENSOR_PIN = A0;
const uint8_t RIGHT_LINE_SENSOR_PIN = A2;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;

EventTimer lineTimer;
bool checkLineTimer(void)
{
  return lineTimer.checkExpired();
}

void idle(void)
{
    Serial.println("Idling!");
    chassis.setMotorEfforts(0, 0);
    robotState = ROBOT_IDLE;
}

String keyCodeString; // this may come in handy later
void handleKeyCode(int16_t keyCode)
{ 
    Serial.println(keyCode);

  if(keyCode == ENTER_SAVE) idle();

  switch(keyCode)
  {
    case UP_ARROW:
      chassis.setMotorTargetSpeeds(10, 10);
      robotState = ROBOT_DRIVING;
      break;
    case RIGHT_ARROW:
      chassis.setMotorTargetSpeeds(10, -10);
      robotState = ROBOT_RIGHT;
      break;
    case DOWN_ARROW:
      chassis.setMotorTargetSpeeds(-10, -10);
      robotState = ROBOT_DRIVING;
      break;
    case LEFT_ARROW:
      chassis.setMotorTargetSpeeds(-10, 10);
      robotState = ROBOT_LEFT;
      break;
    default:
      break;
  }
}

void handleNewDistanceReading(float distanceReading)
{
#ifdef __DEBUG_RANGEFINDER__
    Serial.println(distanceReading);
#endif
    if(robotState == ROBOT_STANDOFF)
    {
      /**
       * TODO: Add control
      */
    }
    else if(robotState == ROBOT_WALLING)
    {
      /**
       * TODO: Add control
      */
    }
}

void handleLineTimer(void)
{
  if(robotState == ROBOT_LINING)
  {
      // TODO: execute line following

      // don't forget to restart the timer
      lineTimer.restart();
  }
}

bool checkIntersection(void)
{
  // TODO: check for an intersection
  return false;
}

void handleIntersection(void)
{
  // TODO: handle intersection
}

bool checkBatteryTimer(void)
{
  return false;
}

void handleBatteryTimer(void)
{
  
}