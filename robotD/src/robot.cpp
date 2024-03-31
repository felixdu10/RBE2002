#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <event_timer.h>

// TODO: Choose your pins
const uint8_t LEFT_LINE_SENSOR_PIN = 20;
const uint8_t RIGHT_LINE_SENSOR_PIN = 22;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;
float Kp = 0.03;
float Kd = 0.15;
int previousError = 0;
float currentAngle = 0;

EventTimer lineTimer;
bool checkLineTimer(void)
{
  //return lineTimer.checkExpired();
  if (robotState == ROBOT_LINING && robotState == ROBOT_ONRAMP){
    return true;
  }
  return false;
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
    case NUM_1:
      robotState = ROBOT_LINING;
      handleLineTimer();
      Serial.println("Line Following");
      break;
    case NUM_2:
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

void readLineSensors() {
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR_PIN);
  Serial.print(leftSensorValue);
  Serial.print("  ");
  Serial.println(rightSensorValue);
  int error = rightSensorValue - leftSensorValue;
  int errorDer = error - previousError;
  previousError = error;
  int correction = error * Kp + errorDer * Kd;

  int leftSpeed = 15 - correction;
  int rightSpeed = 15 + correction;

  // Set motor speeds
  chassis.setMotorTargetSpeeds(leftSpeed, rightSpeed);
}

void handleLineTimer(void)
{
  if(robotState == ROBOT_LINING || robotState == ROBOT_ONRAMP)
  {
      // TODO: execute line following
      lineTimer.start(20);
      readLineSensors();
      if (robotState == ROBOT_ONRAMP) {
        Serial.println("ON RAMP");
      }
      // don't forget to restart the timer
      lineTimer.restart();
  }
}

bool checkIntersection(void)
{
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR_PIN);
  // Serial.print(leftSensorValue);
  // Serial.print(" ");
  // Serial.println(rightSensorValue);
  if (leftSensorValue < 50 && rightSensorValue < 50) {
    return true;
  }
  return false;
}

void handleIntersection(void)
{
  // robotState = ROBOT_IDLE;
  // chassis.setWheelTargetSpeeds(0,0);
  // delay(500);
  // robotState = ROBOT_DRIVING;
  // chassis.setWheelTargetSpeeds(11,11);
  // delay(3000);
  // robotState = ROBOT_LINING;
}

bool checkBatteryTimer(void)
{
  return false;
}

void handleBatteryTimer(void)
{
  
}

void handlePitchUpdate(float pitchAngle, float predictedAngle, float filteredAngle, float bias) {
  // Serial.print(predictedAngle);
  // Serial.print("  ");
  // Serial.print(pitchAngle);
  // Serial.print("  ");
  //Serial.println(filteredAngle);
  currentAngle = filteredAngle - bias;
}

bool checkIfOnSlope() {
  //Serial.println(currentAngle);
  if (currentAngle >= 4 || currentAngle <= -4) {
    return true;
  }
  return false;
}

void handleIfOnSlope() {
  digitalWrite(13, HIGH);
  robotState = ROBOT_ONRAMP;
  //Serial.println(angle);
}

bool checkIfFlat() {
  //Serial.println(angle);
  if(currentAngle < 4 && currentAngle > -4 && robotState == ROBOT_NEARTOP) {
    robotState = ROBOT_ONTOP;
    return true;
  }
  return false;
}

void handleIfFlat() {
  digitalWrite(13, LOW);
  // if (robotState == ROBOT_ONTOP) Serial.println("ON TOP");
  // chassis.setWheelTargetSpeeds(10,10);
  // delay(1800);
  // chassis.setWheelTargetSpeeds(0,0);
  // delay(500);
  // //robotState = ROBOT_ONRAMP;
  // //crossBridge();
}

bool checkIfCross(){
  if(currentAngle < 4 && currentAngle > -4 && robotState == ROBOT_ONTOP){

    return true;
  }
  return false;
}

void handleIfCross(){
  //Serial.println("CROSSING");
  chassis.setWheelTargetSpeeds(5,5);
}

void crossBridge() {
  chassis.setWheelTargetSpeeds(10,10);
  if (checkIfOnSlope()){
    robotState = ROBOT_LINING;
  }
}

bool checkIfOnTop() {
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR_PIN);
  // Serial.print(leftSensorValue);
  // Serial.print(" ");
  // Serial.println(rightSensorValue);
  if (leftSensorValue < 50 && rightSensorValue < 50 && robotState == ROBOT_ONRAMP) {
    robotState = ROBOT_NEARTOP;
    return true;
  }
  return false;
}

void handleIfOnTop() {
  //Serial.println("to TOP");
  chassis.setWheelTargetSpeeds(1,1);
  //robotState = ROBOT_ONRAMP;
}