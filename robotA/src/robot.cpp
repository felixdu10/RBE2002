#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <event_timer.h>
#include <openmv.h>
#include <string.h>

using namespace std;


// TODO: Choose your pins
const uint8_t LEFT_LINE_SENSOR_PIN = A0;
const uint8_t RIGHT_LINE_SENSOR_PIN = A4;

ROBOT_STATE robotState = ROBOT_IDLE;
HEADING heading;
Chassis chassis;
OpenMV camera;
bool foundTag = false;
float Kp = 0.01;
float Kd = 0.07;
int previousError = 0;
int i = 0;
int j = 0;
int lasti = 0;
int lastj = 0;
bool hasTurned = false;
float dist = 0.0;
bool incremented = false;
unsigned long reachedIntersection = 0; // Timestamp of the last intersection
const long crossTime = 250; // Cooldown period in milliseconds
float currentAngle = 0;
bool ramping = false;
bool inPosition = false;



void sendMessage(const String& topic, const String& message)
{
    Serial1.println(topic + String(':') + message);
}
String serString1;
bool checkSerial1(void)
{
    while(Serial1.available())
    {
        char c = Serial1.read();
        serString1 += c;

        if(c == '\n')
        {
            return true;
        }
    }

    return false;
}

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
    case NUM_2:
      robotState = ROBOT_LINING;
      heading = ROBOT_NORTH;
      handleLineTimer();
      break;
    case NUM_4:
      turnWest();
      break;
    case NUM_6:
      turnEast();
      break;
    case NUM_8:
      robotState = ROBOT_LINING;
      heading = ROBOT_SOUTH;
      handleLineTimer();
      break;
    default:
      break;
  }
}

void handleNewDistanceReading(float distanceReading)
{
#ifdef __DEBUG_RANGEFINDER__
    //Serial.println(distanceReading);
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
    dist = distanceReading;
    //Serial.println(dist);
}


void readLineSensors() {
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR_PIN);
  // Serial.print(leftSensorValue);
  // Serial.print("  ");
  // Serial.println(rightSensorValue);
  int error = rightSensorValue - leftSensorValue;
  int errorDer = error - previousError;
  previousError = error;
  int correction = error * Kp + errorDer * Kd;

  int leftSpeed = 10 + correction;
  int rightSpeed = 10 - correction;

  // Set motor speeds
  chassis.setMotorTargetSpeeds(leftSpeed, rightSpeed);
}

void handleLineTimer(void)
{
  if((robotState == ROBOT_ONRAMP || robotState == ROBOT_LINING) && (robotState != ROBOT_FOUND_WALL && robotState != ROBOT_INTERSECTION))
  {
      lasti = i;
      lastj = j;   
      // Serial.print(x);
      // Serial.print(", ");
      // Serial.print(y);
      // Serial.print("    ");
      // Serial.println(robotState);
      //Serial.println(dist);
      lineTimer.start(20);
      readLineSensors();
      incremented = false;
      // Serial.print(chassis.xPose);
      // Serial.print("  ");
      // Serial.println(chassis.yPose);
      // don't forget to restart the timer
      lineTimer.restart();
  }
}

bool checkIntersection(void)
{
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR_PIN);
  if (leftSensorValue < 400 && rightSensorValue < 400 && robotState != ROBOT_ONRAMP && robotState != ROBOT_TAG) {
    robotState = ROBOT_INTERSECTION;
    return true;
  }
  return false;
}


void handleIntersection(void)
{
  if (robotState == ROBOT_INTERSECTION && !incremented){
    reachedIntersection = millis();
    switch(heading) {
      case ROBOT_NORTH:
        j++;
        break;
      case ROBOT_SOUTH:
        j--;
        break;
      case ROBOT_EAST:
        i++;
        break;
      case ROBOT_WEST:
        i--;
        break;
    }
    incremented = true;
    String topic1("1/x");
    String topic2("1/y");
    String topic3("1/pose/x");
    String topic4("1/pose/y");
    String topic5("1/pose/t");
    // String topic3("h");
    String message1(i); 
    String message2(j);
    String message3(chassis.xPose);
    String message4(chassis.yPose); 
    String message5(chassis.thetaStar);   
    // String message3(heading);
    // sendMessage(topic1, message1);
    // sendMessage(topic2, message2);
    // sendMessage(topic3, message3);
    // sendMessage(topic4, message4);
    // sendMessage(topic5, message5);
  }
  if (millis() - reachedIntersection > crossTime) {
    chassis.setWheelTargetSpeeds(4,4);
  }
  else {
    chassis.setWheelTargetSpeeds(5,5);
  }
}

bool checkBatteryTimer(void)
{
  return false;
}

void handleBatteryTimer(void)
{
  
}

bool checkLineCrossed() {
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR_PIN);
  if (!(leftSensorValue < 400 && rightSensorValue < 400) && (millis() - reachedIntersection > crossTime*2)) {
    return true;
  }
  return false;
}

void handleLineCrossed() {
  if (robotState == ROBOT_INTERSECTION) {
    robotState = ROBOT_LINING;
    lineTimer.restart();
  }
  
}

bool checkWall() {
  dist = round(dist);
  if (dist <= 12 && dist > 0 && robotState != ROBOT_INPUT_EAST && robotState != ROBOT_INPUT_WEST) {
    robotState = ROBOT_FOUND_WALL;
    hasTurned = false;
    return true;
  }
  return false;
}

void handleWallLeft() {
  chassis.setWheelTargetSpeeds(0,0);
  if (!hasTurned) {
    chassis.setWheelTargetSpeeds(-5,5); // turn left
  }
}

void handleWallRight() {
  chassis.setWheelTargetSpeeds(0,0);
  if (!hasTurned) {
    chassis.setWheelTargetSpeeds(5,-5); // turn right
  }
}

/**
 * checks for new line after turning
*/
bool checkIfTurned() {
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  if (leftSensorValue < 400 && robotState == ROBOT_FOUND_WALL) {
    hasTurned = true;
    return true;
  }
  return false;
}

/**
 * changes heading depending on previous heading when turning left, starts line following again
*/
void handleIfTurnedLeft() {
  switch(heading) {
    case ROBOT_NORTH:
      heading = ROBOT_WEST;
      break; 
    case ROBOT_EAST:
      heading = ROBOT_NORTH;
      break;
    case ROBOT_SOUTH:
      heading = ROBOT_EAST;
      break;       
    case ROBOT_WEST:
      heading = ROBOT_SOUTH;
      break;
  }
  robotState = ROBOT_LINING;
  lineTimer.restart();
}

/**
 * changes heading depending on previous heading when turning right, starts line following again
*/
void handleIfTurnedRight() {
  switch(heading) {
    case ROBOT_NORTH:
      heading = ROBOT_EAST;
      break; 
    case ROBOT_EAST:
      heading = ROBOT_SOUTH;
      break;
    case ROBOT_SOUTH:
      heading = ROBOT_WEST;
      break;       
    case ROBOT_WEST:
      heading = ROBOT_NORTH;
      break;
  }
  robotState = ROBOT_LINING;
  lineTimer.restart();
}

/**
 * updates angle variable
*/
void handlePitchUpdate(float pitchAngle, float predictedAngle, float filteredAngle, float bias) {
  currentAngle = filteredAngle;
}

/**
 * if robot angle is >10 degrees up or down
*/
bool checkIfOnSlope() {
  //Serial.println(currentAngle);
  if (currentAngle >= 10 || currentAngle <= -10) {
    return true;
  }
  return false;
}

/**
 * turns yellow LED on, resets xPose once for travel calculation
*/
void handleIfOnSlope() {
  digitalWrite(13, HIGH);
  robotState = ROBOT_ONRAMP;
  if (!ramping) {
    chassis.xPose = 0;
    ramping = true;
  }
}

/**
 * checks robot pose to determine how far up the ramp the bot has left to travel in order to not tip the ramp, returns true when reached position
*/
bool checkForBalance() {
  if (chassis.xPose >= 63) {
    return true;
  }
  return false;
}

/**
 * stops once in position on ramp
*/
void handleBalance() {
  if (robotState == ROBOT_ONRAMP) {
    chassis.setWheelTargetSpeeds(0,0);
    robotState = ROBOT_WAITING;
  }
}

/**
 * looks for april tags, sends tag information to MQTT broker
*/
uint8_t FindAprilTags(void)
{
  uint8_t tagCount = camera.getTagCount();
  if(tagCount)
  {
    Serial.println(tagCount);
    AprilTagDatum tag;
    if(camera.readTag(tag) && !foundTag)
    {
      // Serial.print(F("Tag [cx="));
      // Serial.print(tag.cx);
      // Serial.print(F(", cy="));
      // Serial.print(tag.cy);
      // Serial.print(F(", w="));
      // Serial.print(tag.w);
      // Serial.print(F(", h="));
      // Serial.print(tag.h);
      // Serial.print(F(", id="));
      // Serial.print(tag.id);
      // Serial.print(F(", rot="));
      // Serial.print(tag.rot);
      // Serial.println(F("]"));
      String topic1("id");
      String message1(tag.id);
      
      String topic2("rot");
      String message2(tag.rot);

      sendMessage(topic1, message1);
      sendMessage(topic2, message2);
      sendMessage(topic1, message1);
      sendMessage(topic2, message2);
  
      foundTag = true;
    }
  }
  return tagCount;
}


/**
 * checks the MQTT Broker every second for new messages, returns true if one found, false otherwise
*/
bool checkMsg() {
  static uint32_t lastSend = 0;
  uint32_t currTime = millis();
  if(currTime - lastSend >= 1000)
  {
    lastSend = currTime;
    Serial.println("looking for msg");
    return checkSerial1();
  }
  return false;
}

/**
 * checks for specific message. if found, continues
*/
void handleMsg() {
  Serial.println(serString1);
  if (robotState == ROBOT_WAITING && (serString1.substring(serString1.indexOf(":") + 1,serString1.indexOf(":") + 6)).equals("READY")) {
    Serial.println("msg received");
    chassis.xPose = 0;
    robotState = ROBOT_ONRAMP;
    lineTimer.restart();
  }
  serString1 = "";
}

/**
 * checks if robot is flat on ground
*/
bool checkIfFlat() {
  if(currentAngle < 6 && currentAngle > -6) {
    return true;
  }
  return false;
}

/**
 * once flat, stops and starts looking for april tag
*/
void handleIfFlat() {
  if (robotState == ROBOT_ONRAMP) {
    digitalWrite(13, LOW);
    chassis.setWheelTargetSpeeds(0,0);
    Serial.println("looking for tag");
    static uint32_t lastOpenMVread = 0;
    if(millis() - lastOpenMVread > 1)
    {
      FindAprilTags();
      lastOpenMVread = millis();
    }
  }
}

void turnEast() {
  robotState = ROBOT_INPUT_EAST;
  if (!hasTurned) {
    chassis.setWheelTargetSpeeds(5,-5);
  }
}

void turnWest() {
  robotState = ROBOT_INPUT_WEST;
  if (!hasTurned) {
    chassis.setWheelTargetSpeeds(-5,5);
  }
}


bool checkInputTurn() {
  int leftSensorValue = analogRead(LEFT_LINE_SENSOR_PIN);
  if (leftSensorValue < 400) {
    return true;
  }
  return false;
}

void handleInputTurnEast() {
  if (robotState == ROBOT_INPUT_EAST) {
    chassis.setWheelTargetSpeeds(0,0);
    hasTurned = true;
    heading = ROBOT_EAST;
    robotState = ROBOT_LINING;
    lineTimer.restart();
  }
}

void handleInputTurnWest() {
  if (robotState == ROBOT_INPUT_WEST) {
    chassis.setWheelTargetSpeeds(0,0);
    hasTurned = true;
    heading = ROBOT_WEST;
    robotState = ROBOT_LINING;
    lineTimer.restart();
  }
}