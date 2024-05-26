#include <Arduino.h>
#include <robot.h>
#include <Sharp-IR.h>
#include <ir_codes.h>
#include <IRdecoder.h>
#include <Wire.h> // I2C library
#include <LSM6.h>

/**
 * IR remote decoder.
*/
#define IR_PIN 17 //TODO: Choose your pin
#define APRILTAG_DETECTION_PIN 11

IRDecoder decoder(IR_PIN);
SharpIR sharpir(A3);
LSM6 imu;

/**
 * HC-SR04
*/
#include <HC-SR04.h>
HC_SR04 hc_sr04(0, 12); // TODO: Choose your pins
void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
}

void setup() 
{
    Serial.begin(115200);
    delay(500);
    Serial.println("setup()");

    Serial1.begin(115200);

    Serial.println("/setup()");

    chassis.init();

    decoder.init();
    imu.init();
    imu.setAccDataOutputRate(0x4);
    imu.setGyroDataOutputRate(0x4);
    imu.setFullScaleAcc(imu.ACC_FS::ACC_FS8);
    imu.setFullScaleGyro(imu.GYRO_FS::GYRO_FS500);
    sharpir.init();
    hc_sr04.init(ISR_HC_SR04);

    Serial.println("/setup()");
    pinMode(13, OUTPUT);
    pinMode(A2, INPUT);
    pinMode(A0, INPUT);
    pinMode(A4, INPUT);
    pinMode(A3, INPUT);

    pinMode(APRILTAG_DETECTION_PIN, INPUT);
     


}

void loop() 
{
    /**
     * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
     * any number of processes that we want to run on the same schedule.
    */
    if(chassis.loop())
    {
    }

    /**
     * But we can also process asynchronous events, such as IR remote presses or distance sensor readings.
    */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    /** Check the distance sensor.
     * We return true only if there is a new reading, which is passed by reference.
     * 
     * Note that the construct is still checker/handler: if there is a new reading, handle it
     */
    float distanceReading = 0;
    bool hasNewReading = sharpir.getDistance(distanceReading);
    if(hasNewReading) handleNewDistanceReading(distanceReading);

    /**
     * Line sensors. We do two things here:
     *  - handle the line timer, which schedule line following updates
     *  - check for intersections
     * 
     * We could combine the two, but it's cleaner this way with the checker/handlers.
     * An extra couple ADC reads is hardly a problem.
    */
    if(checkLineTimer()) handleLineTimer();
    if (checkInputTurn()) handleInputTurnEast();
    if(checkIntersection()) handleIntersection();
    if(checkLineCrossed()) handleLineCrossed();
    if(checkWall()) handleWallLeft();
    if(checkIfTurned()) handleIfTurnedLeft();
    if(checkBatteryTimer()) handleBatteryTimer();
    float pitchAngle = 0;
    float predictedAngle = 0;
    float filteredAngle = 0;
    if(imu.checkForPitchUpdate(pitchAngle, predictedAngle, filteredAngle)) handlePitchUpdate(pitchAngle, predictedAngle, filteredAngle, imu.gyroBiasY);
    if (checkIfOnSlope()) handleIfOnSlope();
    if (checkForBalance()) handleBalance();
    if (checkMsg()) handleMsg();
    if (checkIfFlat()) handleIfFlat();
}