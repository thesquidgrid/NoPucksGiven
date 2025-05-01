#include <Arduino.h>
#include "MC33926MotorShield.h"
#include <Wire.h>
#include <VL53L0X.h>

// Define sensor pins and value array
#define SENSOR_NUMS 7
uint8_t sensorPins[SENSOR_NUMS] = {
    16,
    15,
    14,
    41,
    40,
    39,
    38
};
uint16_t sensorValues[SENSOR_NUMS];

// PID Constants
#define SETPOINT 3500
#define MAX_SPEED 200
#define MIN_SPEED 120
#define SET_SPEED 150
#define KP .4
#define KI 0
#define KD 1


// Define stepper motor pins
#define STEP_PIN 4
#define DIRECTION_PIN 5
#define ENABLE_PIN 3

// Define steps for 90-degree rotation
#define TURN_90 50

// Motors Setup
MC33926MotorShield leftMotor(12, 10, 11, 24, 26);
MC33926MotorShield rightMotor(9, 7, 8, 25, 26);

// Other Variables
float integralError = 0;
int lastError = 0;

void setup() {
    Serial.begin(115200);

    Wire.begin();

    sensor.setTimeout(500);
    if (!sensor.init())
    {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
    }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();

    // Set sensor pins as inputs
    for (int i = 0; i < SENSOR_NUMS; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    leftMotor.begin();
    rightMotor.begin();
    Serial.println("Setup ran and completed");
    '

    // Enable Stepper Driver
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Enable motor driver

    // Stepper Configuration
    stepper.setMaxSpeed(500); // Steps per second
    stepper.setAcceleration(300); // Acceleration for smoother movement
}

void loop() {
    leftMotor.setSpeed(120);
    rightMotor.setSpeed(130);
    delay(2000);
    rightMotor.setSpeed(0);
    leftMotor.setSpeed(0);
    delay(1000);

    //poll
    while (true) {
        lineFollow(); // Fixed function 
    }

}

// Function for reading line sensors
void readData() {
    for (int i = 0; i < SENSOR_NUMS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
    }
}

// Error Calculation (Simplified)
int calculateError() {
    int leftSum = 0;
    int rightSum = 0;

    for (int i = 0; i < SENSOR_NUMS / 2; i++) {
        leftSum += sensorValues[i]; // Sum left side sensors
        rightSum += sensorValues[SENSOR_NUMS - 1 - i]; // Sum right side sensors
    }

    return rightSum - leftSum; // Positive = too far right, Negative = too far left
}

// PID Function for Line Following
void lineFollow() {
    readData(); // Fixed function name
    int error = calculateError(); // Compute error from left/right difference

    // PID calculations
    int proportional = error;
    integralError += error;
    int derivative = error - lastError;
    int correction = (KP * proportional) + (KI * integralError) + (KD * derivative);
    lastError = error; // Store last error

    // Adjust motor speeds based on correction
    int leftSpeed = SET_SPEED - correction; // Fixed BASE_SPEED -> SET_SPEED
    int rightSpeed = SET_SPEED + correction; // Fixed BASE_SPEED -> SET_SPEED

    // Constrain speeds to valid range
    if ((error > -10) & (error < 10)) {
        if (error < 0) { // go left
            leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
            rightSpeed = 0;

        } else { //go right 
            leftSpeed = 0;
            rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

        }
    } else {
        leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    }

    // Set motor speeds
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);

    Serial.print("Error: ");
    Serial.println(error); // Debugging output
}

// Function to Rotate 90 Degrees
void rotate90() {
    stepper.move(TURN_90); // Move exactly 50 steps
    stepper.runToPosition(); // Ensures it reaches the exact position
}