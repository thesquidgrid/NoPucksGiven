#include <Arduino.h>
#include <AccelStepper.h>

// Define stepper motor pins
#define STEP_PIN 4
#define DIRECTION_PIN 5
#define ENABLE_PIN 3

// Define steps for 90-degree rotation
#define TURN_90 50

// Initialize stepper in DRIVER mode
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);

void setup() {
    // Enable Stepper Driver
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);  // Enable motor driver
      
    // Stepper Configuration
    stepper.setMaxSpeed(500);      // Steps per second
    stepper.setAcceleration(300);  // Acceleration for smoother movement
}

void loop() {
  // Rotate 90 degrees then wait before doing it again until forever
    rotate90();
    delay(3000);
}

// Function to Rotate 90 Degrees
void rotate90() {  
    stepper.move(TURN_90);  // Move exactly 50 steps
    stepper.runToPosition(); // Ensures it reaches the exact position
}
