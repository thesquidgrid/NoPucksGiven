#include <Arduino.h>

#include <AccelStepper.h>

#include "Adafruit_VL53L0X.h"
#include <Wire.h>  // Include the Wire library for custom I2C pins

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


// Define stepper motor pins
#define STEP_PIN 4
#define DIRECTION_PIN 5
#define ENABLE_PIN 3
#define SLEEP_PIN 6 // <-- Added sleep pin

// Define steps for 90-degree rotation
#define TURN_90 50

// Shooter driver pins
#define IN1 21
#define IN2 20
#define PWMA 37

//pin for controlling TOF
const int XSHUT_PIN = 17;  // Pin for controlling the XSHUT pin (D17)


int counter = 0;
// Initialize stepper in DRIVER mode
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);

void setup() {
  // Enable Stepper Driver
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable motor driver

  // Sleep pin setup
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH); // Wake up the stepper driver

  // Stepper Configuration
  stepper.setMaxSpeed(500); // Steps per second
  stepper.setAcceleration(300); // Acceleration for smoother movement

  // tells teensy to output to these pins and not read from them
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  //TOF sensor setup
  // wait until serial port opens for native USB devices
  while (!Serial) delay(10);

  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, HIGH);  // Bring the sensor out of reset
  delay(100);                      // Wait for the sensor to boot
  Wire.begin();

    

  while(!lox.begin(VL53L0X_I2C_ADDR, 0, &Wire)) {
    Serial.println(F("Failed to boot VL53L0X"));
    
  }
  
  lox.startRangeContinuous();
}

void loop() {
  uint16_t range = lox.readRange();
  
    Serial.print("Distance (mm): ");
    Serial.println(range);
  
  delay(50);

  if(range > 200){

  // Wake driver and rotate
  digitalWrite(SLEEP_PIN, HIGH);
  rotate90();
  delay(3000);

  Serial.println("turning");
  
  // Optional: put stepper driver to sleep to save power
  digitalWrite(SLEEP_PIN, LOW);
  delay(3000);
  counter++;
  Serial.println(counter);
  if (counter == 3) {
    analogWrite(PWMA, 50);
    delay(1000);
    analogWrite(PWMA, 300);
    delay(1000);
    analogWrite(PWMA, 0);
    while (1);
  }
  }

}

// Function to Rotate 90 Degrees
void rotate90() {
  stepper.move(TURN_90); // Move exactly 50 steps
  stepper.runToPosition(); // Ensures it reaches the exact position
}

void shoot() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWMA, 1000); // max power

  // Stop motor 

}

void turnOff() {
  analogWrite(PWMA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

//
//// Ramp up shooter motor power with exponential curve
//void exponentialShooterRampUp(int maxPWM, int durationMs) {
//  unsigned long startTime = millis();
//  float progress = 0.0;
//
//  Serial.println("Starting exponential shooter ramp-up...");
//
//  while (progress < 1.0) {
//    unsigned long currentTime = millis();
//    progress = (float)(currentTime - startTime) / durationMs;
//    if (progress > 1.0) progress = 1.0;
//
//    float pwmFraction = pow(progress, 0.5);  // Square root ramp
//    int pwmValue = (int)(maxPWM * pwmFraction);
//
//    analogWrite(PWMA, pwmValue);
//    digitalWrite(IN1, HIGH);
//    digitalWrite(IN2, LOW);
//
//    delay(10);
//  }
//
//  Serial.println("Shooter motor at full speed.");
//}
