#include <Arduino.h>
#include "MC33926MotorShield.h"
#include "QTRSensors.h"
#include "Adafruit_VL53L0X.h"
#include <Wire.h>  // Include the Wire library for custom I2C pins

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int XSHUT_PIN = 17;  // Pin for controlling the XSHUT pin (D17)



// PID constants just for outside edge following
#define EDGE_KP 0.05
#define EDGE_KI 0.0
#define EDGE_KD 2.0
#define EDGE_SETPOINT 500  // Tune this to match sensor[6] value when properly aligned

float edge_integral = 0;
int edge_lastError = 0;


// Constants
#define SENSOR_NUMS 7
#define SETPOINT 3000  // Middle of sensor range
#define MAX_SPEED 225
#define MIN_SPEED 0
#define SET_SPEED 180 // increase this with
#define KP .07      // increase this with set speed
#define KI 0
#define KD 1.5
#define SLEEP_PIN 6

#define IGNORE_ERROR_THRESHOLD 3000

// enum for our state machine
enum RobotState {
  WAIT_FOR_CALIBRATION_BUTTON,
  CALIBRATING,
  READY_TO_START,
  DELAY_BEFORE_START,
  RUNNING
};

RobotState currentState = WAIT_FOR_CALIBRATION_BUTTON;
const int buttonPin = 40;  // or whatever pin you're using for the pushbutton


// QTR Setup
QTRSensors qtr;
uint16_t sensorValues[SENSOR_NUMS];  // Raw readings

// Motors Setup
MC33926MotorShield leftMotor(12, 10, 11, 24, 26);
MC33926MotorShield rightMotor(9, 7, 8, 25, 26);

// PID Variables
float integralError = 0;
int lastError = 0;

void setup() {
  Serial.begin(115200);
  qtr.setTypeAnalog();
  // Initialize Motors
  leftMotor.begin();
  rightMotor.begin();

  // button setup
  pinMode(buttonPin, INPUT_PULLUP);  // Setup button pin

 

  // QTR setup
  qtr.setSensorPins((const uint8_t[]){16, 15, 14, 41, 40, 39, 38}, SENSOR_NUMS);
  qtr.setTypeAnalog();  // Important!

  // Calibrate sensors here later
  // for (int i = 0; i < 250; i++) {
  //   qtr.calibrate();
  //   delay(20);
  // }
  for (int i = 0; i < 1000; i++) {
    qtr.calibrate();
    delay(5);
  }


  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, HIGH);  // Bring the sensor out of reset
  delay(100);                      // Wait for the sensor to boot
  Wire.begin();  

  while (!lox.begin(VL53L0X_I2C_ADDR, 0, &Wire)) {
    Serial.println(F("Failed to boot VL53L0X"));
    
  }
  
  lox.startRangeContinuous();
  Serial.println("Setup complete.");
}
//*******************************************************************************************************************************************************************************************************************
void loop() {


  uint16_t range = lox.readRange();
  
  Serial.print("Distance (mm): ");
  Serial.println(range);
  delay(5);
  //bool detectStrip;
  
  if(range < 200){
    //detectStrip = false;
    lineFollow();
  } else{
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    while(1);
  }
  

  

}
//*******************************************************************************************************************************************************************************************************************
void testQTR() {
  // Read raw sensor values into the array
  qtr.read(sensorValues);

  // Print each value to the Serial Monitor
  for (int i = 0; i < SENSOR_NUMS; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }

  Serial.println();  // Move to next line after printing all sensor values
}

// returns a position uint16 value from the qtr sensor
int readPosition() {
  return qtr.readLineBlack(sensorValues);  // Populate sensorValues[]
}

//*****************************************************************************//LINE FOLLOWING//******************************************************************************************************************
void lineFollow() {
  int position = readPosition();
  int error = position - SETPOINT;

  // PID math
  int proportional = error;
  integralError += error;
  int derivative = error - lastError;
  int correction = (KP * proportional) + (KI * integralError) + (KD * derivative);
  lastError = error;

  // Speed adjustment
  int leftSpeed = SET_SPEED - correction;
  int rightSpeed = SET_SPEED + correction;

  // Asymmetric clamping for sharper turns
  // if (abs(error) > 3800) {
  //   if (error > 0) {
  //     // Turn right hard
  //     leftSpeed = MAX_SPEED;
  //     rightSpeed = 0;
  //   } else {
  //     // Turn left hard
  //     leftSpeed = 0;
  //     rightSpeed = MAX_SPEED;
  //   }
  // } else {
  //   // Normal clamping
  //   leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  //   rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
  // }

  leftSpeed = constrain((SET_SPEED + correction), MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain((SET_SPEED - correction), MIN_SPEED, MAX_SPEED);

  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

void testError() {
  int position = readPosition();
  int error = position - SETPOINT;
    Serial.print("Pos: ");
  Serial.print(position);
  Serial.print(" | Error: ");
  Serial.println(error);
  delay(25);
}

bool isBlackBarDetected() {
  qtr.read(sensorValues);
  int darkCount = 0;
  for (int i = 0; i < SENSOR_NUMS; i++) {
    if (sensorValues[i] > 800) darkCount++;  // Adjust threshold
  }
  return darkCount >= 6;  // Most sensors see black
}

void handleStateMachine() {

  switch (currentState) {
    case WAIT_FOR_CALIBRATION_BUTTON:
      if (digitalRead(buttonPin) == LOW) {
        Serial.println("Button pressed. Starting calibration...");
        currentState = CALIBRATING;
      }
      break;

    case CALIBRATING:
      for (int i = 0; i < 1000; i++) {
        // You don’t need qtr.calibrate() if using manual analog reads
        qtr.calibrate();
        delay(5);
      }
      Serial.println("Calibration done. Press button again to start...");
      currentState = READY_TO_START;
      break;

    case READY_TO_START:
      if (digitalRead(buttonPin) == LOW) {
        currentState = DELAY_BEFORE_START;
        delay(1000);
      }
      break;

    case DELAY_BEFORE_START:
        currentState = RUNNING;
        delay(2000);
      break;

    case RUNNING:
      break;
  }
}

void lineFollowOutside() {
  qtr.read(sensorValues);  // Get all sensor values
  int sensorValue = sensorValues[6];  // Use only right-most sensor (index 6)

  // Calculate error from target intensity
  int error = sensorValue - EDGE_SETPOINT;

  // PID math
  edge_integral += error;
  int derivative = error - edge_lastError;
  int correction = (EDGE_KP * error) + (EDGE_KI * edge_integral) + (EDGE_KD * derivative);
  edge_lastError = error;

  // Adjust speeds based on correction
  int leftSpeed = SET_SPEED + correction;
  int rightSpeed = SET_SPEED - correction;

  // Clamp speeds
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  // Set motor speeds
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}
