#include <QTRSensors.h>
// Line Sensor & PID Configuration Variables
#define SETPOINT    3500  // The goal for readLine (center)
#define KP          0.2   // The P value in PID
#define KI          0.05
#define KD          1.0     // The D value in PID
#define MAX_SPEED   250   // The max speed to set motors to
#define SET_SPEED   200   // The goal speed to set motors to
#define MIN_SPEED   0     // The min speed to set motors to

// Pushbutton Pin
#define BUTTON_PIN  12 // This is our (Optional) pushbutton pin - Teensy pin numbers

// Motor Driver Pins (MC33926) - Teensy pin nunbers
#define ENABLE_PIN 2 
#define L_PWM 3
#define R_PWM 4
#define L_IN1 5
#define L_IN2 6
#define R_IN1 7
#define R_IN2 8

// PID & Other Variables *******************
int lastError = 0;  // For storing PID error
float integralError = 0;
bool calibrationDone = false;

// SENSORS **********************************
#define NUM_SENSORS 7 
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6};
QTRSensors qtr;
unsigned int sensorValues[NUM_SENSORS];


void setup() {
  // Initalize Serial Communication(Using this for printing and other debugging possibilities)
  Serial.begin(9600); // 9600 bits per second

  // Initalize Optional Pushbutton
  // We're connecting the button to an internal pull up resistor here
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize Pins
  // pinMode basically assigns roles for all of the pin numbers, output means it is sending data input is readind data
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  // Enable Motor Driver
  digitalWrite(ENABLE_PIN, HIGH);

  // Initalize qtr Sensor
  // this links the sensor pins to our arrays that calculate the robots position
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPin(2);

  // Use pushbutton for calibration
  Serial.println("Press Button To Start Calibration...");
  while(digitalRead(BUTTON_PIN) == HIGH) {
    calibrateSensors();
  }

  // Second button press to start the robot
  Serial.println("Press Button To Start The Lazy Suzan Shooter!");
  while(digitalRead(BUTTON_PIN) == HIGH) {
    Serial.println("Starting Robot...");
    calibrationDone = true;
  }
}

void calibrateSensors() {
  Serial.println("Calibration Starting...");
  for(int i = 0; i < 100; i++) {
      qtr.calibrate();
      delay(10);
  }
  Serial.println("Calibration Successful!");
}


void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Set Speed Constraints(Can change later)
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    
  // Motor Direction Logic (Turnary opperator logic determines forwards or backwards motor rotation)
  digitalWrite(L_IN1, leftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(L_IN2, leftSpeed >= 0 ? LOW : HIGH);
  analogWrite(L_PWM, abs(leftSpeed)); // Set left motor speed

  digitalWrite(R_IN1, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(R_IN2, rightSpeed >= 0 ? LOW : HIGH);
  analogWrite(R_PWM, abs(rightSpeed)); // Set left motor speed
}

void loop() {
//Insert main code here, include logic where main while loop stops when all sensors detect black
// In order to complete this add more functions above controling the lazy suzan and also the launcher wheel
}