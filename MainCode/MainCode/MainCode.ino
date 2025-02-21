#include <QTRSensors.h>

// Line Sensor & PID Configuration Variables
#define SETPOINT    3500  // The goal for readLine (center)
#define KP          0.2   // The P value in PID
#define KI          0.05
#define KD          1.0   // The D value in PID
#define MAX_SPEED   250   // The max speed to set motors to
#define SET_SPEED   200   // The goal speed to set motors to
#define MIN_SPEED   0     // The min speed to set motors to

// Pushbutton Pin
#define BUTTON_PIN  12 // Optional pushbutton pin

// Motor Driver Pins (MC33926)
// test comment meow
#define ENABLE_PIN  2
#define L_PWM       3
#define R_PWM       4
#define L_IN1       5 // motor 1 = left motor
#define L_IN2       6
#define R_IN1       7 //motor 2 = right motor
#define R_IN2       8
// slew is gpio pin 11
//M2 Feedback(FB) gpio 9
//M1 Feedback(FB) gpio 10

// PID & Other Variables
int lastError = 0;  // For storing PID error
float integralError = 0;
bool calibrationDone = false;

// SENSORS **********************************
#define NUM_SENSORS 7 
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6};
QTRSensors qtr;
unsigned int sensorValues[NUM_SENSORS];

void setup() {
    Serial.begin(9600); // Initialize Serial Monitor

    pinMode(BUTTON_PIN, INPUT_PULLUP); // Initialize Pushbutton
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    digitalWrite(ENABLE_PIN, HIGH); // Enable motor driver

    // Initialize QTR sensor
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);
    qtr.setEmitterPin(2);

    // Calibration Phase
    Serial.println("Press Button To Start Calibration...");
    while (digitalRead(BUTTON_PIN) == HIGH) {
        calibrateSensors();
    }

    // Wait for the second button press to start the robot
    Serial.println("Press Button To Start The Line Following...");
    while (digitalRead(BUTTON_PIN) == HIGH);

    Serial.println("Starting Robot...");
    calibrationDone = true;
}

void calibrateSensors() {
    Serial.println("Calibration Starting...");
    for (int i = 0; i < 100; i++) {
        qtr.calibrate();
        delay(10);
    }
    Serial.println("Calibration Successful!");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Apply Speed Constraints
    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    // Motor Direction Logic
    digitalWrite(L_IN1, leftSpeed >= 0 ? HIGH : LOW);
    digitalWrite(L_IN2, leftSpeed >= 0 ? LOW : HIGH);
    analogWrite(L_PWM, abs(leftSpeed));

    digitalWrite(R_IN1, rightSpeed >= 0 ? HIGH : LOW);
    digitalWrite(R_IN2, rightSpeed >= 0 ? LOW : HIGH);
    analogWrite(R_PWM, abs(rightSpeed));
}

void loop() {
    if (!calibrationDone) return; // Prevent the robot from running if not calibrated

    // Read sensor values and determine position
    int position = qtr.readLineBlack(sensorValues);
    Serial.print("Position: ");
    Serial.println(position);

    // Compute PID error
    int error = position - SETPOINT;
    
    // Calculate PID terms
    int proportional = error;
    integralError += error;
    int derivative = error - lastError;

    // PID Correction
    int correction = (KP * proportional) + (KI * integralError) + (KD * derivative);
    lastError = error; // Update last error for next loop

    // Adjust motor speeds based on correction
    int leftSpeed = SET_SPEED - correction;
    int rightSpeed = SET_SPEED + correction;

    setMotorSpeed(leftSpeed, rightSpeed); // Set motor speeds

    delay(10); // Small delay for stability
}
