#include <QTRSensors.h>

// Line Sensor & PID Configuration Variables
#define SETPOINT    1500  // Adjusted for 3-sensor setup (centered around middle sensor)
#define KP          0.2   // Proportional gain
#define KI          0.05
#define KD          1.0   // Derivative gain
#define MAX_SPEED   250   // Maximum motor speed
#define SET_SPEED   200   // Target speed
#define MIN_SPEED   0     // Minimum motor speed

// Pushbutton Pin
#define BUTTON_PIN  12 // Optional pushbutton pin (Teensy pin numbers)

// Motor Driver Pins (MC33926)
#define ENABLE_PIN  2 
#define L_PWM       3
#define R_PWM       4
#define L_IN1       5
#define L_IN2       6
#define R_IN1       7
#define R_IN2       8

// PID & Other Variables
int lastError = 0;
float integralError = 0;
bool calibrationDone = false;

// SENSORS (UCTRONICS 1x3 ARRAY)
#define NUM_SENSORS 3  // Adjusted for 3-sensor configuration
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2};  // Only 3 sensors (Left, Center, Right)
QTRSensors qtr;
unsigned int sensorValues[NUM_SENSORS];

void setup() {
    Serial.begin(9600); // Initialize Serial Monitor

    // Initialize pushbutton
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Initialize motor driver pins
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    // Enable Motor Driver
    digitalWrite(ENABLE_PIN, HIGH);

    // Initialize UCTRONICS Sensor
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);
    qtr.setEmitterPin(2); // Adjust this based on your wiring

    // Calibration
    Serial.println("Press Button To Start Calibration...");
    while (digitalRead(BUTTON_PIN) == HIGH) {
        calibrateSensors();
    }

    // Second button press to start robot
    Serial.println("Press Button To Start Line Following...");
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
    Serial.println("Calibration Complete!");
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    
    // Motor control logic
    digitalWrite(L_IN1, leftSpeed >= 0 ? HIGH : LOW);
    digitalWrite(L_IN2, leftSpeed >= 0 ? LOW : HIGH);
    analogWrite(L_PWM, abs(leftSpeed)); 

    digitalWrite(R_IN1, rightSpeed >= 0 ? HIGH : LOW);
    digitalWrite(R_IN2, rightSpeed >= 0 ? LOW : HIGH);
    analogWrite(R_PWM, abs(rightSpeed));
}

void loop() {
    if (!calibrationDone) return;

    // Read sensor values
    int position = qtr.readLineBlack(sensorValues);
    Serial.print("Sensor Values: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
    }
    Serial.println();

    // Calculate error for PID
    int error = position - SETPOINT;
    int correction = (KP * error) + (KI * integralError) + (KD * (error - lastError));
    integralError += error;
    lastError = error;

    // Adjust motor speed based on correction
    int leftSpeed = SET_SPEED - correction;
    int rightSpeed = SET_SPEED + correction;
    setMotorSpeed(leftSpeed, rightSpeed);

    delay(10);
}
