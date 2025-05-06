#include <Encoder.h>
#include "MC33926MotorShield.h"

// pins have to be interrupt capable on the teensy
// this library actually uses interupts
Encoder myEnc(x, x);
MC33926MotorShield motor(x, x, x, x, x);

void setup() {
  motor.begin();
  Serial.begin(9600);
}

void loop() {
  rotateMotor90();
  delay(2000);
}

void rotateMotor90() {
  myEnc.write(0);         // Reset the encoder count to 0
  motor.setSpeed(150);    // Start spinning the motor forward at speed 150

  // look up the gear ratio and the CPR of the motor, multiply them together and thats the number of ticks
  //for a full rotation, divide by 4 to get the quarter turn number. That should be 600, CPR = 12, 50:1 gear ratio, (12 x 50) / 4 = 150
  while (abs(myEnc.read()) < 150) {
    // Wait here until the encoder count reaches 600 ticks (or whatever corresponds to 90°)
  }

  motor.setSpeed(0);      // Stop the motor once target position is reached
}

