#include "SparkFun_TB6612.h"
#include <Encoder.h>

// Motor control pins
#define AIN1 2
#define AIN2 3
#define PWMA 4
#define STBY 9

// Motor direction offset
const int offsetA = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Encoder myEnc(5, 6);  // Attach encoder to pins 5 and 6

// Target encoder displacement for 90-degree rotation
const long targetDisplacement = 123;  
long oldPosition = -999;

void setup() {
  Serial.begin(9600);
  myEnc.write(0); // Reset encoder position to 0 at startup
}

void loop() {
  motor1.drive(50); // Drive motor forward at speed 50 (no timeout)

  long newPosition = myEnc.read();
  
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition); 
  }

  if (abs(newPosition) >= targetDisplacement) {
    motor1.brake(); // Stop motor
    Serial.println("Reached 90 degrees!");
    myEnc.write(0); // Reset encoder position to 0 at startup
    delay(1000);
    newPosition = 0;
  }
}
