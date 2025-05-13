#include "SparkFun_TB6612.h"
#include <Encoder.h>

// Motor control pins
#define AIN1 2
#define AIN2 3
#define PWMA 4
#define STBY 9

const int offsetA = 1;
Motor motor1 = Motor(AIN2, AIN1, PWMA, offsetA, STBY);
Encoder myEnc(5, 6);  // Encoder connected to output shaft

// Movement settings
const long targetDisplacement = 123;
const long slowDownThreshold = 30;

long lastPosition = 0;
bool isSlowingDown = false;
bool hasReachedTarget = false;


void setup() {
  Serial.begin(9600);
  myEnc.write(0);
  delay(100);
  motor1.drive(50); // Start movement
}

void loop() {
  delay(10);

  long position = abs(myEnc.read());

  // Ignore jitter
  if (abs(position - lastPosition) >= 1) {
    lastPosition = position;
    Serial.println(position);
  }

  // Transition logic
  if (!hasReachedTarget) {
    if (position >= targetDisplacement) {
      // Stop condition
      motor1.brake();
      Serial.println("Reached target!");
     
      hasReachedTarget = true;

      myEnc.write(0); // Reset encoder
      delay(2000);   
      motor1.drive(50); 
     
      hasReachedTarget = false;
      isSlowingDown = false;
      lastPosition = 0;
      Serial.println("Restarting...");
    } 
    else if (!isSlowingDown && position >= targetDisplacement - slowDownThreshold) {
      // Begin slowing down
      motor1.drive(50);
     
      isSlowingDown = true;
      Serial.println("Slowing down...");
    }
  }
}
