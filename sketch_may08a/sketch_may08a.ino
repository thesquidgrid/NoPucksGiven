// what we need to get done:
// - funneling stuff with black puck 
// - creating proper routine for shooting pucks in the way our diagram shows
// setting up pushbutton and properly integrating state machine
// auto calibration for our local board - will use auto calibrate on the competition board
// tof sensing 200 - 160 to know when to turn the lazy susan to funnel the puck in

//what is done:
//line following:
//lazy susan turning
//finding the black line and starting to linefollowing
//tof goal finder works

/**
 * @file sketch_may08a.ino
 * @brief Controls the NoPucksGiven robot.
 * @author Greg and Sophia
 * @date 2025-05-12
 */

//imports
#include <Arduino.h>

#include <TeensyThreads.h>


#include "MC33926MotorShield.h"

#include "QTRSensors.h"

#include <VL53L0X.h>

#include <Wire.h>  // Include the Wire library for custom I2C pins

#include "SparkFun_TB6612.h"

#include "Encoder.h"

//TOF STUFF
VL53L0X sensor;
#define LONG_RANGE
/** @brief Pin for controlling the XSHUT pin (D17).*/
const int XSHUT_PIN = 17;
/** @brief target displacement for sensing the goal.*/
long oldPosition = -999;

//LINE FOLLOWING:
// PID constants just for outside edge following
#define EDGE_KP 0.05
#define EDGE_KI 0.0
#define EDGE_KD 2.0
#define EDGE_SETPOINT 500 // Tune this to match sensor[6] value when properly aligned

float edge_integral = 0;
int edge_lastError = 0;

// Constants for PID line following

#define SETPOINT 3000 // Middle of sensor range
#define KP .07 // increase this with set speed
#define KI 0
#define KD 1.5
#define IGNORE_ERROR_THRESHOLD 3000
// PID Variables
float integralError = 0;
int lastError = 0;

//QTR constants
#define BLACK_THRESHOLD 300 /* adjust this for our starting sequence*/
// Define sensor pins and value array
#define SENSOR_NUMS 7

// QTR Setup
QTRSensors qtr;
uint16_t sensorValues[SENSOR_NUMS]; // Raw readings

// Motors Setup
/** @brief Motor setup for controlling the left motor*/
MC33926MotorShield leftMotor(12, 10, 11, 24, 26);
/** @brief Motor setup for controlling the left motor*/
MC33926MotorShield rightMotor(9, 7, 8, 25, 26);

#define MAX_SPEED 220
#define MIN_SPEED 0
#define SET_SPEED 180 // increase this with

//ENCODER FOR MOTORS LEFT AND RIGHT
//left:
Encoder leftM(28, 27);
//right:
Encoder rightM(30, 29);

//SHOOTER MOTOR STUFF:
#define IN1_sm 21
#define IN2_sm 20
#define PWMA_sm 37

//LAZY SUSAN motor control:
#define AIN1_lsm 2
#define AIN2_lsm 3
#define PWMA_lsm 4
#define STBY_lsm 9

// Motor direction offsset
const int offsetA = 1;

//LAZY SUSAN MOTOR
/** @brief Motor setup for controlling the lazy susan motor*/
Motor ls_motor = Motor(AIN1_lsm, AIN2_lsm, PWMA_lsm, offsetA, STBY_lsm);
/** @brief encoder setup for controlling the lazy susan motor*/
Encoder lsMotor(5, 6); // Attach encoder to pins 5 and 6
// Movement settings

long lastPosition = 0;
bool isSlowingDown = false;
bool hasReachedTarget = false;

/**
 * @brief setup function.
 */

void setup() {

   pinMode(LED_BUILTIN, OUTPUT); // built in teensy light for state knowledge

   Serial.begin(115200);
   // Initialize Motors
   leftMotor.begin();
   rightMotor.begin();

   //initalize sensor pins
   qtr.setSensorPins((const uint8_t[]) {
      16,
      15,
      14,
      41,
      40,
      39,
      38
   }, SENSOR_NUMS);
   qtr.setTypeAnalog(); // Important!

   //for i2c
   Wire.begin();

   //FOR TOF - TURNS IT ON
   pinMode(XSHUT_PIN, OUTPUT);
   digitalWrite(XSHUT_PIN, HIGH); // Bring the sensor out of reset
   delay(100);

   sensor.setTimeout(500);
   while (!sensor.init()) {
      Serial.println("Failed to detect and initialize sensor!");
      sensor.init();
   }
   sensor.setSignalRateLimit(0.1);
   // increase laser pulse periods (defaults are 14 and 10 PCLKs)
   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
   sensor.setMeasurementTimingBudget(200000);
   sensor.startContinuous();

   //reset all encoders
   rightM.write(0);
   leftM.write(0);
   lsMotor.write(0);

   //shooter motor setup:
   pinMode(IN1_sm, OUTPUT);
   pinMode(IN2_sm, OUTPUT);
   pinMode(PWMA_sm, OUTPUT);
   digitalWrite(IN1_sm, HIGH);
   digitalWrite(IN2_sm, LOW);

   flashLED();
   Serial.println("1. calibrate robot");
   Serial.println("2. line follow");
   Serial.println("3. line follow until goal");
   Serial.println("4. lazy susan rotate");
   Serial.println("5. shooter motor");
   Serial.println("6. test ToF");
   Serial.println("7. moveforward a little bit");
   Serial.println("8. move forward and shoot puck");
   Serial.println("input choice: ");
}

/**
 * @brief loop function.
 */
void loop() {
   // put your main code here, to run repeatedly:
   //debug menu 1.) calibrate 2.) run code:
   //go forward until black line
   //turn until black line
   //go until robot other side
   //run code to shoot puck
   //shoot puck and then continue on
   
   

   if (Serial.available() > 0) {
      int sel = Serial.parseInt();
      debugMenu(sel);
      Serial.println("1. calibrate robot");
      Serial.println("2. line follow");
      Serial.println("3. line follow until goal");
      Serial.println("4. lazy susan rotate");
      Serial.println("5. shooter motor");
      Serial.println("6. test ToF");
      Serial.println("7. moveforward a little bit");
      Serial.println("8. move forward and shoot puck");
      Serial.println("input choice: ");
   } else {
      int meow = Serial.read();
      delay(200);
   }
}
/**
 * @brief turn on puck shooter motor then turn off.
 * @return void. 
 */
void shootPuck() {
   analogWrite(PWMA_sm, 1000);
   delay(10000);
   analogWrite(PWMA_sm, 0);
}

void lineFollowUntilGoal() {
  bool flag = false;
  uint16_t range = sensor.readRangeSingleMillimeters();
  bool flag160 = false;
  int counter = 0;
  while (!flag) {
    range = sensor.readRangeSingleMillimeters();
    delay(5);

    lineFollow();
    if (range > 160) {
      flag160 = true;
    }
    if ((range > 100) && (range < 140) && (flag160 == true)) {
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      flag = true;
    }
    Serial.println(range);
  }
  Serial.println("found goal");
}

void lineFollowUntilGoalOtherSide() {
   bool flag = false;
   uint16_t range = sensor.readRangeSingleMillimeters();
   bool flag180 = false;
   int counter = 0;
   while (!flag) {
      range = sensor.readRangeSingleMillimeters();
      delay(5);

      lineFollow();
      if (range > 180) {
         flag180 = true;
      }
      if ((range > 180) && (range < 190) && (flag180 == true)) {
         leftMotor.setSpeed(0);
         rightMotor.setSpeed(0);
         flag = true;
      }
      Serial.println(range);
   }
   Serial.println("found goal");
}

/**
 * @brief readPosition of qtr.
 * @return int what the qtr sensor senses. 
 */
int readPosition() {
   return qtr.readLineBlack(sensorValues); // Populate sensorValues[]
}

/**
 * @brief PID linefollow.
 * @return void
 */
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

   leftSpeed = constrain((SET_SPEED + correction), MIN_SPEED, MAX_SPEED);
   rightSpeed = constrain((SET_SPEED - correction), MIN_SPEED, MAX_SPEED);

   leftMotor.setSpeed(leftSpeed);
   rightMotor.setSpeed(rightSpeed);
}

/**
 * @brief have the robot go forward until it reaches the black line, then start linefollwing.
 * @return void
 */
void driveUntilBlackThenTurn() {
   // Drive forward
   leftMotor.setSpeed(150);
   rightMotor.setSpeed(150);

   // Wait until all sensors see black
   bool detectStrip = false;
   int sensorCount = 0;
   while (!detectStrip) {
      qtr.read(sensorValues);

      for (int i = 0; i < SENSOR_NUMS; i++) {
         if (sensorValues[i] > BLACK_THRESHOLD) {
            sensorCount++;
         }
      }
      if (sensorCount > 3) {
         detectStrip = true;
      } else {
         sensorCount = 0;
      }
      delay(5); //delay to give it its bearings
   }

   Serial.println("found black line");
   // stop and delay
   leftMotor.setSpeed(0);
   rightMotor.setSpeed(0);
   delay(200);

   // move forward for just a little bit
   rightMotor.setSpeed(150);
   leftMotor.setSpeed(150);
   delay(200); /* adjust this delay to make it consistent*/

   //stop
   leftMotor.setSpeed(0);
   rightMotor.setSpeed(0);
   delay(200);
}

/**
 * @brief test the QTR and the values it's reading
 * @return void
 */

void moveForwardALittleBit() {
 
    
   leftMotor.setSpeed(150);
   rightMotor.setSpeed(170);
   delay(500);
   leftMotor.setSpeed(0);
   rightMotor.setSpeed(0);
}

void test() {
   moveForwardALittleBit();
   long lastPosition = 0;
   bool isSlowingDown = false;
   bool hasReachedTarget = false;
   const long slowDownThreshold = 40;

   int direction = 1;
   int targetDisplacement = 110; // Work with magnitude for thresholding

   lsMotor.write(0); // Reset encoder before starting
   delay(100);

   ls_motor.drive(40 * direction); // Start moving in correct direction
   while (!hasReachedTarget) {

      delay(10);

      long position = abs(lsMotor.read()); // Use absolute position for comparison
      long currentPosition = lsMotor.read(); // For signed value if needed for debug

      // Print encoder position if changed
      if (abs(position - lastPosition) >= 1) {
         lastPosition = position;
         Serial.println(currentPosition); // show signed 
         analogWrite(PWMA_sm, 1000);
      }

      // Transition logic
      if (position >= targetDisplacement) {
         // Stop condition
         ls_motor.brake();
         Serial.println("Reached target!");

         hasReachedTarget = true;
         delay(2000);
         lsMotor.write(0); // Reset encoder
         lastPosition = 0;
      } else if (!isSlowingDown && position >= targetDisplacement - slowDownThreshold) {
         // Begin slowing down

         ls_motor.drive(50 * direction);
         isSlowingDown = true;
         Serial.println("Slowing down...");
      }
   }
   delay(1000);
   analogWrite(PWMA_sm, 0);
}
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

   Serial.println(); // Move to next line after printing all sensor values
}

/**
 * @brief have the robot go forward until it reaches the black line, then start linefollwing.
 * @return void
 */
void goRightUntilFindBlackLine() {
   // Drive forward
   leftMotor.setSpeed(150);

   // Wait until all sensors see black
   bool detectStrip = false;
   int sensorCount = 0;
   while (!detectStrip) {
      qtr.read(sensorValues);

      for (int i = 0; i < SENSOR_NUMS; i++) {
         if (sensorValues[i] > BLACK_THRESHOLD) {
            sensorCount++;
         }
      }
      if (sensorCount > 1) {
         detectStrip = true;
      } else {
         sensorCount = 0;
      }
      delay(5); //delay to give it its bearings
   }

   Serial.println("found black line");
   // stop and delay
   leftMotor.setSpeed(0);
   rightMotor.setSpeed(0);
}

/**
 * @brief move the lazy susan to however many degrees you want it to go.
 * @return void
 */

void moveDegrees(int targetDisplacement) {
   long lastPosition = 0;
   bool isSlowingDown = false;
   bool hasReachedTarget = false;
   const long slowDownThreshold = 30;

   int direction = (targetDisplacement >= 0) ? 1 : -1;
   targetDisplacement = abs(targetDisplacement); // Work with magnitude for thresholding

   lsMotor.write(0); // Reset encoder before starting
   delay(100);

   ls_motor.drive(100 * direction); // Start moving in correct direction

   while (!hasReachedTarget) {
      delay(10);

      long position = abs(lsMotor.read()); // Use absolute position for comparison
      long currentPosition = lsMotor.read(); // For signed value if needed for debug

      // Print encoder position if changed
      if (abs(position - lastPosition) >= 1) {
         lastPosition = position;
         Serial.println(currentPosition); // show signed value
      }

      // Transition logic
      if (position >= targetDisplacement) {
         // Stop condition
         ls_motor.brake();
         Serial.println("Reached target!");

         hasReachedTarget = true;
         delay(2000);
         lsMotor.write(0); // Reset encoder
         lastPosition = 0;
      } else if (!isSlowingDown && position >= targetDisplacement - slowDownThreshold) {
         // Begin slowing down
         ls_motor.drive(50 * direction);
         isSlowingDown = true;
         Serial.println("Slowing down...");
      }
   }
}

void moveDegreesAndShoot(int deg) {
   threads.start(); // Pause all thread switching
   moveDegrees(deg); // Call your function that shouldn't be interrupted
   threads.stop(); // Resume normal thread scheduling
}

/**
 * @brief flash teensy led.
 * @return void
 */

void flashLED() {
   digitalWrite(LED_BUILTIN, HIGH);
   delay(500);
   digitalWrite(LED_BUILTIN, LOW);
}

void lineFollowUntilGoalAndExtras() {
   bool flag = false;
   uint16_t range = sensor.readRangeSingleMillimeters();
   bool flag160 = false;
   while (!flag) {
      range = sensor.readRangeSingleMillimeters();
      delay(5);
      lineFollow();
      if (range > 160) {
         flag160 = true;
      }
      if ((range > 100) && (range < 140) && (flag160 == true)) {
         leftMotor.setSpeed(0);
         rightMotor.setSpeed(0);
         flag = true;
      }

      Serial.println(range);
   }
   Serial.println("found goal");
}

/**
 * @brief debug menu
 * @param int selection - menu select
 * @return void
 */
void debugMenu(int selection) {
   delay(300);
   switch (selection) {
   case 1:
      Serial.println("calibrate robot");
      delay(1000);
      Serial.println("calibrate robot");
      for (int i = 0; i < 1000; i++) { //calibrate robot
         qtr.calibrate();
         delay(5);
      }
      Serial.println("robot calibrate done");
      delay(5000);
      break;

   case 2: {
      Serial.println("linefollow");
      bool flag = false;
      while (!flag) {
         lineFollow();
         Serial.println("m");
         if (Serial.available() > 0) {
            flag = true;
         }
      }
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
   }
   break;
   case 3:
      Serial.println("line follow until goal");
      lineFollowUntilGoal();
      break;
   case 4: {
      Serial.println("rotate lazy susan");
      Serial.print("what do you want it to rotate to? ");
      waitForUserInput();
      delay(500);
      int deg = Serial.parseInt();
      moveDegrees(deg);
   }
   break;
   case 5:
      Serial.println("turn on shooter motor");
      shootPuck();
      break;
   case 6: {
      Serial.println("test ToF");
      bool flag = false;

      uint16_t range = sensor.readRangeSingleMillimeters();
      while (!flag) {
         range = sensor.readRangeSingleMillimeters();
         Serial.println(range);
         delay(100);
         if (Serial.available() > 0) {
            flag = true;
         }
      }
   }
   break;

   case 7:
      test();
      break;
   case 8:
      moveForwardALittleBit();
      break;
   }
   

}

/**
 * @brief wait for user input from serial console.
 * @return void
 */
void waitForUserInput() {

   while (Serial.available() <= 0) {}
}
//void debug console:
//case 1 2 3 4 etc.

//each time case asks you what value you want.
//case stop
