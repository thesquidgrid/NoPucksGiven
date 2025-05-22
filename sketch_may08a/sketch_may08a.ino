// what we need to get done:
// - funneling stuff with black puck 
// - creating proper routine for shooting pucks in the way our diagram shows

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

#define MAX_SPEED 200
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
int puckCount = 0;


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
  debugPrintout();
}



void debugPrintout(){
  Serial.println("1. calibrate robot");
  Serial.println("2. line follow until goal");
  Serial.println("3. line follow until puck");
  Serial.println("4. lazy susan rotate");
  Serial.println("5. shooter motor");
  Serial.println("6. test ToF");
  Serial.println("7. testing funnle routine");
  Serial.println("8. back up until black");
  Serial.println("9. go until puck inside:");
  Serial.println("10. shoot puck:");
  Serial.println("11. run all until puck 1 shoot:");
  Serial.println("12. corner puck 2:");
  Serial.println("13. run all until puck 2 shoot:");
  Serial.println("14. run all until puck 3 shoot:");
  Serial.println("15. run all until puck 4 shoot:");
  Serial.println("input choice: ");
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

void feedAndShoot(){
  ls_motor.drive(-120);
  delay(2000);
  ls_motor.drive(-240); 
  lsMotor.write(0);

  while(lsMotor.read() < 75){
    analogWrite(PWMA_sm, 1000);
  }
  ls_motor.brake();
  analogWrite(PWMA_sm, 0);
  puckCount++;
}

void feedAndShoot2(){
  analogWrite(PWMA_sm, 1000);
  ls_motor.drive(-200);
  delay(20);
  ls_motor.drive(-240); 
  lsMotor.write(0);
  
  while(lsMotor.read() < 300){
    
  }
  ls_motor.brake();
  analogWrite(PWMA_sm, 0);
  puckCount++;
}

void lineFollowUntilPuck(){
  int sensorCount = 0;
  bool stopLoop = false;
  uint16_t range = sensor.readRangeSingleMillimeters();

  lsMotor.write(0); // Reset encoder before starting
  delay(100);
  int counter = 0;
  while (!stopLoop) {
    range = sensor.readRangeSingleMillimeters();

    if(range < 45){ //how you know a puck is coming in
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0); //pause line following for a second
      delay(200);
      Serial.println("found Puck");
      stopLoop = true;
    } else {
      lineFollow();
    }

    delay(5); //delay to give it its bearings
  }

  // stop and delay
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}

void lineFollowCornerPuck(){
  unsigned long currentMillis = 0;
  const unsigned long period = 2000;  //the value is a number of milliseconds between changes of brightness
  while(currentMillis < period){
    currentMillis = millis(); 
    lineFollow();
  }
  goForwardUntilPuckInside();
  goRightUntilFindBlackLine();
  delay(50);
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  //funnel(-120, 80);
}

/**
 * @brief invoke line following function until the tof sensor senses the goal.
 * @return void. 
 */
void lineFollowUntilGoal() {
  bool detectStrip = false;
  int sensorCount = 0;

  lsMotor.write(0); // Reset encoder before starting
  delay(100);
  int counter = 0;
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
      lineFollow();
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
 * @brief readPosition of qtr.
 * @return int what the qtr sensor senses. 
 */
int readPosition() {
  return qtr.readLineBlack(sensorValues); // Populate sensorValues[]
}

void goForwardALittleBit(){
  leftMotor.setSpeed(100);
  rightMotor.setSpeed(100);
  delay(1000);
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
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
    leftMotor.setSpeed(150);
    rightMotor.setSpeed(150);

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

void backUpUntilBlack() {
  // Drive forward
  leftMotor.setSpeed(-150);
  rightMotor.setSpeed(-150);

  // Wait until 3 sensors see black
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
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  delay(200);
}

/**
 * @brief test the QTR and the values it's reading
 * @return void
 */
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

void goLeftUntilFindBlackLine() {
  // Drive forward
  
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
void moveDegrees(int targetDisplacement, int speed) {
  long lastPosition = 0;
  bool isSlowingDown = false;
  bool hasReachedTarget = false;
  const long slowDownThreshold = 30;

  int direction = (targetDisplacement >= 0) ? 1 : -1;
  targetDisplacement = abs(targetDisplacement); // Work with magnitude for thresholding

  lsMotor.write(0); // Reset encoder before starting
  delay(100);

  ls_motor.drive(speed * direction); // Start moving in correct direction

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
    } 
  }
}



void returnToOrigin() {
  long currentPosition = lsMotor.read();  
  int displacementToOrigin = -currentPosition; 
  Serial.print("Returning to origin from: ");
  Serial.println(currentPosition);
  moveDegrees(displacementToOrigin, 40);  
}

void funnel(int targetDisplacement, int speed) {
  long lastPosition = 0;
  bool isSlowingDown = false;
  bool hasReachedTarget = false;
  const long slowDownThreshold = 30;

  int direction = (targetDisplacement >= 0) ? 1 : -1;
  targetDisplacement = abs(targetDisplacement); // Work with magnitude for thresholding

  lsMotor.write(0); // Reset encoder before starting
  delay(100);

  ls_motor.drive(speed * direction); // Start moving in correct direction

  while (!hasReachedTarget) {
    delay(10);
    ls_motor.drive(speed * direction); 
    delay(50);
    ls_motor.brake();
    delay(10);
    leftMotor.setSpeed(150);
    rightMotor.setSpeed(150);
    delay(200);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);


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
    } 
  }
}




 // Move main drive motors forward
   

/**
 * @brief flash teensy led.
 * @return void
 */

void flashLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}



/**
 * @brief loop function.
 */
void loop() {
//  debugMenu(16);
//  delay(3000);
  Serial.println("CALIBRATE NOW");
  debugMenu(1);
  delay(1000);
  debugMenu(11);
  delay(500);
  debugMenu(13);
  delay(500);
  debugMenu(14);
  delay(500);
  debugMenu(15);
  delay(500);
  while(1);
  if (Serial.available() > 0) {
    int sel = Serial.parseInt();
    debugMenu(sel);
    debugPrintout();
  } else {
    int meow = Serial.read();
    delay(200);
  }
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
    delay(3000);
    break;

  case 2: {
    Serial.println("line follow until goal");
    lineFollowUntilGoal();
  }
  break;
  case 3:
    Serial.println("line follow until puck");
    lineFollowUntilPuck();
    break;
  case 4: {
    Serial.println("rotate lazy susan");
    Serial.print("what do you want it to rotate to? ");
    waitForUserInput();
    delay(500);
    int deg = Serial.parseInt();
    moveDegrees(deg, 100);
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
    Serial.println("funnle");
    funnel(-150, 80);
  break;
  case 8:
    Serial.println("back up");
    backUpUntilBlack();
  break;
  case 9:
    Serial.println("go until puck inside:");
    goForwardUntilPuckInside();
  break;
  case 10:
    Serial.println("shoot puck");
    feedAndShoot();
  break;
  case 11:
    Serial.println("run all");
    driveUntilBlackThenTurn();
    delay(50);
    goRightUntilFindBlackLine();
    lineFollowUntilPuck();
    delay(50);
    goForwardUntilPuckInside();
    delay(50);
    Serial.println("funnel");
    funnel(-150, 60);
    delay(50);
    backUpUntilBlack();
    delay(50);
    lineFollowUntilGoal();
    delay(50);
    goForwardALittleBit();
    feedAndShoot();
    delay(50);
  break;

  case 12: 
    Serial.println("cornerPuck");
    lineFollowCornerPuck();
  break;

  case 13:
    lineFollowUntilPuck();
    delay(50);
    lineFollowCornerPuck();
    moveDegrees(-30,50);
    delay(1000);
    goRightUntilFindBlackLine();
    goForwardUntilPuckInside2();   
    funnel(-150, 60);
    delay(50);
    lineFollowUntilPuck();
    delay(50);
    goForwardUntilPuckInside();
    delay(50);
    Serial.println("funnel");
    funnel(-150, 60);
    delay(50);
    backUpUntilBlack();
    delay(50);
    lineFollowUntilGoal();
    delay(50);
    goForwardALittleBit();
    feedAndShoot();
  break;
    case 14:
    lineFollowUntilPuck();
    delay(50);
    lineFollowCornerPuck();
    delay(1000);
    goRightUntilFindBlackLine();
    goForwardUntilPuckInside2();   
    funnel(-70, 60);
    delay(50);
    lineFollowUntilGoal();
    delay(50);
    goForwardALittleBit();
    feedAndShoot();
    break;

    case 15:
    lineFollowUntilGoal();
    delay(50);
    goForwardALittleBit();
    feedAndShoot();
  break;
    case 16:
    feedAndShoot2();
    break;
  }
  

}

void goForwardUntilPuckInside(){
  bool stopLoop = false;
  uint16_t range = sensor.readRangeSingleMillimeters();

  while (!stopLoop) {
    if(range < 50){
       leftMotor.setSpeed(150);
       rightMotor.setSpeed(150); 
       delay(300);
       leftMotor.setSpeed(0);
       rightMotor.setSpeed(0); 
    } else{
      stopLoop = true;
      leftMotor.setSpeed(150);
      rightMotor.setSpeed(150); 
      delay(200);
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0); 
    }
    range = sensor.readRangeSingleMillimeters();
  }
}

void goForwardUntilPuckInside2(){
  bool stopLoop = false;
  uint16_t range = sensor.readRangeSingleMillimeters();

  while (!stopLoop) {
    if(range < 50){
       leftMotor.setSpeed(150);
       rightMotor.setSpeed(150); 
       delay(100);
       leftMotor.setSpeed(0);
       rightMotor.setSpeed(0); 
    } else{
      stopLoop = true;
      leftMotor.setSpeed(150);
      rightMotor.setSpeed(150); 
      delay(100);
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0); 
    }
    range = sensor.readRangeSingleMillimeters();
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
