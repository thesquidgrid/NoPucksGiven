#include "MC33926MotorShield.h"

// Constructors ////////////////////////////////////////////////////////////////

MC33926MotorShield::MC33926MotorShield(
    unsigned char _In1,
    unsigned char _In2,
    unsigned char _D1,
    unsigned char _nSF,
    unsigned char _En
): In1(_In1), In2(_In2), D1(_D1), nSF(_nSF), En(_En) {
}

// Public Methods //////////////////////////////////////////////////////////////
void MC33926MotorShield::begin() {
    // Define pinMode for the pins and set the frequency for timer1.
    pinMode(In1, OUTPUT);
    pinMode(In2, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(nSF, INPUT_PULLUP);
    pinMode(En, OUTPUT);
    digitalWrite(En, HIGH);
    
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void MC33926MotorShield::setSpeed(int speed) {
    if(speed < 0) {
        speed = -speed;  // Make speed a positive quantity
        digitalWrite(In1, HIGH); 
        digitalWrite(In2, LOW); 
    }
    else {
        digitalWrite(In1, LOW); 
        digitalWrite(In2, HIGH); 
    }

    if(speed > 255) {
        speed = 255;
    }

    analogWrite(D1, speed); }

// Return error status
unsigned char MC33926MotorShield::getFault() {
    return !digitalRead(nSF);
}
