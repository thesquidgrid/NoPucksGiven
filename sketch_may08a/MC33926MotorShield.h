#pragma once

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || \
    defined(__AVR_ATmega328PB__) || defined (__AVR_ATmega32U4__)
#define DUALMC33926MOTORSHIELD_TIMER1_AVAILABLE
#endif

#include <Arduino.h>

class MC33926MotorShield {
    public:
        // CONSTRUCTORS
        // Default pin selection.
        MC33926MotorShield() = delete;
        // User-defined pin selection.
        MC33926MotorShield(unsigned char _In1,
                               unsigned char _In2,
                               unsigned char _D1,
                               unsigned char _nSF,
                               unsigned char _En);

        // PUBLIC METHODS
        void setSpeed(int speed); // Set speed for M1.
        unsigned char getFault(); // Get fault reading.
        void begin();
    private:
        unsigned char In1;
        unsigned char In2;
        unsigned char D1;
        unsigned char nSF;
        unsigned char En;
};
