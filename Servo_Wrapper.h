#pragma once
#include <Arduino.h>
#include "Servo.h"

class Servo_Wrapper : public Servo
{
    private:
        const unsigned int _LOWER_BOUND;
        const unsigned int _UPPER_BOUND;
    
    public:
        Servo_Wrapper(unsigned int lowerBound = 0, unsigned int upperBound = 180);

        void write(unsigned int value);

        void writeMicroseconds(int value) = delete; //Only method to set degree should be write()

        int readMicroseconds() = delete;    //Unnecessary without writeMicroseconds()

        unsigned int getLowerBound() const;

        unsigned int getUpperBound() const;
};