#pragma once
#include <Arduino.h>
#include "Servo.h"

class Servo_Wrapper : public Servo
{
    private:
        const volatile unsigned int _LOWER_BOUND;
        const volatile unsigned int _UPPER_BOUND;
    
    public:
        Servo_Wrapper();

        void write(unsigned int value);

        void writeMicroseconds(int value) = delete; //Only method to set degree should be write()

        int readMicroseconds() = delete;    //Unnecessary without writeMicroseconds()

        void setBounds(unsigned int lowerBound, unsigned int upperBound);

        unsigned int getLowerBound() const;

        unsigned int getUpperBound() const;
};