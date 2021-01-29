#pragma once
#include <Arduino.h>
#include "Servo.h"

class Servo_Wrapper : public Servo
{
    private:
        const unsigned int _LOWER_BOUND;
        const unsigned int _UPPER_BOUND;

        unsigned int _angle;    //in degrees
    
    public:
        static const unsigned int SERVO_S1 = 10; //Servo pin for s1 servo
        static const unsigned int SERVO_S2 = 9; //Servo pin for s2 servo

    public:
        Servo_Wrapper(unsigned int lowerBound = 0, unsigned int upperBound = 180);

        void write(unsigned int angle);

        void writeMicroseconds(int value) = delete; //Only method to set degree should be write()

        unsigned int read() const;

        int readMicroseconds() = delete;    //Unnecessary without writeMicroseconds()

        unsigned int getLowerBound() const;

        unsigned int getUpperBound() const;
};