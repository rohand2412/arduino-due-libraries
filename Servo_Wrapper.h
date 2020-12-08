#pragma once
#include <Arduino.h>
#include "Servo.h"

class Servo_Wrapper : public Servo
{
    private:
        int _lowerBound;
        int _upperBound;
    
    public:
        Servo_Wrapper();

        void write(int value);

        void writeMicroseconds(int value) = delete; //Only method to set degree should be write()

        int readMicroseconds() = delete;    //Unnecessary without writeMicroseconds()

        void setBounds(int lowerBound, int upperBound);

        int getLowerBound() const;

        int getUpperBound() const;
};