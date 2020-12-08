#include "Servo_Wrapper.h"

Servo_Wrapper::Servo_Wrapper() : Servo()
{
    _lowerBound = 0;
    _upperBound = 180;
}

void Servo_Wrapper::write(int value)
{
    value = (value < 0) ? 0 : value;                        //Make sure its positive
    value += _lowerBound;                                   //Translate angle to bound range
    value = (value > _upperBound) ? _upperBound : value;    //Cap value at _upperBound
    Servo::write(value);                                    //Pass value to Servo::write()
}

void Servo_Wrapper::setBounds(int lowerBound, int upperBound)
{
    _lowerBound = lowerBound;
    _upperBound = upperBound;
}

int Servo_Wrapper::getLowerBound() const
{
    return _lowerBound;
}

int Servo_Wrapper::getUpperBound() const
{
    return _upperBound;
}