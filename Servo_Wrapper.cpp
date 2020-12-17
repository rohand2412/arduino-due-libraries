#include "Servo_Wrapper.h"

Servo_Wrapper::Servo_Wrapper(unsigned int lowerBound /*= 0*/,
                             unsigned int upperBound /*= 180*/) : 
                             Servo(), 
                             _LOWER_BOUND(lowerBound), 
                             _UPPER_BOUND(upperBound)
{}; 

void Servo_Wrapper::write(unsigned int value)
{
    value += _LOWER_BOUND;                                  //Translate angle to bound range
    value = (value > _UPPER_BOUND) ? _UPPER_BOUND : value;  //Cap value at _upperBound
    Servo::write(value);                                    //Pass value to Servo::write()
}

unsigned int Servo_Wrapper::getLowerBound() const
{
    return _LOWER_BOUND;
}

unsigned int Servo_Wrapper::getUpperBound() const
{
    return _UPPER_BOUND;
}