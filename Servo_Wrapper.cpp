#include "Servo_Wrapper.h"

Servo_Wrapper::Servo_Wrapper() : Servo(), _LOWER_BOUND(0), _UPPER_BOUND(180){}; 

void Servo_Wrapper::write(unsigned int value)
{
    value += _LOWER_BOUND;                                  //Translate angle to bound range
    value = (value > _UPPER_BOUND) ? _UPPER_BOUND : value;  //Cap value at _upperBound
    Servo::write(value);                                    //Pass value to Servo::write()
}

void Servo_Wrapper::setBounds(unsigned int lowerBound, unsigned int upperBound)
{
    unsigned int& lb = const_cast <unsigned int &> (_LOWER_BOUND);
    lb = lowerBound;
    unsigned int &ub = const_cast <unsigned int &> (_UPPER_BOUND);
    ub = upperBound;
}

unsigned int Servo_Wrapper::getLowerBound() const
{
    return _LOWER_BOUND;
}

unsigned int Servo_Wrapper::getUpperBound() const
{
    return _UPPER_BOUND;
}