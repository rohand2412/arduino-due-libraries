#include "Servo_Wrapper.h"

Servo_Wrapper::Servo_Wrapper(unsigned int lowerBound /*= 0*/,
                             unsigned int upperBound /*= 180*/) : 
                             Servo(), 
                             _LOWER_BOUND(lowerBound), 
                             _UPPER_BOUND(upperBound)
{
    _value = 0xFFFFFFFF;    //_value = -1
};

void Servo_Wrapper::write(unsigned int value)
{
    _value = value;
    value += _LOWER_BOUND;                                  //Translate angle to bound range
    value = (value > _UPPER_BOUND) ? _UPPER_BOUND : value;  //Cap value at _upperBound
    Servo::write(value); //Pass value to Servo::write()
}

unsigned int Servo_Wrapper::read() const
{
    //Need to pass RGB LED object to class when RGB class completed
    //Flash Red if read is called before _value is initialized
    //Until then Serial Port will be only form of indication
    if (_value == 0xFFFFFFFF)   //_value == -1
    {
        while(true)
        {
            Serial.print("Servo_Wrapper::read() called before ");
            Serial.println("initialization of Servo_Wrapper::write()");
        }
    }
    return _value;
}

unsigned int Servo_Wrapper::getLowerBound() const
{
    return _LOWER_BOUND;
}

unsigned int Servo_Wrapper::getUpperBound() const
{
    return _UPPER_BOUND;
}