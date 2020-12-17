#include "Servo_Wrapper.h"

Servo_Wrapper::Servo_Wrapper(unsigned int lowerBound /*= 0*/,
                             unsigned int upperBound /*= 180*/) : 
                             Servo(), 
                             _LOWER_BOUND(lowerBound), 
                             _UPPER_BOUND(upperBound)
{
    _angle = 0xFFFFFFFF;    //_angle = -1
};

void Servo_Wrapper::write(unsigned int angle)
{
    _angle = angle;
    angle += _LOWER_BOUND;                                  //Translate angle to bound range
    angle = (angle > _UPPER_BOUND) ? _UPPER_BOUND : angle;  //Cap angle at _upperBound
    Servo::write(angle);                                    //Pass angle to Servo::write()
}

unsigned int Servo_Wrapper::read() const
{
    //Need to pass RGB LED object to class when RGB class completed
    //Flash Red if read is called before _angle is initialized
    //Until then Serial Port will be only form of indication
    if (_angle == 0xFFFFFFFF)   //_angle == -1
    {
        while(true)
        {
            Serial.print("Servo_Wrapper::read() called before ");
            Serial.println("initialization of Servo_Wrapper::write()");
        }
    }
    return _angle;
}

unsigned int Servo_Wrapper::getLowerBound() const
{
    return _LOWER_BOUND;
}

unsigned int Servo_Wrapper::getUpperBound() const
{
    return _UPPER_BOUND;
}