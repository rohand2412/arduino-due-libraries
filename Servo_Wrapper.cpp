/**
 * Limits range of motion of a servo
 * Copyright (C) 2022  Rohan Dugad
 * 
 * Contact info:
 * https://docs.google.com/document/d/17IhBs4cz7FXphE0praCaWMjz016a7BFU5IQbm1CNnUc/edit?usp=sharing
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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