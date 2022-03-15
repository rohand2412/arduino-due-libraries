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