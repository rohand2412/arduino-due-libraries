/**
 * Wraps an LED with convenient methods
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

#include <Arduino.h>
#include "LED.h"

LED::LED(unsigned int pin)
{
    _pin = pin;
}

void LED::begin()
{
    pinMode(_pin, OUTPUT);

    _update();
}

void LED::setPwm(uint8_t pwm)
{
    _pwm = pwm;

    _update();
}

uint8_t LED::getPwm() const
{
    return _pwm;
}

void LED::setState(bool state)
{
    _state = state;

    _update();
}

bool LED::getState() const
{
    return _state;
}

void LED::toggle()
{
    setState(!_state);
}

void LED::on()
{
    setState(true);
}

void LED::off()
{
    setState(false);
}

void LED::_update()
{
    if (_state && _pwm)
    {
        analogWrite(_pin, _pwm);
    }
    else
    {
        analogWrite(_pin, 0);
    }
}