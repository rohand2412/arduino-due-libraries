/**
 * Button class optimized with interrupts
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
#include "Button.h"

Button::Button(uint8_t pin, unsigned long debounceInterval /*= 30*/) : _pin(pin), _debounceInterval(debounceInterval)
{
    //Initialized to number that guarantees entry into
    //if condition in update() on startup
    _lastInterruptTime = millis() - _debounceInterval;
}

void Button::begin(void (*externalPinISR)())
{
    //Set pin mode
    pinMode(_pin, INPUT);

    //Attached ISR
    attachInterrupt(digitalPinToInterrupt(_pin), externalPinISR, CHANGE);
}

void Button::update()
{
    //Store current raw state
    bool rawState = digitalRead(_pin);

    //Check if state hasn't changed within last interval time
    if (millis() - _lastInterruptTime > _debounceInterval)
    {
        //Assume raw state to be actual button state
        _state = rawState;
    }
}

bool Button::getState() const
{
    //Return state of button
    return _state;
}

uint8_t Button::getPin() const
{
    //Return pin of button
    return _pin;
}

void Button::pinISR()
{
    //Store time stamp of when raw button state changed
    _lastInterruptTime = millis();
}