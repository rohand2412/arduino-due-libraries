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