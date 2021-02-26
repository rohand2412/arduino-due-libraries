#include <Arduino.h>
#include "Button.h"

Button::Button(uint8_t pin, unsigned long debounceInterval /*= 30*/) : _pin(pin), _debounceInterval(debounceInterval)
{
    _lastInterruptTime = millis() - _debounceInterval;
}

void Button::begin(void (*externalPinISR)())
{
    pinMode(_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pin), externalPinISR, CHANGE);
}

void Button::update()
{
    bool rawState = digitalRead(_pin);

    if (millis() - _lastInterruptTime > _debounceInterval)
    {
        _state = rawState;
    }
}

bool Button::getState() const
{
    return _state;
}

uint8_t Button::getPin() const
{
    return _pin;
}

void Button::pinISR()
{
    _lastInterruptTime = millis();
}