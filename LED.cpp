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