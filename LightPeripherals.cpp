#include <Arduino.h>
#include "LightPeripherals.h"

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

RGB_LED::RGB_LED(unsigned int rPin, unsigned int bPin, unsigned int gPin)
    : _red(rPin), _blue(bPin), _green(gPin)
{}

void RGB_LED::begin()
{
    _red.begin();
    _blue.begin();
    _green.begin();
}

void RGB_LED::setState(bool rState, bool bState, bool gState)
{
    _red.setState(rState);
    _blue.setState(bState);
    _green.setState(gState);
}

void RGB_LED::toggle()
{
    _red.toggle();
    _blue.toggle();
    _green.toggle();
}

void RGB_LED::on()
{
    setState(true, true, true);
}

void RGB_LED::off()
{
    setState(false, false, false);
}