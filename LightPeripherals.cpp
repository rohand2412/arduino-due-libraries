#include <Arduino.h>
#include "LightPeripherals.h"

LED::LED(unsigned int pin)
{
    _pin = pin;
}

void LED::begin()
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    _state = false;
}

void LED::setState(bool state)
{
    digitalWrite(_pin, state);
    _state = state;
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