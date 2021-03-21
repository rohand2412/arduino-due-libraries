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

RGB_LED::RGB_LED(unsigned int rPin, unsigned int gPin, unsigned int bPin)
    : _red(rPin), _green(gPin), _blue(bPin)
{}

void RGB_LED::begin()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].begin();
    }
}

void RGB_LED::setPwm(uint8_t pwm, size_t led)
{
    _leds[led].setPwm(pwm);
}

void RGB_LED::setPwm(uint8_t rPwm, uint8_t gPwm, uint8_t bPwm)
{
    setPwm(rPwm, RED_INDEX);
    setPwm(gPwm, GREEN_INDEX);
    setPwm(bPwm, BLUE_INDEX);
}

uint8_t RGB_LED::getPwm(size_t led) const
{
    return _leds[led].getPwm();
}

void RGB_LED::setState(bool state)
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].setState(state);
    }
}

void RGB_LED::toggle()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].toggle();
    }
}

void RGB_LED::on()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].on();
    }
}

void RGB_LED::off()
{
    for (size_t led = 0; led < LED_NUM; led++)
    {
        _leds[led].off();
    }
}