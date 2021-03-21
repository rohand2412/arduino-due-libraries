#include <Arduino.h>
#include "RGB_LED.h"

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