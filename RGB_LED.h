#pragma once
#include "Arduino.h"
#include "LED.h"

class RGB_LED
{
    public:
        static const size_t LED_NUM = 3;
        static const size_t RED_INDEX = 0;
        static const size_t GREEN_INDEX = 1;
        static const size_t BLUE_INDEX = 2;

    private:
        LED _red;
        LED _green;
        LED _blue;

        LED _leds[LED_NUM] = {_red, _green, _blue};

    public:
        RGB_LED(unsigned int rPin, unsigned int gPin, unsigned int bPin);

        void begin();

        void setPwm(uint8_t pwm, size_t led);

        void setPwm(uint8_t rPwm, uint8_t gPwm, uint8_t bPwm);

        uint8_t getPwm(size_t led) const;

        void setState(bool state);

        void toggle();

        void on();

        void off();
};