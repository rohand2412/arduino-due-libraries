#pragma once
#include <Arduino.h>

class LED
{
    private:
        unsigned int _pin;

        bool _state = false;

        uint8_t _pwm = 0;

    public:
        LED(unsigned int pin);

        void begin();

        void setPwm(uint8_t pwm);

        uint8_t getPwm() const;

        void setState(bool state);

        bool getState() const;

        void toggle();

        void on();

        void off();

    private:
        void _update();
};

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