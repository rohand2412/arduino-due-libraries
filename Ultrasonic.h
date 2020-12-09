#pragma once
#include <Arduino.h>

class Ultrasonic
{
    private:
        const unsigned int 
            _echoPin, 
            _trigPin,
            _burstFrequencyMS;
        
        unsigned int
            _burstMS,
            _roundTripTime,
            _distance,      //in mm
            _startMS;

    public:
        Ultrasonic(unsigned int echoPin, 
                   unsigned int trigPin, 
                   unsigned int burstFrequencyMS);

        void begin(void (*externalEchoPinISR)());

        void update();

        unsigned int getDistance() const;

        void echoPinISR();
};