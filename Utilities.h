#pragma once
#include <Arduino.h>

class Utilities
{
    public:
        static size_t indexCap(size_t index, size_t maxIndex);
    
    private:
        Utilities();
};