#include <Arduino.h>
#include "Serial_Wrapper.h"

UARTClass Serial_Wrapper::_port = Serial;

Serial_Wrapper::Serial_Wrapper(){};

void Serial_Wrapper::begin(uint32_t baudRate, UARTClass& port)
{
    if (baudRate > 115200 && baudRate != 750000)
    {
        Serial.begin(115200);
        while (true)
        {
            Serial.println("[ERROR] UART_BRGR NOT SPECIFIED");
        }
    }

    port.begin(baudRate);

    if (baudRate == 750000)
    {
        UART->UART_BRGR = 8;
    }

    while (!port)
        ;
}

void Serial_Wrapper::setDefault(const UARTClass& port)
{
    _port = port;
}