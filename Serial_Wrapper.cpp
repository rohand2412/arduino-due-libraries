#include <Arduino.h>
#include "Serial_Wrapper.h"

UARTClass Serial_Wrapper::_port = Serial;

size_t Serial_Wrapper::_itemNum = 0;

const uint8_t Serial_Wrapper::_CRC_CALCULATOR[0x20] =
    {0,
     3, 6, 5, 7, 4, 1, 2, 5, 6, 3,
     0, 2, 1, 4, 7, 1, 2, 7, 4, 6,
     5, 0, 3, 4, 7, 2, 1, 3, 0, 5,
     6};

Serial_Wrapper::_State Serial_Wrapper::_state = Serial_Wrapper::_State::INIT;

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

void Serial_Wrapper::send(const uint8_t* buffer, size_t bufferLen, UARTClass& port /*= _port*/)
{
    port.write(_doCRC(_DELIMITER_BYTE));
    for (size_t item = 0; item < bufferLen; item++)
    {
        if (buffer[item] > 0x1f)
        {
            if (!Serial)
            {
                begin(115200, Serial);
            }
            while (true)
            {
                Serial.println("[ERROR] TRYING TO SEND BYTE BIGGER THAN 0x1f");
            }
        }
        else
        {
            if (buffer[item] == _DELIMITER_BYTE || buffer[item] == _ESCAPE_BYTE)
            {
                port.write(_doCRC(_ESCAPE_BYTE));
                port.write(_doCRC(_escape(buffer[item])));
            }
            else
            {
                port.write(_doCRC(buffer[item]));
            }
        }
    }
    port.write(_doCRC(_DELIMITER_BYTE));
}

size_t Serial_Wrapper::receive(uint8_t* buffer, size_t bufferLen, UARTClass& port /*= _port*/)
{
    uint8_t message;
    while (port.available())
    {
        message = _undoCRC(port.read());
        if (message != 0xFF) //!= -1
        {
            if (_receiveSM(buffer, &_itemNum, bufferLen, message))
            {
                if (_itemNum == 0)
                {
                    continue;
                }

                size_t itemNum = _itemNum;
                _itemNum = 0;
                return itemNum;
            }
        }
        else
        {
            _state = _State::INIT;
            _itemNum = 0;
        }
    }

    return 0xFFFFFFFF; //return -1
}

bool Serial_Wrapper::_receiveSM(uint8_t *buffer, size_t *itemNum, size_t bufferLen, uint8_t byte_in)
{
    switch (_state)
    {
        case _State::INIT:
            if (byte_in == _DELIMITER_BYTE)
            {
                _state = _State::NORMAL;
            }
            return false;

        case _State::NORMAL:
            if (byte_in == _DELIMITER_BYTE)
            {
                return true;
            }
            if (byte_in == _ESCAPE_BYTE)
            {
                _state = _State::ESCAPE;
                return false;
            }
            buffer[(*itemNum)++] = byte_in;
            return false;
        
        case _State::ESCAPE:
            buffer[(*itemNum)++] = _unescape(byte_in);
            _state = _State::NORMAL;
            return false;
    }
}

uint8_t Serial_Wrapper::_doCRC(uint8_t message)
{
    return message * 8 + _CRC_CALCULATOR[message];
}

uint8_t Serial_Wrapper::_undoCRC(uint8_t crc_byte)
{
    const uint8_t message = (crc_byte & 0xf8) / 8;
    const uint8_t crc = crc_byte & 0x07;

    if (_CRC_CALCULATOR[message] == crc)
    {
        return message;
    }
    else
    {
        if (!Serial)
        {
            begin(115200, Serial);
        }
        Serial.println("[WARNING] CORRUPT BYTE DETECTED");

        return 0xFF; //return -1
    }
}

uint8_t Serial_Wrapper::_escape(uint8_t raw)
{
    return raw ^ _CONVERSION;
}

uint8_t Serial_Wrapper::_unescape(uint8_t escaped)
{
    return escaped ^ _CONVERSION;
}