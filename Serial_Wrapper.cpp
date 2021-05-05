#include <Arduino.h>
#include "Serial_Wrapper.h"

//Initialize static variables
UARTClass Serial_Wrapper::_port = Serial;

size_t Serial_Wrapper::_itemNum = 0;

const uint8_t Serial_Wrapper::_CRC_CALCULATOR[0x20] =
    {0,
     3, 6, 5, 7, 4, 1, 2, 5, 6, 3,
     0, 2, 1, 4, 7, 1, 2, 7, 4, 6,
     5, 0, 3, 4, 7, 2, 1, 3, 0, 5,
     6};

Serial_Wrapper::_State Serial_Wrapper::_state = Serial_Wrapper::_State::INIT;

//Private empty constructor
Serial_Wrapper::Serial_Wrapper(){};

void Serial_Wrapper::begin(uint32_t baudRate, UARTClass& port)
{
    //Check if baudrate does not have corresponding
    //predefined UART_BRGR
    if (baudRate > 460800 && baudRate != 750000)
    {
        //Start Serial
        begin(750000, Serial);

        //Make user aware of error
        while (true)
        {
            Serial.println("[ERROR] UART_BRGR NOT SPECIFIED");
        }
    }

    //Start port with baudrate
    port.begin(baudRate);

    //Check if baudrate is 750000
    if (baudRate == 750000)
    {
        //Set register to 8
        UART->UART_BRGR = 8;
    }

    //Wait for port to be initialized
    while (!port)
        ;
}

void Serial_Wrapper::setDefault(const UARTClass& port)
{
    //Save port as default
    _port = port;
}

void Serial_Wrapper::send(const int32_t* buffer, size_t bufferLen, UARTClass& port /*= _port*/)
{
    //Start packet with delimiter byte
    port.write(_doCRC(_PACKET_DELIMITER_BYTE));

    //Iterate through packet items
    for (size_t item = 0; item < bufferLen; item++)
    {
        //Temporary unsigned copy of the current item
        uint32_t itemByte = buffer[item];

        //Array of the bytes that item would be split into
        uint8_t itemBytes[_MAX_ITEM_BYTES];

        //Number of bytes that the item would be split into
        size_t bytes = 0;

        //Split item into groups of five bits
        for (bytes; bytes < _MAX_ITEM_BYTES; bytes++)
        {
            //Save least significant 5 bits
            itemBytes[bytes] = itemByte & 0x1F;

            //Shift copy 5 bits to the right
            itemByte = itemByte >> _ITEM_BIT_LEN;
        }

        //Reduced to largest index instead of number of bytes
        bytes--;

        //Iterate down from the largest index
        for (bytes; bytes > 0; bytes--)
        {
            //Check if current group of five bits is not empty
            if (itemBytes[bytes] != 0)
            {
                //Stop short as soon as we hit actual bit data
                break;
            }
        }

        //Send bytes from msb to lsb
        for (size_t byteIndex = bytes; byteIndex < _MAX_ITEM_BYTES; byteIndex--)
        {
            //Write individual byte to the port
            _write(itemBytes[byteIndex], port);
        }

        //Complete item exchange with item delimiter
        port.write(_doCRC(_ITEM_DELIMITER_BYTE));
    }

    //End packet with delimeter byte
    port.write(_doCRC(_PACKET_DELIMITER_BYTE));
}

size_t Serial_Wrapper::receive(int32_t* buffer, size_t bufferLen, UARTClass& port /*= _port*/)
{
    //Allocate memory for incoming message
    uint8_t message;

    //Loop till there are no more bytes waiting to be read
    while (port.available())
    {
        //Undo CRC to get message
        message = _undoCRC(port.read());

        //Check if byte was not corrupt
        if (message != 0xFF) //!= -1
        {
            //See if state machine has received a full packet
            //that it can return
            if (_receiveSM(buffer, &_itemNum, bufferLen, message))
            {
                //Check if state machine returned packet with zero length
                if (_itemNum == 0)
                {
                    //Start of new packet so clear buffer
                    for (size_t item = 0; item < bufferLen; item++)
                    {
                        //Reset each item to 0
                        buffer[item] = 0;
                    }

                    //Continue looping for incoming packet
                    continue;
                }

                //Temporarily save _itemNum
                size_t itemNum = _itemNum;

                //Reset _itemNum
                _itemNum = 0;

                //Return stored value
                return itemNum;
            }
        }
        //Byte was corrupt
        else
        {
            //Reset packet reading
            _state = _State::INIT;
            _itemNum = 0;
        }
    }

    //Return -1 if packet has been completed yet
    return 0xFFFFFFFF; //return -1
}

void Serial_Wrapper::_write(uint8_t item, UARTClass& port)
{
    //Check if item is a delimeter or escape byte
    if (item == _PACKET_DELIMITER_BYTE || item == _ITEM_DELIMITER_BYTE || item == _ESCAPE_BYTE)
    {
        //Write escape byte
        port.write(_doCRC(_ESCAPE_BYTE));

        //Escape item and then write it
        port.write(_doCRC(_escape(item)));
    }
    //Item doesn't conflict
    else
    {
        //Write item
        port.write(_doCRC(item));
    }
}

bool Serial_Wrapper::_receiveSM(int32_t *buffer, size_t *itemNum, size_t bufferLen, uint8_t byte_in)
{
    //Switch on state of state machine
    switch (_state)
    {
        case _State::INIT:
            //Check for packet start or end
            if (byte_in == _PACKET_DELIMITER_BYTE)
            {
                //Switch state to NORMAL
                _state = _State::NORMAL;
            }

            //Packet hasn't been completed yet
            return false;

        case _State::NORMAL:
            //Check for packet start or end
            if (byte_in == _PACKET_DELIMITER_BYTE)
            {
                //Assume it is an end
                //and return true
                return true;
            }

            //Check for item end
            if (byte_in == _ITEM_DELIMITER_BYTE)
            {
                //Increment item index of buffer
                (*itemNum)++;

                //Packet is still not complete
                return false;
            }

            //Check if next byte needs to be unescaped
            if (byte_in == _ESCAPE_BYTE)
            {
                //Switch state to ESCAPE 
                _state = _State::ESCAPE;

                //Packet hasn't been completed yet
                return false;
            }

            //Check if itemNum exceeds array length
            if ((*itemNum) >= bufferLen)
            {
                //Check if Serial has already been started
                if (!Serial)
                {
                    //Start serial at 750000 baud
                    begin(750000, Serial);
                }

                //Endlessly repeat
                while (true)
                {
                    //Spam user with error messages
                    Serial.println("[ERROR] PACKET LENGTH OVERFLOW! PLEASE ALLOCATE MORE MEMORY!");
                }
            }

            //If made it here, byte is just regular item in packet
            
            //Store item in buffer
            buffer[(*itemNum)] = buffer[(*itemNum)] << _ITEM_BIT_LEN;
            buffer[(*itemNum)] += byte_in;

            //Packet hasn't been completed yet
            return false;
        
        case _State::ESCAPE:
            //Store unescaped byte
            buffer[(*itemNum)] = buffer[(*itemNum)] << _ITEM_BIT_LEN;
            buffer[(*itemNum)] += _unescape(byte_in);

            //Switch state back to NORMAL
            _state = _State::NORMAL;

            //Packet hasn't been completed yet
            return false;
    }
}

uint8_t Serial_Wrapper::_doCRC(uint8_t message)
{
    //Shift message by 3 bits
    //and then fill the three bits with a CRC
    return message * 8 + _CRC_CALCULATOR[message];
}

uint8_t Serial_Wrapper::_undoCRC(uint8_t crc_byte)
{
    //Mask last three bits and then shift by three bits
    const uint8_t message = (crc_byte & 0xf8) / 8;

    //Mask first five bits to be left with last three bits
    const uint8_t crc = crc_byte & 0x07;

    //Check if calculated crc matches crc received
    if (_CRC_CALCULATOR[message] == crc)
    {
        //Message was not corrupt
        return message;
    }
    //Message was corrupt
    else
    {
        //Start Serial if not started already
        if (!Serial)
        {
            begin(750000, Serial);
        }

        //Make user aware of corrupted byte
        Serial.println("[WARNING] CORRUPT BYTE DETECTED");

        //Indicate byte was corrupt
        return 0xFF; //return -1
    }
}

uint8_t Serial_Wrapper::_escape(uint8_t raw)
{
    //XOR with coversion mask
    return raw ^ _CONVERSION;
}

uint8_t Serial_Wrapper::_unescape(uint8_t escaped)
{
    //XOR with conversion mask
    return escaped ^ _CONVERSION;
}