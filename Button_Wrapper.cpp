#include <Arduino.h>
#include "Button_Wrapper.h"
#include "Button.h"

Button_Wrapper::Button_Wrapper(uint8_t pin, uint8_t mode /*= BUTTON_SWITCH*/,
                               unsigned long debounceInterval /*= 30*/) :
                               _button(pin, debounceInterval)
{
    //Store mode
    _mode = mode;

    //Button is off
    _switchState = false;

    //Check if mode is inversed
    if (_mode == BUTTON_INVERSE_SWITCH || _mode == BUTTON_INVERSE_RAW)
    {
        //Initial state of button is released
        _buttonState = true;

        //true because state is inverse
        //of typical released state
    }

    else
    {
        //Initial state of button is released
        _buttonState = false;
    }
}

void Button_Wrapper::begin(void (*externalPinISR)())
{
    //Configure button 
    _button.begin(externalPinISR);
}

void Button_Wrapper::update()
{
    //Update button
    _button.update();

    //Select mode
    switch (_mode)
    {
        case BUTTON_SWITCH:
            //Check if button state has risen
            if (_button.getState() > _buttonState)
            {
                //Flip switch state
                _switchState = !_switchState;
            }

            //Update buttonState with new state
            _buttonState = _button.getState();

            break;

        case BUTTON_INVERSE_SWITCH:
            //Check if button state has fallen
            if (_button.getState() < _buttonState)
            {
                //Flip switch state
                _switchState = !_switchState;
            }

            //Update buttonState with new state
            _buttonState = _button.getState();

            break;

        case BUTTON_RAW:
            //Update buttonState with new state
            _buttonState = _button.getState();

            break;

        case BUTTON_INVERSE_RAW:
            //Update buttonState with new inversed state
            _buttonState = !_button.getState();

            break;
    }
}

void Button_Wrapper::setMode(uint8_t mode)
{
    //Check if mode was inversed
    //or if conversion from switch to raw was inversed
    if ((_mode == BUTTON_INVERSE_RAW && mode == BUTTON_RAW)
        || (_mode == BUTTON_RAW && mode == BUTTON_INVERSE_RAW)
        || (_mode == BUTTON_INVERSE_SWITCH && mode == BUTTON_SWITCH)
        || (_mode == BUTTON_SWITCH && mode == BUTTON_INVERSE_SWITCH)
        || (_mode == BUTTON_INVERSE_SWITCH && mode == BUTTON_RAW)
        || (_mode == BUTTON_SWITCH && mode == BUTTON_INVERSE_RAW))
    {
        //Inverse _buttonState
        _buttonState = !_buttonState;
    }
    //Check if mode was changed from raw to switch
    else if ((_mode == BUTTON_RAW && mode == BUTTON_SWITCH)
             || (_mode == BUTTON_INVERSE_RAW && mode == BUTTON_INVERSE_SWITCH))
    {
        _switchState = false;
    }
    //Check if conversionfrom raw to switch was inversed
    else if ((_mode == BUTTON_INVERSE_RAW && mode == BUTTON_SWITCH)
             || (_mode == BUTTON_RAW && mode == BUTTON_INVERSE_SWITCH))
    {
        _buttonState = !_buttonState;
        _switchState = false;
    }

    //Update _mode with new mdoe
    _mode = mode;
}

uint8_t Button_Wrapper::getMode() const
{
    //Return mode
    return _mode;
}

void Button_Wrapper::setState(bool state)
{
    //Set new state to internal state
    _switchState = state;
}

bool Button_Wrapper::getState() const
{
    //Check if mode is a type of switch
    if (_mode == BUTTON_SWITCH || _mode == BUTTON_INVERSE_SWITCH)
    {
        //Return switch state
        return _switchState;
    }

    //mode is a type of raw

    //Return button state
    return _buttonState;
}

uint8_t Button_Wrapper::getPin() const
{
    //Return pin
    return _button.getPin();
}

void Button_Wrapper::pinISR()
{
    //Run ISR
    _button.pinISR();
}