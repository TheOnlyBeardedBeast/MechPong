#include "Switch.hpp"
#include "Arduino.h"

Switch::Switch(int pinNumber) : pin(pinNumber)
{
    pinMode(pin,INPUT_PULLUP);

    // PI Pico sdk
    // gpio_init(pin);
    // gpio_set_dir(pin, GPIO_IN);
    // gpio_pull_up(pin);
}

bool Switch::isClicked() const
{
    // return (sio_hw->gpio_in & (1 << this->pin)) == 0;
    return digitalRead(this->pin) == LOW;
}
