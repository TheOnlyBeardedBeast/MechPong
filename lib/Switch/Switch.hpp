#pragma once

class Switch {
    private:
        const int pin;

    public:
        explicit Switch(int pinNumber);
        bool isClicked() const;
};