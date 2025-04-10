#pragma once

#include <Arduino.h>

#include "TrigTables.hpp"

uint16_t getRelativeAngle(uint16_t angle) {
    if(angle > 180) {
        angle -= 180;
    }

    return abs(90 - angle);
}