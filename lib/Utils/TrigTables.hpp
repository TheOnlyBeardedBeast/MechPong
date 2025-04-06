#pragma once
#include <Arduino.h>
#include "Configuration.hpp"

extern float sinTable[ANGLE_COUNT];
extern float cosTable[ANGLE_COUNT];

// Precompute sine and cosine values for 30° to 150°
void initTrigTables();

// Lookup sine and cosine, handling 180° shift
inline float fastSin(int degrees) {
    bool isFlipped = (degrees >= 180);  // If in the 210°-330° range

    if (isFlipped) degrees -= 180;

    int index = (degrees - MIN_ANGLE) / ANGLE_STEP;
    return isFlipped ? -sinTable[index] : sinTable[index];
}

inline float fastCos(int degrees) {
    bool isFlipped = (degrees > 180);  // If in the 210°-330° range

    if (isFlipped) degrees -= 180;

    int index = (degrees - MIN_ANGLE) / ANGLE_STEP;
    return isFlipped ? -cosTable[index] : cosTable[index];
}
