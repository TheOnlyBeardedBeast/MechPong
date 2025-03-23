#pragma once
#include <Arduino.h>

constexpr int ANGLE_COUNT = 25;  // Angles: 30, 35, 40, ..., 150
constexpr int ANGLE_MIN = 30;
constexpr int ANGLE_STEP = 5;

extern float sinTable[ANGLE_COUNT];
extern float cosTable[ANGLE_COUNT];

// Precompute sine and cosine values for 30° to 150°
void initTrigTables();

// Lookup sine and cosine, handling 180° shift
inline float fastSin(int degrees) {
    bool isFlipped = (degrees >= 180);  // If in the 210°-330° range

    if (isFlipped) degrees -= 180;

    int index = (degrees - ANGLE_MIN) / ANGLE_STEP;
    return isFlipped ? -sinTable[index] : sinTable[index];
}

inline float fastCos(int degrees) {
    bool isFlipped = (degrees > 180);  // If in the 210°-330° range

    if (isFlipped) degrees -= 180;

    int index = (degrees - ANGLE_MIN) / ANGLE_STEP;
    return isFlipped ? -cosTable[index] : cosTable[index];
}
