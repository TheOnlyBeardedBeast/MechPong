#include <TrigTables.hpp>
#include "Configuration.hpp"

float sinTable[ANGLE_COUNT];
float cosTable[ANGLE_COUNT];

void initTrigTables() {
    for (int i = 0; i < ANGLE_COUNT; ++i) {
        float rad = (ANGLE_MIN + i * ANGLE_STEP) * DEG_RAD;
        sinTable[i] = sin(rad);
        if(i==12){
            cosTable[i] = 0;
        } else {
            cosTable[i] = cos(rad);
        }
    }
}