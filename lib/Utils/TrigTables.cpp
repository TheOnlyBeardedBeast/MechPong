#include <TrigTables.hpp>
#include "Configuration.hpp"

float sinTable[ANGLE_COUNT];
float cosTable[ANGLE_COUNT];
float tanTable[ANGLE_COUNT];

void initTrigTables() {
    for (int i = 0; i < ANGLE_COUNT; ++i) {
        float rad = (MIN_ANGLE + i * ANGLE_STEP) * DEG_RAD;
        sinTable[i] = sinf(rad);
        if(i==12){
            cosTable[i] = 0;
            tanTable[i] = 0;
        } else {
            cosTable[i] = cosf(rad);
            tanTable[i] = tanf(rad);
        }
    }
}