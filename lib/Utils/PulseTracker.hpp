#pragma once
#include <Arduino.h>

class PulseTracker {
public:
    PulseTracker() 
        : pulseIdx(0), last_pulse_time(0), pulseSum(0) 
    {
        for (int i = 0; i < 16; ++i) pulse_history[i] = 0;
    }

    // Call this to update last pulse time
    void setPulseTime(uint32_t now) {
        last_pulse_time = now;
    }

    // Call this when a new pulse arrives; returns moving average
    uint16_t addPulse(uint32_t dt) {
        // Remove old value from sum
        pulseSum -= pulse_history[pulseIdx];

        // Store new value
        pulse_history[pulseIdx] = dt;
        pulseSum += dt;

        // Increment index (wrap around 16)
        pulseIdx = (pulseIdx + 1) & 15;

        // Return 16-sample average
        return pulseSum >> 4; // divide by 16
    }

    uint32_t getLastPulseTime()
    {
        return this->last_pulse_time;
    }

private:
    uint32_t pulse_history[16];
    uint8_t pulseIdx;
    uint32_t last_pulse_time;
    uint32_t pulseSum;
};
