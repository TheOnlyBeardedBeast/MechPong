#pragma once

#include <StepDirection.hpp>
#include <Arduino.h>

StepDirection diffToDirection(int16_t diff)
{
    if (diff < 0)
    {
        return StepDirection::DIRECTION_CCW;
    }


    return StepDirection::DIRECTION_CW;
};

uint16_t calculateRampSteps(uint32_t speedPower, uint32_t edgeSpeedPower, uint32_t acceleration2)
{
    return static_cast<uint16_t>((speedPower - edgeSpeedPower) / acceleration2);
}

byte dirToPin(StepDirection dir)
{
    if (dir == StepDirection::DIRECTION_CW)
    {
        return HIGH;
    }

    return LOW;
}

uint32_t calculateAccelerationSpeed(uint32_t speedPower, uint32_t edgeSpeed, uint32_t acceleration2)
{
    return max(sqrt(acceleration2 + speedPower), edgeSpeed*edgeSpeed);
}

uint32_t calculateDeccelerationSpeed(uint32_t speedPower, uint32_t edgeSpeed, uint32_t acceleration2)
{
    return max(sqrt(speedPower - acceleration2), edgeSpeed*edgeSpeed);
}