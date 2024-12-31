#include <Ball.hpp>

PongStepper xStepper;
PongStepper yStepper;

#if defined(ARDUINO_GIGA)
Portenta_H7_Timer xyTimer(TIM15);
#elif defined(ARDUINO_SAM_DUE)
#include "DueTimer.h"
DueTimer ITimerX(3);
DueTimer ITimerY(4);
#endif

#ifdef ARDUINO_ARCH_RP2040
#include "RPi_Pico_TimerInterrupt.h"
RPI_PICO_Timer ITimerX(3);
RPI_PICO_Timer ITimerY(4);
#endif

extern Ball ball;

void setupBall()
{
    // Motor A and B STEP and DIRECTION pins
    pinMode(BALLSTEPPER_A_STP, OUTPUT);
    pinMode(BALLSTEPPER_A_DIR, OUTPUT);
    pinMode(BALLSTEPPER_B_STP, OUTPUT);
    pinMode(BALLSTEPPER_B_DIR, OUTPUT);

    // Limit switches for X and Y axes
    pinMode(LS1, INPUT_PULLUP);
    pinMode(LS2, INPUT_PULLUP);
    pinMode(LS3, INPUT_PULLUP);
    pinMode(LS4, INPUT_PULLUP);

    xStepper.init(BALLSTEPPER_A_STP, BALLSTEPPER_A_DIR, BALLSTEPPER_B_STP, BALLSTEPPER_B_DIR);
    xStepper.setTimer(&xyTimer);

    yStepper.init(BALLSTEPPER_A_STP, BALLSTEPPER_A_DIR, BALLSTEPPER_B_STP, BALLSTEPPER_B_DIR);
    yStepper.setTimer(&xyTimer);

    ball.setMotors(&xyStepper);
    Ball::instance = &ball;
}