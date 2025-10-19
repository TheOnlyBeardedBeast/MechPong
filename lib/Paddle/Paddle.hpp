#pragma once

#include <StepDirection.hpp>
#include <PaddleStepper.hpp>
#include <PaddleSubscriber.hpp>
#include <Switch.hpp>

using CallbackFunction = void (*)(int);

class Paddle
{
public:
    // variables
    byte id;
    PaddleStepper *_stepper = NULL;
    Switch *limitSwitch = NULL;

    // constructors
    Paddle();

    // methods
    void initializeEncoder(int A, int B);
    void initializeStepper(int step, int dir);
    byte tryShoot(long ballPos, long ballTarget, long possibleHitOffset);

    void stop();
    void center();
    long getPosition();
    long getCenterRelativePosition();
    bool needsToMove();

    void setMaxSpeed(float speed);
    void setAcceleration(float acceleration);
    long stepsToQuickStop();

    static Paddle *instances[2];
    static bool attached;
    static void attachPaddles();
    static void detachPaddles();
    static void isrReadEncoder0();
    static void isrReadEncoder01();
    static void isrReadEncoder10();
    static void isrReadEncoder11();
    static void calibrate();
    static bool calibrated;
    static void centerAll();
    static void run();
    static bool isRunning();
    static void update();
    static void quickStop();
    static void resetAcceleration();

    byte stepIndex = 4;
    long futureTarget = 0;
private:
    int _pinA;
    int _pinB;
    bool isStopping = false;

    // methods
    int readA();
    int readB();
    void increment();
    void decrement();
    uint8_t pulse_history = 0;
    uint32_t last_pulse_time = 0;

public:
    void subscribe(PaddleSubscriber *subscriber)
    {
        this->_stepper->subscribe(subscriber);
    }

    void unsubScribe()
    {
        this->_stepper->unsubscribe();
    }

    ~Paddle();
};