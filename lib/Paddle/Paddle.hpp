#pragma once

#include <StepDirection.hpp>
#include <PaddleStepper.hpp>
#include <PaddleSubscriber.hpp>

using CallbackFunction = void (*)(int);

class Paddle
{
public:
    // variables
    byte id;
    PaddleStepper *_stepper = NULL;

    // constructors
    Paddle();

    // methods
    void initializeEncoder(int A, int B);
    void initializeStepper(int step, int dir);
    byte canShoot(long x);

    void stop();
    void center();
    long getPosition();
    long getCenterRelativePosition();
    bool needsToMove();

    void setMaxSpeed(float speed);
    void setAcceleration(float acceleration);

    static Paddle *instances[2];
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

    byte stepIndex = 8;
private:
    int _pinA;
    int _pinB;
    bool isStopping = false;

    // methods
    int readA();
    int readB();
    void increment();
    void decrement();
    void changeTarget(bool direction);

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