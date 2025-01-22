#include "Paddle.hpp"
#include <Configuration.hpp>

Paddle *Paddle::instances[2];
bool Paddle::calibrated = false;

Paddle::Paddle()
{
}

void Paddle::initializeEncoder(int A, int B)
{
    pinMode(A, INPUT_PULLUP);
    pinMode(B, INPUT_PULLUP);

    this->_pinA = A;
    this->_pinB = B;
}

void Paddle::initializeStepper(int step, int dir)
{
    pinMode(step,OUTPUT);
    pinMode(dir,OUTPUT);

    this->_stepper = new PaddleStepper(step,dir);
    this->_stepper->setMaxSpeed(1200);
    this->_stepper->setAcceleration(3000);
}

void Paddle::center()
{
    this->_stepper->moveTo(PADDLE_CENTER);
}

void Paddle::stop()
{
    this->_stepper->stop();
}

int Paddle::readA()
{
    return digitalRead(this->_pinA);
    // return bitRead(this->_registerA, this->_bitMaskA);
}

int Paddle::readB()
{
    return digitalRead(this->_pinB);
    // return bitRead(this->_registerB, this->_bitMaskB);
}

void Paddle::increment()
{
    this->stepIndex+=0.5f;

    int delta = stepIndex > 0 ? floor(stepIndex) : ceil(stepIndex);

    if(delta==1){
        long target = constrain(this->_stepper->targetPosition() + 1, 0, PADDLE_LIMIT);
        this->_stepper->moveTo(target);
        this->stepIndex = 0;
    }
}

void Paddle::decrement()
{
    this->stepIndex-=0.5f;
    int delta = stepIndex > 0 ? floor(stepIndex) : ceil(stepIndex);
    if(delta == -1)
    {
        long target = constrain(this->_stepper->targetPosition() - 1, 0, PADDLE_LIMIT);
        this->_stepper->moveTo(target);
        this->stepIndex = 0;
    }
    
}

void Paddle::changeTarget(bool direction)
{
    // if (this->stepIndex == 0)
    // {
        if(direction)
        {
            this->increment();
        } else {
            this->decrement();
        }
    // }
    
    // this->stepIndex = (this->stepIndex + 1) % PADDLE_SENSITIVITY;
}

Paddle::~Paddle()
{
    delete this->_stepper;
}

long Paddle::getPosition()
{
    return this->_stepper->currentPosition();
}

long Paddle::getCenterRelativePosition()
{
    return this->getPosition() - PADDLE_CENTER;
}

bool Paddle::needsToMove()
{
    return this->_stepper->isRunning();
}

void Paddle::setMaxSpeed(float speed)
{
    this->_stepper->setMaxSpeed(speed);
}

void Paddle::setAcceleration(float acceleration)
{
    this->_stepper->setAcceleration(acceleration);
}

void Paddle::isrReadEncoder0()
{
    int a = Paddle::instances[0]->readA();
    int b = Paddle::instances[0]->readB();
    if(a != b)
    {
        Paddle::instances[0]->increment();
    } else {
        Paddle::instances[0]->decrement();
    }
}

void Paddle::isrReadEncoder01()
{

    int a = Paddle::instances[0]->readA();
    int b = Paddle::instances[0]->readB();
    if(a == b)
    {
        Paddle::instances[0]->increment();
    } else {
        Paddle::instances[0]->decrement();
    }
}

void Paddle::isrReadEncoder10()
{
    Paddle::instances[1]->changeTarget(!Paddle::instances[1]->readB());
}

void Paddle::isrReadEncoder11()
{
    Paddle::instances[0]->changeTarget(Paddle::instances[1]->readA());
}

void Paddle::calibrate()
{
    Paddle *p1 = Paddle::instances[0];
    Paddle *p2 = Paddle::instances[1];

    p1->_stepper->setSpeed(-400);
    p2->_stepper->setSpeed(-400);

    delayMicroseconds(20);

    // while (!p1->_stepper->calibrated || !p2->_stepper->calibrated)
    // {
    //     // TODO
    //     if (!digitalRead(3))
    //     {
    //         p1->_stepper->calibrated = true;
    //     }

    //     // TODO
    //     if (!digitalRead(4))
    //     {
    //         p2->_stepper->calibrated = true;
    //     }

    //     if (!p1->_stepper->calibrated)
    //     {
    //         p1->_stepper->runSpeed();
    //     }

    //     if (!p2->_stepper->calibrated)
    //     {
    //         p2->_stepper->runSpeed();
    //     }
    // }

    p1->_stepper->setCurrentPosition(-10);
    p2->_stepper->setCurrentPosition(-10);

    Paddle::calibrated = true;
}

void Paddle::centerAll()
{
    Paddle *p1 = Paddle::instances[0];
    Paddle *p2 = Paddle::instances[1];

    p1->_stepper->moveTo(PADDLE_CENTER);
    p2->_stepper->moveTo(PADDLE_CENTER);
    delay(1);
}

void Paddle::run()
{
    Paddle::instances[0]->_stepper->run();
    Paddle::instances[1]->_stepper->run();
}

bool Paddle::isRunning()
{
    return Paddle::instances[0]->needsToMove() || Paddle::instances[1]->needsToMove();
}

void Paddle::attachPaddles()
{

#ifdef ARDUINO_ARCH_RP2040
  attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[0]->_pinA),
        Paddle::isrReadEncoder0, RISING);
    attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[0]->_pinB),
        Paddle::isrReadEncoder01, RISING);
    attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[1]->_pinA),
        Paddle::isrReadEncoder10, RISING);
    attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[1]->_pinB),
        Paddle::isrReadEncoder11, RISING);
#endif
#ifdef ARDUINO_SAM_DUE
  attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[0]->_pinA),
        Paddle::isrReadEncoder0, RISING);
    attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[0]->_pinB),
        Paddle::isrReadEncoder01, RISING);
    attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[1]->_pinA),
        Paddle::isrReadEncoder10, RISING);
    attachInterrupt(
        digitalPinToInterrupt(
            Paddle::instances[1]->_pinB),
        Paddle::isrReadEncoder11, RISING);
#endif
    
}

void Paddle::detachPaddles()
{
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[0]->_pinA));
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[1]->_pinA));
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[0]->_pinB));
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[1]->_pinB));
}

byte Paddle::canShoot(long ballPos)
{
    long paddlePos = this->getCenterRelativePosition();

    long paddleLeftEdge = paddlePos - PADDLE_WIDTH_HALF;
    long paddleRightEdge = paddlePos + PADDLE_WIDTH_HALF;
    long ballLeftEdge = ballPos - BALL_WIDTH_HALF;
    long ballRightEdge = ballPos + BALL_WIDTH_HALF;

    long hitzone = PADDLE_WIDTH_HALF + BALL_WIDTH_HALF;

    // Check if the ball is within the paddle's bounds
    if (ballRightEdge >= paddleLeftEdge && ballLeftEdge <= paddleRightEdge)
    {
        // Calculate the difference in positions
        double relativePosition = (ballPos - paddlePos);

        // Map the relative position to an angle between 30 and 150 degrees
        return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
    }
    else
    {
        // No collision
        return 0;
    }
}