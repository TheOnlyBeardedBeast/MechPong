#include "Paddle.hpp"
#include <Configuration.hpp>
#include "sign.hpp"

Paddle *Paddle::instances[2];
bool Paddle::calibrated = false;
bool Paddle::attached = false;

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
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);

    this->_stepper = new PaddleStepper(step, dir);
    this->_stepper->setMaxSpeed(PADDLE_MAX_SPEED);
    this->_stepper->setAcceleration(PADDLE_ACCELERATION);
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
    // this->stepIndex++;

    // if(this->stepIndex==4){
    //     // long target = constrain(this->_stepper->targetPosition() + 1, 0, PADDLE_LIMIT);
    //     // this->_stepper->moveTo(target);
    this->futureTarget = constrain(this->futureTarget + 1, 0, PADDLE_LIMIT);
    //     this->stepIndex = 2;
    // }
}

void Paddle::decrement()
{
    // this->stepIndex--;

    // if(this->stepIndex == 0)
    // {
    // long target = constrain(this->_stepper->targetPosition() - 1, 0, PADDLE_LIMIT);
    // this->_stepper->moveTo(target);
    this->futureTarget = constrain(this->futureTarget - 1, 0, PADDLE_LIMIT);
    // this->stepIndex = 2;
    // }
}

Paddle::~Paddle()
{
    delete this->_stepper;
    delete this->limitSwitch;
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

long Paddle::stepsToQuickStop()
{
    return (this->_stepper->stepperStepsToStop >> 2) * sign(this->_stepper->speed());
}

void Paddle::isrReadEncoder0()
{
    int b = Paddle::instances[0]->readB();

    auto& instance = *Paddle::instances[0];

    uint32_t now = time_us_32();
    uint32_t dt = now - instance.last_pulse_time;
    instance.last_pulse_time = now;

    if(dt > 250000)
    {
        return;
    }
    
    instance.pulseSum -= instance.pulse_history[instance.pulseIdx];

    instance.pulse_history[instance.pulseIdx] = dt;
    instance.pulseSum += dt;

    instance.pulseIdx = (instance.pulseIdx + 1) & 15;

    uint16_t avg = instance.pulseSum >> 4;

    if(avg > 60000)
    {
        return;
    }
    // Paddle::instances[0]->last_pulse_time = now;

    // Paddle::instances[0]->pulse_history >>= 1;
    // if (dt <= 10000)  // 5 ms threshold
    //     Paddle::instances[0]->pulse_history |= 0x80;  // set MSB
    // // else leave MSB = 0

    // if (Paddle::instances[0]->pulse_history != 0xFF) {
    //     return;
    // }

    if (b == LOW)
    {
        Paddle::instances[0]->increment();
    }
    else
    {
        Paddle::instances[0]->decrement();
    }

}

void Paddle::isrReadEncoder01()
{

    int a = Paddle::instances[0]->readA();

    auto& instance = *Paddle::instances[0];

    uint32_t now = time_us_32();
    uint32_t dt = now - instance.last_pulse_time;
    instance.last_pulse_time = now;

    if(dt > 250000)
    {
        return;
    }
    
    instance.pulseSum -= instance.pulse_history[instance.pulseIdx];

    instance.pulse_history[instance.pulseIdx] = dt;
    instance.pulseSum += dt;

    instance.pulseIdx = (instance.pulseIdx + 1) & 15;

    uint16_t avg = instance.pulseSum >> 4;

    if(avg > 60000)
    {
        return;
    }

    if (a == HIGH)
    {
        Paddle::instances[0]->increment();
    }
    else
    {
        Paddle::instances[0]->decrement();
    }
}

void Paddle::isrReadEncoder10()
{
    int b = Paddle::instances[1]->readB();

    auto& instance = *Paddle::instances[1];

    uint32_t now = time_us_32();
    uint32_t dt = now - instance.last_pulse_time;
    instance.last_pulse_time = now;

    if(dt > 250000)
    {
        return;
    }
    
    instance.pulseSum -= instance.pulse_history[instance.pulseIdx];

    instance.pulse_history[instance.pulseIdx] = dt;
    instance.pulseSum += dt;

    instance.pulseIdx = (instance.pulseIdx + 1) & 15;

    uint16_t avg = instance.pulseSum >> 4;

    if(avg > 60000)
    {
        return;
    }

    if (b == LOW)
    {
        Paddle::instances[1]->increment();
    }
    else
    {
        Paddle::instances[1]->decrement();
    }
}

void Paddle::isrReadEncoder11()
{
    int a = Paddle::instances[1]->readA();

    auto& instance = *Paddle::instances[1];

    uint32_t now = time_us_32();
    uint32_t dt = now - instance.last_pulse_time;
    instance.last_pulse_time = now;

    if(dt > 250000)
    {
        return;
    }
    
    instance.pulseSum -= instance.pulse_history[instance.pulseIdx];

    instance.pulse_history[instance.pulseIdx] = dt;
    instance.pulseSum += dt;

    instance.pulseIdx = (instance.pulseIdx + 1) & 15;

    uint16_t avg = instance.pulseSum >> 4;

    if(avg > 60000)
    {
        return;
    }

    if (a == HIGH)
    {
        Paddle::instances[1]->increment();
    }
    else
    {
        Paddle::instances[1]->decrement();
    }
}

void Paddle::calibrate()
{
    Paddle::calibrated = false;

    Paddle *p1 = Paddle::instances[0];
    Paddle *p2 = Paddle::instances[1];

    p1->_stepper->setSpeed(-CALIBRATION_SPEED);
    p2->_stepper->setSpeed(-CALIBRATION_SPEED);

    delayMicroseconds(20);

    while (!Paddle::calibrated)
    {

        if (!p1->limitSwitch->isClicked())
        {
            p1->_stepper->runSpeed();
        }

        if (!p2->limitSwitch->isClicked())
        {
            p2->_stepper->runSpeed();
        }

        if (p1->limitSwitch->isClicked() && p2->limitSwitch->isClicked())
        {
            Paddle::calibrated = true;
        }

        sleep_ms(1);
    }

    p1->_stepper->setCurrentPosition(-BALL_WIDTH - BALL_WIDTH_HALF);
    p2->_stepper->setCurrentPosition(-BALL_WIDTH - BALL_WIDTH_HALF);
}

void Paddle::centerAll()
{
    Paddle *p1 = Paddle::instances[0];
    Paddle *p2 = Paddle::instances[1];

    p1->_stepper->moveTo(PADDLE_CENTER);
    p2->_stepper->moveTo(PADDLE_CENTER);
    sleep_ms(1);
}

void Paddle::run()
{
    Paddle::instances[0]->_stepper->run();
    sleep_us(1);
    Paddle::instances[1]->_stepper->run();
    sleep_us(1);
}

bool Paddle::isRunning()
{
    return Paddle::instances[0]->needsToMove() || Paddle::instances[1]->needsToMove();
}

void Paddle::update()
{
    if (!Paddle::attached)
    {
        return;
    }
    Paddle::instances[0]->_stepper->moveTo(Paddle::instances[0]->futureTarget);
    Paddle::instances[1]->_stepper->moveTo(Paddle::instances[1]->futureTarget);
}

void Paddle::attachPaddles()
{
    Paddle::attached = true;
    Paddle::instances[0]->futureTarget = Paddle::instances[0]->_stepper->currentPosition();
    Paddle::instances[1]->futureTarget = Paddle::instances[1]->_stepper->currentPosition();

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
    Paddle::attached = false;
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[0]->_pinA));
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[1]->_pinA));
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[0]->_pinB));
    detachInterrupt(digitalPinToInterrupt(Paddle::instances[1]->_pinB));
}

void Paddle::quickStop()
{
    Paddle::instances[0]->_stepper->setAcceleration(PADDLE_ACCELERATION << 2);
    Paddle::instances[1]->_stepper->setAcceleration(PADDLE_ACCELERATION << 2);

    Paddle::instances[0]->_stepper->stop();
    Paddle::instances[1]->_stepper->stop();
}

void Paddle::resetAcceleration()
{
    Paddle::instances[0]->_stepper->setAcceleration(PADDLE_ACCELERATION);
    Paddle::instances[1]->_stepper->setAcceleration(PADDLE_ACCELERATION);
}

// byte Paddle::tryShoot(long ballPos, long ballTarget, long possibleHitOffset)
// {
//     long paddlePos = this->getCenterRelativePosition();

//     long paddleLeftEdge = paddlePos - PADDLE_WIDTH_HALF;
//     long paddleRightEdge = paddlePos + PADDLE_WIDTH_HALF;
//     long ballLeftEdge = ballPos - BALL_WIDTH_HALF;
//     long ballRightEdge = ballPos + BALL_WIDTH_HALF;

//     long hitzone = PADDLE_WIDTH_HALF + BALL_WIDTH_HALF;

//     // // Check if the ball is within the paddle's bounds
//     // if (ballRightEdge >= paddleLeftEdge && ballLeftEdge <= paddleRightEdge)
//     // {
//     //     // Calculate the difference in positions
//     //     double relativePosition = (ballPos - paddlePos);

//     //     // Map the relative position to an angle between 30 and 150 degrees
//     //     return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
//     // }

//     ballLeftEdge = ballPos + possibleHitOffset - BALL_WIDTH_HALF;
//     ballRightEdge = ballPos + possibleHitOffset + BALL_WIDTH_HALF;

//     long paddleStopSteps = this->stepsToQuickStop();

//     paddleLeftEdge = paddlePos + paddleStopSteps - PADDLE_WIDTH_HALF;
//     paddleRightEdge = paddlePos + paddleStopSteps + PADDLE_WIDTH_HALF;

//     if (ballRightEdge >= paddleLeftEdge && ballLeftEdge <= paddleRightEdge)
//     {
//         double relativePosition = (ballPos + possibleHitOffset - paddlePos + paddleStopSteps);

//         return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
//     }

//     ballLeftEdge = ballTarget - BALL_WIDTH_HALF;
//     ballRightEdge = ballTarget + BALL_WIDTH_HALF;

//     if (ballRightEdge >= paddleLeftEdge && ballLeftEdge <= paddleRightEdge)
//     {
//         double relativePosition = (ballTarget - paddlePos + paddleStopSteps);

//         return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
//     }

//     paddleLeftEdge = paddlePos - PADDLE_WIDTH_HALF;
//     paddleRightEdge = paddlePos + PADDLE_WIDTH_HALF;

//     // if ((ballPos < paddleLeftEdge && ballTarget > paddleRightEdge + paddleStopSteps) ||
//     //     (ballPos > paddleRightEdge && ballTarget < paddleLeftEdge + paddleStopSteps))
//     // {
//     //     // double relativePosition = (ballPos - paddlePos);
//     //     // return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
//     //     return 90;
//     // }

//     if ((ballPos < paddleLeftEdge && ballTarget > paddleRightEdge + paddleStopSteps) ||
//         (ballPos > paddleRightEdge && ballTarget < paddleLeftEdge + paddleStopSteps))
//     {
//         long estimatedCrossPos = (paddleLeftEdge + paddleRightEdge + paddleStopSteps) / 2;

//         double relativePosition = estimatedCrossPos - paddlePos + paddleStopSteps;

//         relativePosition = constrain(relativePosition, -hitzone, hitzone);

//         return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
//     }

//     // No collision
//     return 0;
// }

byte Paddle::tryShoot(long ballPos, long ballTarget, long possibleHitOffset)
{
    long paddlePos = this->getCenterRelativePosition();
    long paddleStopSteps = this->stepsToQuickStop();

    long hitzone = PADDLE_WIDTH_HALF;

    // Paddle edges - current and future (after stop steps)
    long paddleLeftEdge = paddlePos - PADDLE_WIDTH_HALF;
    long paddleRightEdge = paddlePos + PADDLE_WIDTH_HALF;
    long paddleLeftEdgeFuture = paddleLeftEdge + paddleStopSteps;
    long paddleRightEdgeFuture = paddleRightEdge + paddleStopSteps;

    // Ball edges - raw position
    long ballLeftEdge = ballPos - BALL_WIDTH_HALF;
    long ballRightEdge = ballPos + BALL_WIDTH_HALF;

    // Ball edges - with possibleHitOffset
    long ballLeftEdgeOffset = ballPos + possibleHitOffset - BALL_WIDTH_HALF;
    long ballRightEdgeOffset = ballPos + possibleHitOffset + BALL_WIDTH_HALF;

    // Ball edges - target position
    long ballLeftEdgeTarget = ballTarget - BALL_WIDTH_HALF;
    long ballRightEdgeTarget = ballTarget + BALL_WIDTH_HALF;

    // First check: raw position overlap
    // if (ballRightEdge >= paddleLeftEdge && ballLeftEdge <= paddleRightEdge)
    // {
    //     double relativePosition = ballPos - paddlePos;
    //     relativePosition = constrain(relativePosition, -hitzone, hitzone);
    //     return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
    // }

    // Second check: predicted with offset
    if (ballRightEdgeOffset >= paddleLeftEdgeFuture && ballLeftEdgeOffset <= paddleRightEdgeFuture)
    {
        double relativePosition = ballPos + possibleHitOffset - paddlePos + paddleStopSteps;
        relativePosition = constrain(relativePosition, -hitzone, hitzone);
        return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
    }

    // Third check: ball's target edges overlap future paddle edges
    if (ballRightEdgeTarget >= paddleLeftEdgeFuture && ballLeftEdgeTarget <= paddleRightEdgeFuture)
    {
        double relativePosition = ballTarget - paddlePos + paddleStopSteps;
        relativePosition = constrain(relativePosition, -hitzone, hitzone);
        return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
    }

    // Final check: crossing through paddle
    if ((ballPos < paddleLeftEdge && ballTarget > paddleRightEdgeFuture) ||
        (ballPos > paddleRightEdge && ballTarget < paddleLeftEdgeFuture))
    {
        long estimatedCrossPos = (paddleLeftEdge + paddleRightEdge + paddleStopSteps) / 2;
        double relativePosition = estimatedCrossPos - paddlePos + paddleStopSteps;
        relativePosition = constrain(relativePosition, -hitzone, hitzone);
        return round(map(relativePosition, -hitzone, hitzone, MIN_ANGLE_MUL, MAX_ANGLE_MUL)) * 5;
    }

    // No collision
    return 0;
}
