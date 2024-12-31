#include <Configuration.hpp>

#include "Ball.hpp"

Ball *Ball::instance;

int mapAngleToSpeed(int angle)
{
    return static_cast<int>((1 / abs(sin(angle * PI / 180))) * MAX_SPEED);
}

void Ball::setMotors(PongStepper *xStepper, PongStepper *yStepper)
{
    this->_xStepper = xStepper;
    this->_yStepper = yStepper;
}


void Ball::setposition(int x, int y)
{
    this->setposition(x, y, mapAngleToSpeed(this->lastAngle));
}

void Ball::setCurrentPosition(int x, int y)
{
    // this->_xStepper->setCurrentPosition(x);
    // this->_yStepper->setCurrentPosition(y);
}

void Ball::setposition(int x, int y, int speed)
{
    // this->_xSteppers->setPosition(x, speed, START_SPEED, END_SPEED);
    // this->_ySteppers->setPosition(y, speed, START_SPEED, END_SPEED);
}

void Ball::setposition(int x, int y, int speed, int startSpeed, int endSpeed)
{
    // this->_xSteppers->setPosition(x, speed, startSpeed, endSpeed);
    // this->_ySteppers->setPosition(y, speed, startSpeed, endSpeed);
}

void Ball::stop()
{
    // this->_xSteppers->stop();
    // this->_ySteppers->stop();
}

Point Ball::getPosition()
{
    Point result = Point();
    // result.x = this->_xStepper->getPosition();
    // result.y = this->_yStepper->getPosition();

    return result;
}

long Ball::getCenterRelativePosition()
{
    return this->getPosition().y - (this->limits.y >> 1);
}

void Ball::calibrate()
{
    // this->initCalibration();

    // BoolCallback leftLimitHit = []()
    // { return digitalRead(LS1) == HIGH ? true : false; };

    // this->_steppers->moveWhile(HIGH, HIGH, CALIBRATION_SPEED, leftLimitHit);
    // delay(200);
    // this->setCurrentPosition(GAMEPLAY_AREA_X + 10, GAMEPLAY_AREA_Y + 15);
    // this->limits.x = GAMEPLAY_AREA_X;
    // this->limits.y = GAMEPLAY_AREA_Y; // 1step = 0.025cm // somehow 2280 works better maybe belt tension issue
    // delay(200);
    // return;
}

void Ball::runCenter()
{
    this->center();
}

void Ball::center()
{
    this->setposition(this->limits.x >> 1, this->limits.y >> 1);
}

bool Ball::needsToMove()
{
    // return this->_xSteppers->isMoving() || this->_ySteppers->isMoving();
    return true;
}

void Ball::bounce()
{
    if (this->lastAngle < 180)
    {
        this->shootDeg(180 - this->lastAngle);
    }
    else
    {
        this->shootDeg(540 - this->lastAngle);
    }
}

void Ball::shootDeg(uint16_t degrees)
{
    this->lastAngle = degrees;
    this->shootAngle((float)degrees * (float)PI / 180.f);
}

void Ball::shootAngle(float angleRadians)
{
    Point ballPos = this->getPosition();

    if ((ballPos.y == this->limits.y &&
         (this->lastAngle < 90 || this->lastAngle > 270)) ||
        (ballPos.y == 0 &&
         (this->lastAngle > 90 && this->lastAngle < 270)))
    {
        return this->shootDeg(this->inverseAngle(this->lastAngle));
    }

    double dy = cos(angleRadians);
    double dx = sin(angleRadians);

    bool horizontalModifier = dy >= 0;
    bool verticalModifier = dx >= 0;

    double a = horizontalModifier ? this->limits.y - ballPos.y : ballPos.y;
    double o = verticalModifier ? this->limits.x - ballPos.x : ballPos.x;

    double ty = a / dy;
    double tx = o / dx;

    double t = min(abs(ty), abs(tx));

    int newX = constrain(ballPos.x + round(dx * t), 0, this->limits.x);
    int newY = constrain(ballPos.y + round(dy * t), 0, this->limits.y);

    // speed modification
    float modifier = abs(sin(angleRadians) - 0.70710678118) * 2;
    int _startSpeed = ballPos.x == 0 || ballPos.x == this->limits.x ? START_SPEED : START_SPEED + (int)(START_SPEED * modifier);
    int _endSpeed = newX == 0 || newX == this->limits.x ? END_SPEED : END_SPEED + (int)(END_SPEED * modifier);

    this->setposition(newX, newY, mapAngleToSpeed(this->lastAngle), _startSpeed, _endSpeed);
    // if (newX != 0 && newX != this->limits.x)
    // {
    //     // will bounce
    //     if (this->lastAngle < 180)
    //     {
    //         this->setNextTarget(min(newX + 50, this->limits.x), newY);
    //     }
    //     else
    //     {
    //         this->setNextTarget(max(newX - 50, 0), newY);
    //     }
    // }
}

uint16_t Ball::inverseAngle(int16_t angle)
{
    if (angle < 180)
    {
        return 180 - angle;
    }

    return 540 - angle;
}

void Ball::stopNow()
{
    // this->_steppers->stopNow();
}