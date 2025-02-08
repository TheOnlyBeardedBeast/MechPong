#include <Configuration.hpp>

#include "Ball.hpp"

Ball *Ball::instance;

int mapAngleToSpeedX(int angle)
{
    return static_cast<int>((1 / abs(sin(angle * PI / 180))) * MAX_SPEED);
}

int mapAngleToSpeedY(int angle)
{
    return static_cast<int>((1 / abs(cos(angle * PI / 180))) * MAX_SPEED);
}

void Ball::init(size_t stepX, size_t dirX, size_t stepY, size_t dirY)
{
    pinMode(stepX, OUTPUT);
    pinMode(dirX,OUTPUT);
    pinMode(stepY, OUTPUT);
    pinMode(dirY,OUTPUT);

    this->_xStepper = new PongStepper(stepX,dirX);
    this->_yStepper = new PongStepper(stepY,dirY);
}

void Ball::setposition(int x, int y)
{
    this->setposition(x, y);
}

void Ball::setCurrentPosition(int x, int y)
{
    this->_xStepper->setCurrentPosition(x);
    this->_yStepper->setCurrentPosition(y);
}

void Ball::setposition(int x, int y)
{
    this->_xStepper->moveTo(x);
    this->_yStepper->moveTo(y);
}

void Ball::stop()
{
    this->_xStepper->stop();
    this->_yStepper->stop();
}

Point Ball::getPosition()
{
    Point result = Point();

    result.x = this->_xStepper->currentPosition();
    result.y = this->_yStepper->currentPosition();

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
    return this->_xStepper->isRunning() || this->_yStepper->isRunning();
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

    this->_xStepper->setMaxSpeed(mapAngleToSpeedX(this->lastAngle));
    this->_yStepper->setMaxSpeed(mapAngleToSpeedY(this->lastAngle));

    this->setposition(newX, newY);

    // TODO: whole route planning here
}

uint16_t Ball::inverseAngle(int16_t angle)
{
    if (angle < 180)
    {
        return 180 - angle;
    }

    return 540 - angle;
}

Ball::~Ball()
{
    delete this->_xStepper;
    delete this->_yStepper;
}