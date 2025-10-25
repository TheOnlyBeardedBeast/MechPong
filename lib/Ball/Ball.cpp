#include <Configuration.hpp>
#include <TrigTables.hpp>
#include <GetRelativeAngle.hpp>
#include <sign.hpp>

#include "Ball.hpp"

Ball *Ball::instance;

int mapAngleToSpeedX(int angle, int spped = MAX_SPEED)
{
    return static_cast<int>((abs(fastSin(angle))) * spped);
}

int mapAngleToSpeedY(int angle, int spped = MAX_SPEED)
{
    return static_cast<int>((abs(fastCos(angle))) * spped);
}

void Ball::init(size_t stepX, size_t dirX, size_t stepY, size_t dirY)
{
    // pinMode(stepX, OUTPUT);
    pinMode(dirX, OUTPUT);
    // pinMode(stepY, OUTPUT);
    pinMode(dirY, OUTPUT);

    this->_xStepper = new PongStepper(stepX, dirX);
    this->_yStepper = new PongStepper(stepY, dirY);

    this->_xStepper->setMaxSpeed(MAX_SPEED);
    this->_yStepper->setMaxSpeed(MAX_SPEED);
    this->_xStepper->setAcceleration(BALL_ACCELERATION);
    this->_yStepper->setAcceleration(BALL_ACCELERATION);
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

volatile Point Ball::getPosition()
{
    Point result = Point();

    result.x = static_cast<int>(this->_xStepper->currentPosition());
    result.y = static_cast<int>(this->_yStepper->currentPosition());

    return result;
}

long Ball::getCenterRelativePosition()
{
    return this->_yStepper->currentPosition() - (GAMEPLAY_AREA_Y >> 1);
}

void Ball::calibrate()
{
    this->_yStepper->setSpeed(-CALIBRATION_SPEED);
    while (!this->limitSwitchY->isClicked())
    {
        this->_yStepper->runSpeed();
    }

    this->_xStepper->setSpeed(-CALIBRATION_SPEED);
    while (!this->limitSwitchX->isClicked())
    {
        this->_xStepper->runSpeed();
    }

    this->_xStepper->setCurrentPosition(-BALL_WIDTH_HALF);
    this->_yStepper->setCurrentPosition(-BALL_WIDTH_HALF);
}

void Ball::runCenter()
{
    this->center();
}

void Ball::center()
{
    this->setposition(GAMEPLAY_AREA_X >> 1, GAMEPLAY_AREA_Y >> 1);
}

bool Ball::needsToMove()
{
    return this->_xStepper->isRunning() || this->_yStepper->isRunning();
}

void Ball::bounce()
{
    if (this->lastAngle < 180)
    {
        this->shootDeg(180 - this->lastAngle, true);
    }
    else
    {
        this->shootDeg(540 - this->lastAngle, true);
    }
}

void Ball::shootDeg(uint16_t degrees, bool isBounce)
{
    // Serial.println("SHOOT");
    // Serial.println(degrees);
    this->lastAngle = degrees;
    this->shootAngle(0, isBounce);
}

void Ball::shootAngle(float angleRadians, bool isBounce)
{
    Point ballPos = this->getPosition();

    // if(this->lastAngle == 90){
    //     this->_xStepper->setMaxSpeed(mapAngleToSpeedX(this->lastAngle));
    //     this->_xStepper->moveTo(ballPos.x == GAMEPLAY_AREA_X ? 0 : GAMEPLAY_AREA_X);

    //     return;
    // }

    if ((ballPos.y == GAMEPLAY_AREA_Y &&
         (this->lastAngle < 90 || this->lastAngle > 270)) ||
        (ballPos.y == 0 &&
         (this->lastAngle > 90 && this->lastAngle < 270)))
    {
        return this->shootDeg(this->inverseAngle(this->lastAngle), isBounce);
    }

    double dy = fastCos(this->lastAngle);
    double dx = fastSin(this->lastAngle);

    bool horizontalModifier = dy >= 0;
    bool verticalModifier = dx >= 0;

    double a = horizontalModifier ? GAMEPLAY_AREA_Y - ballPos.y : ballPos.y;
    double o = verticalModifier ? GAMEPLAY_AREA_X - ballPos.x : ballPos.x;

    double ty = a / dy;
    double tx = o / dx;

    double t = min(abs(ty), abs(tx));

    // int newX = constrain(ballPos.x + round(dx * t), 0, GAMEPLAY_AREA_X);
    int newY = constrain(ballPos.y + round(dy * t), 0, GAMEPLAY_AREA_Y);

    this->_xStepper->setMaxSpeed(mapAngleToSpeedX(this->lastAngle,this->_maxSpeed));
    this->_yStepper->setMaxSpeed(mapAngleToSpeedY(this->lastAngle,this->_maxSpeed));

    if (!isBounce)
    {
        this->_xStepper->moveTo(verticalModifier ? GAMEPLAY_AREA_X : 0);
    }
    this->_yStepper->moveTo(newY);
}

uint16_t Ball::inverseAngle(int16_t angle)
{
    if (angle < 180)
    {
        return 180 - angle;
    }

    return 540 - angle;
}

void Ball::resetSpeeds()
{
    this->_xStepper->setMaxSpeed(MAX_SPEED);
    this->_yStepper->setMaxSpeed(MAX_SPEED);
}

void Ball::run()
{
    this->_xStepper->run();
    sleep_us(1);
    this->_yStepper->run();
    sleep_us(1);
}

bool Ball::isRunning()
{
    return this->_xStepper->isRunning() || this->_yStepper->isRunning();
}

long Ball::getX()
{
    return this->_xStepper->currentPosition();
}

long Ball::getY()
{
    return this->_yStepper->currentPosition();
}

long Ball::stepsToStopX()
{
    // long speed = this->_xStepper->speed();

    // return (long)((speed * speed) / (BALL_ACCELERATION << 1));
    return this->_xStepper->stepperStepsToStop;
}

void Ball::printInfo()
{
    Serial.print("x.targetPosition:");
    Serial.println(this->_xStepper->targetPosition());
    Serial.print("x.speed:");
    Serial.println(this->_xStepper->speed());
    Serial.print("y.targetPosition:");
    Serial.println(this->_yStepper->targetPosition());
    Serial.print("y.speed:");
    Serial.println(this->_yStepper->speed());
    Serial.print("isRunning:");
    Serial.println(this->isRunning());
}

long Ball::getPossibleHitOffset()
{
    uint16_t relativeNagle = getRelativeAngle(this->lastAngle);

    return this->stepsToStopX() * fastTan(relativeNagle) * sign(this->_yStepper->speed());
}

long Ball::getCenterRelativeTargetPosition()
{
    return this->_yStepper->targetPosition() - (GAMEPLAY_AREA_Y >> 1);
}

Ball::~Ball()
{
    delete this->_xStepper;
    delete this->_yStepper;
}

void Ball::increaseSpeed()
{
    if(_maxSpeed == MAX_SPEED)
    {
        return;
    }

    this->_maxSpeed += SPEED_UPDATE;
}

void Ball::resetGameSpeed()
{
    if(SPEED_SCALING)
    {
        this->_maxSpeed = START_SPEED;
        return;
    }

    this->_maxSpeed = MAX_SPEED;
}
