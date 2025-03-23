#include <Configuration.hpp>
#include <TrigTables.hpp>

#include "Ball.hpp"

Ball *Ball::instance;

int mapAngleToSpeedX(int angle)
{
    return static_cast<int>((abs(sin(angle * PI / 180))) * MAX_SPEED);
}

int mapAngleToSpeedY(int angle)
{
    return static_cast<int>((abs(cos(angle * PI / 180))) * MAX_SPEED);
}

void Ball::init(size_t stepX, size_t dirX, size_t stepY, size_t dirY)
{
    pinMode(stepX, OUTPUT);
    pinMode(dirX,OUTPUT);
    pinMode(stepY, OUTPUT);
    pinMode(dirY,OUTPUT);

    this->_xStepper = new PongStepper(stepX,dirX);
    this->_yStepper = new PongStepper(stepY,dirY);

    this->_xStepper->setMaxSpeed(1600);
    this->_yStepper->setMaxSpeed(1600);
    this->_xStepper->setAcceleration(20000);
    this->_yStepper->setAcceleration(20000);
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
    return this->_yStepper->currentPosition() - (1000 >> 1);
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
    this->setposition(1000 >> 1, 1000 >> 1);
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
    Serial.println("SHOOT");
    Serial.println(degrees);
    this->lastAngle = degrees;
    this->shootAngle(0, isBounce);
}

void Ball::shootAngle(float angleRadians, bool isBounce)
{
    Point ballPos = this->getPosition();

    if(this->lastAngle == 90){
        this->_xStepper->setMaxSpeed(mapAngleToSpeedX(this->lastAngle));
        this->_xStepper->moveTo(ballPos.x == 1000 ? 0 : 1000);

        return;
    }

    if ((ballPos.y == 1000 &&
         (this->lastAngle < 90 || this->lastAngle > 270)) ||
        (ballPos.y == 0 &&
         (this->lastAngle > 90 && this->lastAngle < 270)))
    {
        return this->shootDeg(this->inverseAngle(this->lastAngle),isBounce);
    }

    double dy = fastCos(this->lastAngle);
    double dx = fastSin(this->lastAngle);

    bool horizontalModifier = dy >= 0;
    bool verticalModifier = dx >= 0;

    double a = horizontalModifier ? 1000 - ballPos.y : ballPos.y;
    double o = verticalModifier ? 1000 - ballPos.x : ballPos.x;

    double ty = a / dy;
    double tx = o / dx;

    double t = min(abs(ty), abs(tx));

    // int newX = constrain(ballPos.x + round(dx * t), 0, 1000);
    int newY = constrain(ballPos.y + round(dy * t), 0, 1000);

    this->_xStepper->setMaxSpeed(mapAngleToSpeedX(this->lastAngle));
    this->_yStepper->setMaxSpeed(mapAngleToSpeedY(this->lastAngle));

   
        if(!isBounce){
            this->_xStepper->moveTo(ballPos.x == 1000 ? 0 : 1000);
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
    this->_yStepper->run();
}

bool Ball::isRunning()
{
    return this->_xStepper->isRunning() || this->_yStepper->isRunning();
}

long Ball::getX()
{
    Serial.println(this->_xStepper->currentPosition());
    return this->_xStepper->currentPosition();
}

long Ball::getY()
{
    return this->_yStepper->currentPosition();
}

Ball::~Ball()
{
    delete this->_xStepper;
    delete this->_yStepper;
}