#include "PaddleStepper.hpp"

void PaddleStepper::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
        _targetPos = absolute;
        computeNewSpeed();

        if(this->subscriber != NULL)
        {
            this->subscriber->moveTo(absolute);
        }
    }
}

void PaddleStepper::updateDirection()
{
    if (this->_futureDirection != this->_direction)
    {
        this->_direction = this->_futureDirection;
        digitalWrite(this->_pin[1], this->_direction ^ _pinInverted[1]);

        // Depracted
        // if (this->subscriber != NULL)
        // {
        //     this->subscriber->externalUpdateDirection(this->_direction);
        // }
    }
}

void PaddleStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean PaddleStepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
    {
        return false;
    }

    unsigned long time = micros();
    unsigned long delta = time - _lastStepTime;

    if(this->_futureDirection != this->_direction && delta >= this->_stepInterval - 10 && delta < this->_stepInterval)
    {
        this->updateDirection();
        return false;
    }

    if (this->shouldClear)
    {
        if (delta >= this->_minPulseWidth)
        {
            this->clear();
        }

        return false;
    }

    if (delta >= _stepInterval)
    {
        if (_direction == true)
        {
            // Clockwise
            _currentPos += 1;
        }
        else
        {
            // Anticlockwise
            _currentPos -= 1;
        }
        step(_currentPos);

        _lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    }
    else
    {
        return false;
    }
}

long PaddleStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long PaddleStepper::targetPosition()
{
    return _targetPos;
}

long PaddleStepper::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void PaddleStepper::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

void PaddleStepper::clear()
{
    this->shouldClear = false;
    digitalWrite(this->_pin[0], LOW);

    // Depracted
    // if (this->subscriber != NULL)
    // {
    //     this->subscriber->clear();
    // }
}

void PaddleStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
        // We are at the target and its time to stop
        _stepInterval = 0;
        _speed = 0.0;
        _n = 0;
        return;
    }

    if (distanceTo > 0)
    {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || _direction == false)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && _direction == true)
                _n = -_n; // Start accceleration
        }
    }
    else if (distanceTo < 0)
    {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || _direction == true)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && _direction == false)
                _n = -_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
        // First step from stopped
        _cn = _c0;
        this->_futureDirection = ((distanceTo > 0) ? true : false);
    }
    else
    {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
        _cn = max(_cn, _cmin);
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == false)
        _speed = -_speed;
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
boolean PaddleStepper::run()
{
    if (runSpeed())
        computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

PaddleStepper::PaddleStepper(uint8_t pin1, uint8_t pin2)
{
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 5;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _pin[0] = pin1;
    _pin[1] = pin2;
    _enableInverted = false;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = false;

    _pinInverted[0] = 0;
    _pinInverted[1] = 0;

    // Some reasonable default
    setAcceleration(1);
}

void PaddleStepper::setMaxSpeed(float speed)
{
    if (speed < 0.0)
    {
        speed = -speed;
    }
    if (_maxSpeed != speed)
    {
        _maxSpeed = speed;
        _cmin = 1000000.0 / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (_n > 0)
        {
            _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
            computeNewSpeed();
        }
    }
}

float PaddleStepper::maxSpeed()
{
    return _maxSpeed;
}

void PaddleStepper::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
        return;
    if (acceleration < 0.0)
        acceleration = -acceleration;
    if (_acceleration != acceleration)
    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
        _acceleration = acceleration;
        computeNewSpeed();
    }
}

void PaddleStepper::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
        _stepInterval = 0;
    else
    {
        _stepInterval = fabs(1000000.0 / speed);
        this->_futureDirection = ((speed > 0.0) ? true : false);
    }
    _speed = speed;
}

float PaddleStepper::speed()
{
    return _speed;
}

// Subclasses can override
void PaddleStepper::step(long step)
{
    (void)(step); // Unused

    // _pin[0] is step, _pin[1] is direction

    digitalWrite(this->_pin[0], HIGH);

    this->shouldClear = true;

    // Depracted
    // if (this->subscriber != NULL)
    // {
    //     this->subscriber->externalStep(step);
    // }
}

void PaddleStepper::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void PaddleStepper::setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void PaddleStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void PaddleStepper::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert)
{
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void PaddleStepper::runToPosition()
{
    while (run());
}

boolean PaddleStepper::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
    {
        return false;
    }

    if (_targetPos > _currentPos)
    {
        this->_futureDirection = (true);
    }
    else
    {
        this->_futureDirection = (false);
    }

    return runSpeed();
}

// Blocks until the new target position is reached
void PaddleStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void PaddleStepper::stop()
{
    if (_speed != 0.0)
    {
        long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
        if (_speed > 0)
            move(stepsToStop);
        else
            move(-stepsToStop);
    }
}

bool PaddleStepper::isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos && !this->shouldClear);
}

void PaddleStepper::subscribe(PaddleSubscriber *subscriber)
{
    this->subscriber = subscriber;
    this->subscriber->externalUpdateDirection(this->_direction);
}
void PaddleStepper::unsubscribe()
{
    this->subscriber = NULL;
}