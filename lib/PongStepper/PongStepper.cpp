#include "PongStepper.hpp"

void PongStepper::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
        _targetPos = absolute;
        computeNewSpeed();
    }
}

void PongStepper::updateDirection()
{
    if (this->_futureDirection != this->_direction)
    {
        this->_direction = this->_futureDirection;
        if(this->_direction ^ _pinInverted[1])
        {
            sio_hw->gpio_set = (1 << this->_pin[1]);
        } else {
            sio_hw->gpio_clr =(1 << this->_pin[1]);
        }
    }
}

void PongStepper::externalUpdateDirection(bool dir)
{
    this->_futureDirection = dir;
    this->_direction = dir;

    if(this->_direction ^ _pinInverted[1])
    {
        sio_hw->gpio_set = (1 << this->_pin[1]);
    } else {
        sio_hw->gpio_clr =(1 << this->_pin[1]);
    }
}

void PongStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean PongStepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval && !this->shouldClear)
    {
        return false;
    }

    unsigned long time = time_us_32();
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

long PongStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long PongStepper::targetPosition()
{
    return _targetPos;
}

volatile long PongStepper::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void PongStepper::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

void PongStepper::clear()
{
    this->shouldClear = false;
    sio_hw->gpio_clr = (1 << this->_pin[0]);
}

void PongStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
    this->stepperStepsToStop = stepsToStop;

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
boolean PongStepper::run()
{
    if (runSpeed())
        computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

PongStepper::PongStepper(uint8_t pin1, uint8_t pin2)
{
    pinMode(pin1,OUTPUT);
    pinMode(pin2,OUTPUT);

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

void PongStepper::setMaxSpeed(float speed)
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

float PongStepper::maxSpeed()
{
    return _maxSpeed;
}

void PongStepper::setAcceleration(float acceleration)
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

void PongStepper::setSpeed(float speed)
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

float PongStepper::speed()
{
    return _speed;
}

// Subclasses can override
void PongStepper::step(long step)
{
    (void)(step); // Unused

    // _pin[0] is step, _pin[1] is direction

    sio_hw->gpio_set = (1 << this->_pin[0]);

    this->shouldClear = true;
}

void PongStepper::externalStep(long step)
{
    (void)(step);
    
    if (this->_direction == true)
    {
        // Clockwise
        this->_currentPos += 1;
    }
    else
    {
        // Anticlockwise
        this->_currentPos -= 1;
    }

    this->step(this->_currentPos);
}

void PongStepper::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void PongStepper::setEnablePin(uint8_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
}

void PongStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void PongStepper::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert)
{
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void PongStepper::runToPosition()
{
    while (run());
}

boolean PongStepper::runSpeedToPosition()
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
void PongStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void PongStepper::stop()
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

bool PongStepper::isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos && !this->shouldClear);
}