#include "PongStepper.hpp"

PongStepper *PongStepper::instances[2];

void PongStepper::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
	_targetPos = absolute;
	computeNewSpeed();
    }
}

void PongStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

long PongStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long PongStepper::targetPosition()
{
    return _targetPos;
}

long PongStepper::currentPosition()
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

// Subclasses can override
unsigned long PongStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
	_stepInterval = 0;
	_speed = 0.0;
	_n = 0;
    this->ITimer->stopTimer();
	return _stepInterval;
    }

    if (distanceTo > 0)
    {
	// We are anticlockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (_n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
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
	    if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
		_n = -stepsToStop; // Start deceleration
	}
	else if (_n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
		_n = -_n; // Start accceleration
	}
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
	// First step from stopped
	_cn = _c0;
	setDirection((distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW);
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
    if (_direction == DIRECTION_CCW)
	_speed = -_speed;

    this->step1(0);

    if(this->axis == Axis::X){
        this->ITimer->attachInterruptInterval(this->_stepInterval,PongStepper::runX);
    } else {
        this->ITimer->attachInterruptInterval(this->_stepInterval,PongStepper::runY);
    }

    this->shouldClear = true;
    this->shouldClearAt = micros() + _stepInterval + 5;

    return _stepInterval;
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
bool PongStepper::run()
{
	computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

PongStepper::PongStepper(uint8_t pin1, uint8_t pin2)
{
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 0.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
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

    enableOutputs();
    setDirection(DIRECTION_CCW);

    int i;
    for (i = 0; i < 4; i++)
	_pinInverted[i] = 0;
	
    // Some reasonable default
    setAcceleration(1);
    setMaxSpeed(1);
}

void PongStepper::setMaxSpeed(float speed)
{
    if (speed < 0.0)
       speed = -speed;
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

float   PongStepper::maxSpeed()
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

float   PongStepper::acceleration()
{
    return _acceleration;
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
	setDirection((speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW);
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
   step1(step);
}

void PongStepper::stepLow()
{
    setOutputPins(_direction == DIRECTION_CCW ? 0b10 : 0b00);
}

long PongStepper::stepForward()
{
    // Clockwise
    _currentPos += 1;
	step(_currentPos);
	_lastStepTime = micros();
    return _currentPos;
}

long PongStepper::stepBackward()
{
    // Counter-clockwise
    _currentPos -= 1;
	step(_currentPos);
	_lastStepTime = micros();
    return _currentPos;
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void PongStepper::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    uint8_t i;
    for (i = 0; i < numpins; i++)
	digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void PongStepper::step1(long step)
{
    (void)(step); // Unused

    setOutputPins(_direction == DIRECTION_CW ? 0b11 : 0b01); // step HIGH
}

void PongStepper::stepSingle()
{
    setOutputPins(_direction == DIRECTION_CW ? 0b11 : 0b01);
}

void PongStepper::setAxis(Axis axis)
{
    this->axis = axis;
    PongStepper::instances[static_cast<int>(axis)] = this;
}

void PongStepper::setTimer(uint8_t timerIndex)
{
    this->ITimer = new RPI_PICO_Timer(timerIndex);
}

#ifdef ARDUINO_ARCH_RP2040
void PongStepper::setTimer(RPI_PICO_Timer *iTimer)
{
    this->ITimer = iTimer;
}
#endif

bool PongStepper::runX(struct repeating_timer *t)
{
    return PongStepper::instances[0]->run();
}

bool PongStepper::runY(struct repeating_timer *t)
{
    return PongStepper::instances[1]->run();
}

void PongStepper::setDirection(StepDirection dir)
{
    this->_direction = dir;

    setOutputPins(_direction ? 0b10 : 0b00); 
}

// Prevents power consumption on the outputs
void    PongStepper::disableOutputs()
{ 
    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, LOW ^ _enableInverted);
    }
}

void PongStepper::enableOutputs()
{

    pinMode(_pin[0], OUTPUT);
    pinMode(_pin[1], OUTPUT);

    if (_enablePin != 0xff)
    {
        pinMode(_enablePin, OUTPUT);
        digitalWrite(_enablePin, HIGH ^ _enableInverted);
    }
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
    _pinInverted[2] = pin3Invert;
    _pinInverted[3] = pin4Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void PongStepper::runToPosition()
{
    while (run())
	YIELD; // Let system housekeeping occur
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
    return !(_speed == 0.0 && _targetPos == _currentPos && this->shouldClear == false);
}
