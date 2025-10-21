#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

class PPongStepper {
public:
    PPongStepper() : running(false), alarm_id(-1) {}

    void start();
    void stop();
    int64_t runAsync();  // your existing logic
    void scheduleNextStep(int64_t delay_us);

private:
    void onAlarm();  // called from IRQ

public:
    TimerPhase phase;
    int _direction;
    int _futureDirection;
    int _currentPos;
    int _stepInterval;
    uint32_t _lastStepTime;

private:
    bool running;
    int alarm_id;
};

//--------------------------------------------------
// Start the stepper
void PongStepper::start() {
    if (running) return;

    alarm_id = hardware_alarm_claim_unused(true);  // claim a free hardware alarm
    hardware_alarm_set_callback(alarm_id, [](uint alarm_num) {
        // We assume single stepper; in multi-stepper version, pass pointer
        instance->onAlarm();  
    });

    running = true;
    scheduleNextStep(20);  // first step delay
}

// Stop the stepper
void PongStepper::stop() {
    if (!running) return;

    hardware_alarm_cancel(alarm_id);
    hardware_alarm_unclaim(alarm_id);
    running = false;
}

// Schedule the next step at absolute time
void PongStepper::scheduleNextStep(int64_t delay_us) {
    absolute_time_t t = delayed_by_us(get_absolute_time(), delay_us);
    hardware_alarm_set_target(alarm_id, t);
}

//--------------------------------------------------
// Called inside IRQ context
void PongStepper::onAlarm() {
    int64_t interval = runAsync();  // your existing logic

    if (interval > 0) {
        scheduleNextStep(interval);
    } else {
        stop();
    }
}

//--------------------------------------------------
// Your existing runAsync() remains almost identical
int64_t PongStepper::runAsync()
{
    switch (this->phase)
    {
        case TimerPhase::STEP:
        {
            if (_direction)
                _currentPos += 1;
            else
                _currentPos -= 1;

            step(_currentPos);
            _lastStepTime = time_us_32();
            computeNewSpeed();

            this->phase = TimerPhase::CLEAR;
            return 20;
        }
        case TimerPhase::CLEAR:
        {
            clear();

            if(_futureDirection != _direction)
            {
                this->phase = TimerPhase::UPDATE_DIRECTION;
                return 20;
            }

            this->phase = TimerPhase::STEP;
            return max(0, _stepInterval - 20);
        }
        case TimerPhase::UPDATE_DIRECTION:
        {
            updateDirection();
            this->phase = TimerPhase::STEP;
            return max(0, _stepInterval - 40);
        }
        default:
            return 0;
    }
}
