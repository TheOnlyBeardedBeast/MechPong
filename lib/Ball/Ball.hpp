#pragma once

#include <Configuration.hpp>
#include <PongStepper.hpp>
#include <Point.hpp>

class Ball
{
public:
    static Ball *instance;

    Point limits;

    /// @brief Sets the motors which moves the balls
    /// @param _steppers
    void setMotors(PongStepper *xStepper, PongStepper *yStepper);

    /// @brief Sets the target position
    /// @param x
    /// @param y
    void setposition(int x, int y);

    /// @brief Overrides the current position tracked by the stepper
    /// @param x
    /// @param y
    void setCurrentPosition(int x, int y);

    /// @brief Sets an absolute position to go to with a custom speed
    /// @param x
    /// @param y
    /// @param speed
    void setposition(int x, int y, int speed);

    void setposition(int x, int y, int speed, int startSpeed, int endSpeed);

    /// @brief Starts the stopping process
    /// @attention Doesnt stop the stepper immediatelly
    void stop();

    /// @brief Stops the motor immediatelly
    void stopNow();

    /// @brief Calibrates the game setting the min and max limits
    void calibrate();

    /// @brief Gets the current cartesian position of the ball from corexy/hbot positioning
    /// @return Poit which contains an X and an Y integer
    Point getPosition();

    long getCenterRelativePosition();

    /// @brief Sets the ball ball position to the playfield center
    void center();

    /// @brief Center the ball on the playfield and waits until the ball is positioned
    void runCenter();

    /// @brief Checks if the ball has any distance to make
    /// @return return true or false
    bool needsToMove();

    /// @brief angle the ball used last time
    uint16_t lastAngle = 0;

    /// @brief Bounces onn the limits
    void bounce();

    /// @brief Shoots a ball in an angle and sets its position to the limits
    /// @param degrees defines the angle of shooting
    void shootDeg(uint16_t degrees);

    /// @brief Shoots a ball in an angle and sets its position to the limits
    /// @param radians defines the angle of shooting
    void shootAngle(float radians);

    /// @brief Calculates the mirrored angle (Example: 30 -> 150, 210 -> 330)
    /// @param angle in degrees
    /// @return returns the mirrored value
    uint16_t inverseAngle(int16_t angle);

    PongStepper *subscriber(){
        return this->_yStepper;
    }

private:
    PongStepper *_xStepper = nullptr;
    PongStepper *_yStepper = nullptr;

    int dx;
    int dy;
};