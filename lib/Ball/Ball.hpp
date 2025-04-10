#pragma once

#include <Configuration.hpp>
#include <PongStepper.hpp>
#include <Point.hpp>

class Ball
{
public:
    static Ball *instance;

    Point limits;

    void init(size_t stepX, size_t dirX, size_t stepY, size_t dirY);

    /// @brief Sets the target position
    /// @param x
    /// @param y
    void setposition(int x, int y);

    /// @brief Overrides the current position tracked by the stepper
    /// @param x
    /// @param y
    void setCurrentPosition(int x, int y);

    /// @brief Starts the stopping process
    /// @attention Doesnt stop the stepper immediatelly
    void stop();

    /// @brief Calibrates the game setting the min and max limits
    void calibrate();

    /// @brief Gets the current cartesian position of the ball from corexy/hbot positioning
    /// @return Poit which contains an X and an Y integer
    volatile Point getPosition();

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
    void shootDeg(uint16_t degrees, bool isBounce);

    /// @brief Shoots a ball in an angle and sets its position to the limits
    /// @param radians defines the angle of shooting
    void shootAngle(float radians, bool isBounce);

    /// @brief Calculates the mirrored angle (Example: 30 -> 150, 210 -> 330)
    /// @param angle in degrees
    /// @return returns the mirrored value
    uint16_t inverseAngle(int16_t angle);

    void resetSpeeds();

    void run();

    bool isRunning();

    long getX();
    long getY();
    long stepsToSop();

    void printInfo();

    long getPossibleStopPoint();

    PongStepper *subscriber(){
        return this->_yStepper;
    }

    ~Ball();

    PongStepper *_xStepper = NULL;
private:
    PongStepper *_yStepper = NULL;

    int dx;
    int dy;
};