#include <ArduinoQueue.h>
#include <MotionSegment.hpp>
#include <MotionParams.hpp>
#include <Configuration.hpp>


double getTime(ArduinoQueue<MotionSegment> segments)
{
    double queueTime = 0;

    uint8_t size = segments.item_count();

    for (byte i = 0; i < size; i++)
    {
        MotionSegment segment = segments.dequeue();

        int accSteps = (segment.targetVelocity * segment.targetVelocity - segment.initialVelocity * segment.initialVelocity) / BALL_ACCELERATION;

        if (accSteps > segment.distance >> 1)
        {
            accSteps = segment.distance >> 1;
        }

        double achievedSpeed = segment.initialVelocity * segment.initialVelocity + 2 * BALL_ACCELERATION * accSteps;
        double cruiseDistance = segment.distance - 2 * accSteps;

        double accTime = (achievedSpeed - segment.initialVelocity) / BALL_ACCELERATION;
        double cruiseTime = cruiseDistance / achievedSpeed;

        queueTime += (2 * accTime) + cruiseTime;
    }

    return queueTime;
}

MotionParams calculateMotionParams(
    double distance, // Total distance in steps
    double t_accel,  // Time for BALL_ACCELERATION phase (seconds)
    double t_const,  // Time for constant speed phase (seconds)
    double t_decel,  // Time for deceleration phase (seconds)
    double v0        // Initial velocity (steps/sec)
)
{
    // Step 1: Calculate the BALL_ACCELERATION needed to reach v_max
    // v_max = v0 + a * t_accel
    // We need to solve for 'a' and 'v_max' under the distance and time constraints

    // Distance during BALL_ACCELERATION (d_accel)
    // d_accel = v0 * t_accel + 0.5 * a * t_accel^2
    double accelTimeSquared = t_accel * t_accel;

    // Step 2: Distance covered during BALL_ACCELERATION and deceleration
    // d_total = 2 * d_accel + d_const
    double t_total = t_accel + t_const + t_decel;

    // Calculate BALL_ACCELERATION based on the constraints
    double a = (distance - v0 * t_total) / (t_accel * (t_accel + t_const));

    // Calculate the max speed reached during the constant speed phase
    double v_max = v0 + a * t_accel;

    // Return results
    return {v_max, a};
}