#include "vex.h"
#include "turnToHeadingPID.h"

using namespace vex;

// Constructor
TurnToHeadingPID::TurnToHeadingPID(float proportional, float integral, float derivative, int errorThreshold)
{
    kp = proportional;
    ki = integral;
    kd = derivative;
    threshold = errorThreshold;
    angularIntegral = 0;
    angularPreviousError = 0;
}

// Destructor
TurnToHeadingPID::~TurnToHeadingPID()
{
}

// Execute the PID control loop
void TurnToHeadingPID::execute(double targetHeading, double timeOut)
{
    // Reset integral and previous error
    angularIntegral = 0;
    angularPreviousError = 0;

    float error = 0;
    float speed = 0;
    float robotHeading = 0;
    float derivative = 0;

    timer myTimer;

    while (true)
    {
        robotHeading = BrainInertial.heading(degrees);

        error = targetHeading - robotHeading;
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        angularIntegral += error;

        if (angularIntegral > 400)
            angularIntegral = 400;
        if (angularIntegral < -400)
            angularIntegral = -400;

        if (fabs(error) < 2)
            angularIntegral = 0;

        derivative = error - angularPreviousError;
        angularPreviousError = error;

        speed = (kp * error) + (ki * angularIntegral) + (kd * derivative);

        // Constrain speed (optional, prevents crazy motor values)
        if (speed > 50)
            speed = 50;
        if (speed < -50)
            speed = -50;

        // Spin left/right motors to turn
        LeftMotor.spin(forward, speed, percent);
        RightMotor.spin(forward, -speed, percent);

        // Break loop if heading is close enough
        if (fabs(error) <= threshold || myTimer.time(msec) >= timeOut)
        {
            LeftMotor.stop(brake);
            RightMotor.stop(brake);
            break;
        }

        // Short delay
        this_thread::sleep_for(20);
    }
}
