#ifndef TURNTOHEADINGPID_H
#define TURNTOHEADINGPID_H

#include "vex.h"

class driveToDistance
{
private:
    float distanceIntegral;
    float distancePreviousError;
    float kp;
    float ki;
    float kd;
    int threshold;
    float headingError;
    float headingPreviousError;
    float headingCorrection;
    float kpHeading;
    float kiHeading;
    float kdHeading;
    int headingToHoldThreshold = 

public:
    driveToDistance(float proportional = 0.5, float integral = 0.2, float derivative = 0.3, int errorThreshold = 2);
    ~driveToDistance();

    void execute(double targetHeading, double timeOut);
};

#endif
