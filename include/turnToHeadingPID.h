#ifndef TURNTOHEADINGPID_H
#define TURNTOHEADINGPID_H

#include "vex.h"

class TurnToHeadingPID
{
private:
    float angularIntegral;
    float angularPreviousError;
    float kp;
    float ki;
    float kd;
    int threshold;

public:
    TurnToHeadingPID(float proportional = 0.5, float integral = 0.2, float derivative = 0.3, int errorThreshold = 2);
    ~TurnToHeadingPID();
    
    void execute(double targetHeading, double timeOut);
};

#endif
