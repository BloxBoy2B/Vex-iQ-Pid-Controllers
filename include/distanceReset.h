#ifndef TURNTOHEADINGPID_H
#define TURNTOHEADINGPID_H

#include "vex.h"

class distanceReset
{
private:
    float distance1Offset();
    float distance2Offset();
    float distance3Offset();
    float distance4Offset();


public:
    TurnToHeadingPID(float proportional = 0.5, float integral = 0.2, float derivative = 0.3, int errorThreshold = 2);
    ~TurnToHeadingPID();
    
    void execute(double targetHeading, double timeOut);
};

#endif
