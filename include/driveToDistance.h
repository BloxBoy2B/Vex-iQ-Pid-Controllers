#ifndef DRIVETODISTANCE_H
#define DRIVETODISTANCE_H

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
    float headingIntegral;
    float headingPreviousError;
    float kpHeading;
    float kiHeading;
    float kdHeading;
    int headingToHoldThreshold;
    float wallKp;
    float wallMaxCorrection;
    float distanceOffset;
    float distanceOffsetScale;
    float gearRatio;
    vex::distance *distanceSensors[4];

    vex::distance *getDistanceSensor(int sensorIndex) const;
    float clamp(float value, float minValue, float maxValue) const;
    float readDistanceMm(int sensorIndex) const;
    float readDistanceMm(vex::distance *sensor) const;

public:
    driveToDistance(float distanceProportional = 0.5, float distanceIntegralGain = 0.2, float distanceDerivative = 0.3, int distanceErrorThreshold = 2,
                    float headingProportional = 0.3, float headingIntegralGain = 0.05, float headingDerivative = 0.1, int headingErrorThreshold = 2,
                    vex::distance *sensor1 = nullptr, vex::distance *sensor2 = nullptr, vex::distance *sensor3 = nullptr, vex::distance *sensor4 = nullptr);
    ~driveToDistance();

    void setDistanceSensor(int slot, vex::distance *sensor);
    void setWallCorrection(float proportional, float maxCorrection = 15.0);
    void setDistanceOffsetScale(float scale);
    void setGearRatio(int motorTeeth, int wheelTeeth);
    void setGearRatio(const char *ratioText);
    void execute(double targetDistance, double headingToHold, double timeOut,
                 int xSensorIndex = -1, double xTargetDistance = 0,
                 int ySensorIndex = -1, double yTargetDistance = 0);
    void execute(double targetDistance, double headingToHold, double timeOut,
                 vex::distance *xSensor, double xTargetDistance,
                 vex::distance *ySensor, double yTargetDistance);
};

#endif
