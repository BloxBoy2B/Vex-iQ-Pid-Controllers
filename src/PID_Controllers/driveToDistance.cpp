#include "vex.h"
#include "driveToDistance.h"

using namespace vex;

driveToDistance::driveToDistance(float distanceProportional, float distanceIntegralGain, float distanceDerivative, int distanceErrorThreshold,
                                 float headingProportional, float headingIntegralGain, float headingDerivative, int headingErrorThreshold,
                                 vex::distance *sensor1, vex::distance *sensor2, vex::distance *sensor3, vex::distance *sensor4)
{
    kp = distanceProportional;
    ki = distanceIntegralGain;
    kd = distanceDerivative;
    threshold = distanceErrorThreshold;

    kpHeading = headingProportional;
    kiHeading = headingIntegralGain;
    kdHeading = headingDerivative;
    headingToHoldThreshold = headingErrorThreshold;

    distanceIntegral = 0;
    distancePreviousError = 0;
    headingIntegral = 0;
    headingPreviousError = 0;

    wallKp = 0.1f;
    wallMaxCorrection = 15.0f;
    distanceOffset = 0.0f;
    distanceOffsetScale = 1.0f;
    gearRatio = 1.0f;

    distanceSensors[0] = sensor1;
    distanceSensors[1] = sensor2;
    distanceSensors[2] = sensor3;
    distanceSensors[3] = sensor4;
}

driveToDistance::~driveToDistance()
{
}

vex::distance *driveToDistance::getDistanceSensor(int sensorIndex) const
{
    if (sensorIndex < 1 || sensorIndex > 4)
    {
        return nullptr;
    }
    return distanceSensors[sensorIndex - 1];
}

float driveToDistance::clamp(float value, float minValue, float maxValue) const
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

float driveToDistance::readDistanceMm(int sensorIndex) const
{
    vex::distance *sensor = getDistanceSensor(sensorIndex);
    if (sensor == nullptr)
    {
        return -1.0f;
    }
    return sensor->objectDistance(mm);
}

float driveToDistance::readDistanceMm(vex::distance *sensor) const
{
    if (sensor == nullptr)
    {
        return -1.0f;
    }
    return sensor->objectDistance(mm);
}

void driveToDistance::setDistanceSensor(int slot, vex::distance *sensor)
{
    if (slot < 1 || slot > 4)
    {
        return;
    }
    distanceSensors[slot - 1] = sensor;
}

void driveToDistance::setWallCorrection(float proportional, float maxCorrection)
{
    wallKp = proportional;
    wallMaxCorrection = maxCorrection;
}

void driveToDistance::setDistanceOffsetScale(float scale)
{
    distanceOffsetScale = scale;
}

void driveToDistance::setGearRatio(int motorTeeth, int wheelTeeth)
{
    if (motorTeeth <= 0 || wheelTeeth <= 0)
    {
        return;
    }
    gearRatio = static_cast<float>(motorTeeth) / static_cast<float>(wheelTeeth);
}

void driveToDistance::setGearRatio(const char *ratioText)
{
    if (ratioText == nullptr)
    {
        return;
    }

    const char *separator = strchr(ratioText, ':');
    if (separator == nullptr)
    {
        return;
    }

    int motorTeeth = atoi(ratioText);
    int wheelTeeth = atoi(separator + 1);
    setGearRatio(motorTeeth, wheelTeeth);
}

void driveToDistance::execute(double targetDistance, double headingToHold, double timeOut,
                              int xSensorIndex, double xTargetDistance,
                              int ySensorIndex, double yTargetDistance)
{
    vex::distance *xSensor = getDistanceSensor(xSensorIndex);
    vex::distance *ySensor = getDistanceSensor(ySensorIndex);
    execute(targetDistance, headingToHold, timeOut, xSensor, xTargetDistance, ySensor, yTargetDistance);
}

void driveToDistance::execute(double targetDistance, double headingToHold, double timeOut,
                              vex::distance *xSensor, double xTargetDistance,
                              vex::distance *ySensor, double yTargetDistance)
{
    distanceIntegral = 0;
    distancePreviousError = 0;
    headingIntegral = 0;
    headingPreviousError = 0;

    float error = 0;
    float speed = 0;
    float derivative = 0;

    float headingError = 0;
    float headingDerivative = 0;
    float headingCorrection = 0;

    timer myTimer;

    LeftMotor.setPosition(0, degrees);
    RightMotor.setPosition(0, degrees);

    const float degreesPerMm = (360.0f * gearRatio) / (static_cast<float>(M_PI) * 200.0f);
    double adjustedTargetDistance = (targetDistance + distanceOffset) * degreesPerMm;

    while (true)
    {
        float currentDistance = (LeftMotor.position(degrees) + RightMotor.position(degrees)) / 2.0f;
        error = adjustedTargetDistance - currentDistance;

        distanceIntegral += error;
        distanceIntegral = clamp(distanceIntegral, -400.0f, 400.0f);
        if (fabs(error) < 5)
        {
            distanceIntegral = 0;
        }

        derivative = error - distancePreviousError;
        distancePreviousError = error;

        speed = (kp * error) + (ki * distanceIntegral) + (kd * derivative);
        speed = clamp(speed, -50.0f, 50.0f);

        float currentHeading = BrainInertial.heading(degrees);
        headingError = headingToHold - currentHeading;

        if (headingError > 180)
        {
            headingError -= 360;
        }
        if (headingError < -180)
        {
            headingError += 360;
        }

        headingIntegral += headingError;
        headingIntegral = clamp(headingIntegral, -100.0f, 100.0f);
        if (fabs(headingError) < 2)
        {
            headingIntegral = 0;
        }

        headingDerivative = headingError - headingPreviousError;
        headingPreviousError = headingError;

        headingCorrection = (kpHeading * headingError) + (kiHeading * headingIntegral) + (kdHeading * headingDerivative);
        headingCorrection = clamp(headingCorrection, -20.0f, 20.0f);

        float xDistance = readDistanceMm(xSensor);
        if (xDistance >= 0)
        {
            float xError = static_cast<float>(xTargetDistance) - xDistance;
            float xCorrection = clamp(wallKp * xError, -wallMaxCorrection, wallMaxCorrection);
            headingCorrection += xCorrection;
        }

        float leftSpeed = speed + headingCorrection;
        float rightSpeed = speed - headingCorrection;

        LeftMotor.spin(forward, leftSpeed, percent);
        RightMotor.spin(forward, rightSpeed, percent);

        if ((fabs(error) <= threshold && fabs(headingError) <= headingToHoldThreshold) || myTimer.time(msec) >= timeOut)
        {
            LeftMotor.stop(brake);
            RightMotor.stop(brake);
            break;
        }

        this_thread::sleep_for(20);
    }

    float yDistance = readDistanceMm(ySensor);
    if (yDistance >= 0)
    {
        float yError = static_cast<float>(yTargetDistance) - yDistance;
        distanceOffset += yError * distanceOffsetScale;
    }
}
