#include "vex.h"
#include "driveToDistance.h"
using namespace vex;

// Constructor
driveToDistance::driveToDistance(float distanceProportional, float distanceIntegral, float distanceDerivative, int distanceErrorThreshold, float headingProportional, float headingIntegral, float headingDerivative, int headingErrorThreshold)
{
    kp = distanceProportional;
    ki = distanceIntegral;
    kd = distanceDerivative;
    headingErrorThreshold = headingToHoldThreshold;

    kpHeading = headingProportional;
    kiHeading = headingIntegral;
    kdHeading = headingDerivative;
    
    
 

    threshold = distanceErrorThreshold;


   
}

// Destructor
driveToDistance::~TurnToHeadingPID()
{
}

// Execute the PID control loop
void driveToDistance::execute(double targetDistance, double headingToHold, double timeOut)
{
    // Reset integral and previous error
    distanceIntegral = 0;
    headingIntegral = 0;
    

  float error = 0;
  float speed = 0;
  float derivative = 0;
  double kp = 0.5;
  double ki = 0.01;
  double kd = 0.3;

  float headingError = 0;
  float headingIntegral = 0;
  float headingPreviousError = 0;
  float headingDerivative = 0;
  float headingCorrection = 0;

  double kpHeading = 0.3;
  double kiHeading = 0.05;
  double kdHeading = 0.1;

  double headingToHoldThreshold = 2;

  timer myTimer;

  LeftMotor.setPosition(0, degrees);
  RightMotor.setPosition(0, degrees);

  // You can adjust this threshold for how close is 'good enough'
  int threshold = 10;

  while (true)
  {
    // Get average position of both motors (in degrees)
    float currentDistance = (LeftMotor.position(degrees) + RightMotor.position(degrees)) / 2.0;

    error = targetDistance - currentDistance;

    DistanceIntegral += error;

    if (DistanceIntegral > 400)
      DistanceIntegral = 400;
    if (DistanceIntegral < -400)
      DistanceIntegral = -400;

    if (fabs(error) < 5)
      DistanceIntegral = 0;

    derivative = error - linearPreviousError;
    linearPreviousError = error;

    speed = (kp * error) + (ki * DistanceIntegral) + (kd * derivative);

    if (speed > 50)
      speed = 50;
    if (speed < -50)
      speed = -50;

    float currentHeading = BrainInertial.heading(degrees);
    headingError = headingToHold - currentHeading;

    if (headingError > 180)
      headingError -= 360;
    if (headingError < -180)
      headingError += 360;

    headingIntegral += headingError;
    if (headingIntegral > 100)
      headingIntegral = 100;
    if (headingIntegral < -100)
      headingIntegral = -100;
    if (fabs(headingError) < 2)
      headingIntegral = 0;

    headingDerivative = headingError - headingPreviousError;
    headingPreviousError = headingError;

    headingCorrection = (kpHeading * headingError) + (kiHeading * headingIntegral) + (kdHeading * headingDerivative);

    if (headingCorrection > 20)
      headingCorrection = 20;
    if (headingCorrection < -20)
      headingCorrection = -20;

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
    

    // Short delay
    this_thread::sleep_for(20);
  }
}
