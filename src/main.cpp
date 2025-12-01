#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
motor LeftMotor = motor(PORT1, false);
motor RightMotor = motor(PORT6, true);

// generating and setting random seed
void initializeRandomSeed()
{
  wait(100, msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
      xAxis + yAxis + zAxis);
  // Set the seed
  srand(seed);
}

void vexcodeInit()
{

  // Initializing random seed.
  BrainInertial.calibrate();
  waitUntil(!BrainInertial.isCalibrating());
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

float angularIntegral = 0;
float DistanceIntegral = 0;
float angularPreviousError = 0;
float linearPreviousError = 0;

// "when started" hat block
int whenStarted1()
{
  return 0;
}

#include "vex.h"

// This function turns the robot to a given heading using PID
void turnToHeadingPID(double targetHeading, double timeOut)
{
  angularIntegral = 0;
  angularPreviousError = 0;

  float error = 0;
  float speed = 0;
  float robotHeading = 0;
  float derivative = 0;
  double kp = 0.5;
  double ki = 0.2;
  double kd = 0.3;

  int threshold = 2;

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

void driveToDistancePID(double targetDistance, double headingToHold, double timeOut)
{
  DistanceIntegral = 0;
  linearPreviousError = 0;

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

int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  driveToDistancePID(300, 0, 4000);
  turnToHeadingPID(90, 2000);
  driveToDistancePID(400, 90, 4000);
  turnToHeadingPID(180, 2000);

  return 0;
}
