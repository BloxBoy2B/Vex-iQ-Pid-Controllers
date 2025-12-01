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

float angularError;
float targetHeading;
float angularIntegral = 0;
float targetDistance;
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
void turnToHeadingPID(double targetHeading)
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
    if (fabs(error) <= threshold)
    {
      LeftMotor.stop();
      RightMotor.stop();
      break;
    }

    // Short delay
    this_thread::sleep_for(20);
  }
}

void driveToDistancePID(double targetDistance)
{
  DistanceIntegral = 0;
  linearPreviousError = 0;
  
  float error = 0;
  float speed = 0;
  float derivative = 0;
  double kp = 0.5;
  double ki = 0.01;
  double kd = 0.3;

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

    // Spin both motors forward to drive straight
    LeftMotor.spin(forward, speed, percent);
    RightMotor.spin(forward, speed, percent);

    // Break loop if distance is close enough
    if (fabs(error) <= threshold)
    {
      LeftMotor.stop();
      RightMotor.stop();
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

  driveToDistancePID(300);
  turnToHeadingPID(90);

  return 0;
}
