#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"
#include "turnToHeadingPID.h"
#include "driveToDistance.h"

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
distance Distance1 = distance(PORT2);
distance Distance2 = distance(PORT3);
distance Distance3 = distance(PORT4);
distance Distance4 = distance(PORT5);

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

int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  TurnToHeadingPID turnPID(0.6, 0.15, 0.35, 2);
  driveToDistance drivePID(0.5, 0.01, 0.3, 10, 0.3, 0.05, 0.1, 2,
                           &Distance1, &Distance2, &Distance3, &Distance4);
  drivePID.setGearRatio("36:12");
  drivePID.setWallCorrection(0.1, 15.0);
  drivePID.setDistanceOffsetScale(1.0);

  drivePID.execute(300, 0, 4000, &Distance2, 371, &Distance1, 300);
  turnPID.execute(90, 2000);
  drivePID.execute(400, 90, 4000, &Distance2, 371, &Distance1, 400);
  turnPID.execute(180, 2000);

  return 0;
}
