# VEX IQ PID Controller

A Vex IQ PID Controller that provides accurate distance and turns.

## What it does

Makes the robot turn and drive straight more accurately than just setting motor speeds. Uses PID math to calculate how fast the motors should go.

## How to use it

Put this in your main function:

```cpp
int main() {
    vexcodeInit();
    
    driveToDistancePID(300, 0);
    turnToHeadingPID(90);
    driveToDistancePID(400, 90);
    
    return 0;
}
```

**turnToHeadingPID(angle)** - turns the robot to face an angle
- example: turnToHeadingPID(180) turns to face backwards

**driveToDistancePID(distance, heading)** - drives forward a certain distance while staying straight
- example: driveToDistancePID(500, 90) drives forward 500 degrees while facing 90°

## Advanced driveToDistance class (with 4 distance sensors)

The `driveToDistance` class supports:
- X-axis wall correction using any one of 4 distance sensors.
- Y-axis distance correction that carries error into the next move.
- Customizable sensor mapping (pick which sensor is X or Y for each move).

### Quick setup

1. Create your distance sensors in your robot configuration (ports are examples).
2. Pass the sensor pointers into the `driveToDistance` constructor (or use `setDistanceSensor`).
3. Call `execute(...)` with your chosen X and Y sensor indices.

### Example configuration (robot setup)

```cpp
// Robot configuration example
brain Brain;
inertial BrainInertial = inertial();
motor LeftMotor = motor(PORT1, false);
motor RightMotor = motor(PORT6, true);

distance Distance1 = distance(PORT2);
distance Distance2 = distance(PORT3);
distance Distance3 = distance(PORT4);
distance Distance4 = distance(PORT5);
```

### Example usage

```cpp
#include "vex.h"
#include "driveToDistance.h"

using namespace vex;

int main() {
    vexcodeInit();

    // Create the controller and register your 4 sensors
    driveToDistance drivePID(0.5, 0.01, 0.3, 10, 0.3, 0.05, 0.1, 2,
                             &Distance1, &Distance2, &Distance3, &Distance4);

    // Optional tuning for wall correction
    drivePID.setWallCorrection(0.1, 15.0);

    // Scale how much of the Y error (in mm) you want to apply to the next move
    // Example: 1.0 = apply full error, 0.5 = apply half
    drivePID.setDistanceOffsetScale(1.0);

    // Drive forward to 400 mm while holding heading 0.
    // X-axis: use sensor 2 to keep 371 mm from the wall.
    // Y-axis: use sensor 1 to correct forward distance after the move.
    drivePID.execute(400, 0, 3000, 2, 371, 1, 400);

    return 0;
}
```

### How it works

The class runs two control loops at the same time:
- **Distance PID (Y)**: drives forward using motor encoders to hit `targetDistance`.
- **Heading PID**: keeps the robot pointed at `headingToHold`.

On top of those:
- **X-axis wall correction**: while driving, it reads the chosen X sensor and adds a small
  correction to heading if the robot is too close or too far from the wall.
- **Y-axis correction with offset**: when the move ends, it reads the chosen Y sensor and
  stores the error (in mm). That error becomes a **distance offset** added to the next move
  so the robot compensates for encoder drift.

### Sensor selection (X/Y)

There are 4 physical distance sensors, but only 2 are used per move:
- `xSensorIndex` = which sensor to use for wall correction (1-4).
- `ySensorIndex` = which sensor to use for distance correction (1-4).
- Use `-1` to disable an axis.

### execute(...) signature

```cpp
void execute(double targetDistance, // in mm
             double headingToHold,
             double timeOut,
             int xSensorIndex = -1,
             double xTargetDistance = 0,
             int ySensorIndex = -1,
             double yTargetDistance = 0);
```

### Tuning tips

- **wallKp** (`setWallCorrection`) controls how aggressively the robot turns toward the wall.
- **distanceOffsetScale** scales how much of the mm error to apply to the next move.
- If the robot oscillates around the wall, reduce wallKp or maxCorrection.
- If the next move over-corrects, lower distanceOffsetScale.

## What is PID

PID is a control system that uses 3 parts:
- P (proportional) - how far away you are from the target
- I (integral) - adds up errors over time to fix small mistakes
- D (derivative) - slows down before reaching the target so you dont overshoot

Its better than just setting speeds because it adjusts automatically.

## Changing the tuning

If the robot overshoots or wobbles too much, change the kp ki kd values in the code.

**Turning values:**
- kp = 0.5
- ki = 0.2
- kd = 0.3

**Driving values:**
- kp = 0.5
- ki = 0.01
- kd = 0.3

Make them bigger if the robot is too slow, smaller if its too aggressive.

## Hardware

- Left motor: PORT1
- Right motor: PORT6
- Inertial sensor in the brain

## Notes

The integral resets between each movement so old errors dont mess up new movements. The heading correction keeps the robot driving straight by adjusting left and right motor speeds slightly.
