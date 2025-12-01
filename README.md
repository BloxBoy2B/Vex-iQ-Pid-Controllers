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
