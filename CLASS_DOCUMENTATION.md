# PID Controller Class Documentation

## What is a Class?

A **class** is a blueprint for creating objects in C++. Think of it like a template that defines:
- **Data** (variables/properties) that the object will store
- **Functions** (methods) that the object can perform

### Why Use Classes?

1. **Organization**: Groups related data and functions together
2. **Reusability**: Create multiple instances with different settings
3. **Encapsulation**: Keeps internal details private and only exposes what's needed
4. **Maintainability**: Easier to update and debug code in one place

---

## TurnToHeadingPID Class Structure

### Header File (`include/turnToHeadingPID.h`)

This file **declares** the class - it tells the compiler what the class looks like but doesn't implement it yet.

```cpp
#ifndef TURNTOHEADINGPID_H
#define TURNTOHEADINGPID_H

#include "vex.h"

class TurnToHeadingPID
{
private:
    // Private member variables - only accessible inside the class
    float angularIntegral;
    float angularPreviousError;
    float kp;
    float ki;
    float kd;
    int threshold;

public:
    // Public methods - can be called from outside the class
    TurnToHeadingPID(float proportional = 0.5, float integral = 0.2, 
                     float derivative = 0.3, int errorThreshold = 2);
    ~TurnToHeadingPID();
    
    void execute(double targetHeading, double timeOut);
};

#endif
```

#### Parts Explained:

**1. Include Guards** (`#ifndef`, `#define`, `#endif`)
- Prevents the header from being included multiple times
- Required for all header files

**2. Private Section**
- Variables that only the class can access
- Stores PID tuning values (kp, ki, kd)
- Stores state (angularIntegral, angularPreviousError)
- Stores settings (threshold)

**3. Public Section**
- **Constructor**: `TurnToHeadingPID(...)` - Called when you create an object
  - Has default parameters (= 0.5, = 0.2, etc.)
  - Initializes the PID tuning values
- **Destructor**: `~TurnToHeadingPID()` - Called when object is destroyed
  - Cleans up resources (though we don't need it here)
- **execute()**: The method that runs the PID control loop

---

### Implementation File (`src/PID_Controllers/turnToHeadingPID.cpp`)

This file **implements** the class - it contains the actual code for each method.

```cpp
#include "vex.h"
#include "turnToHeadingPID.h"

using namespace vex;

// Constructor - runs when you create an object
TurnToHeadingPID::TurnToHeadingPID(float proportional, float integral, 
                                   float derivative, int errorThreshold)
{
    kp = proportional;      // Set P gain
    ki = integral;          // Set I gain
    kd = derivative;        // Set D gain
    threshold = errorThreshold;
    angularIntegral = 0;    // Reset integral
    angularPreviousError = 0;
}

// Destructor - runs when object is destroyed
TurnToHeadingPID::~TurnToHeadingPID()
{
    // Nothing to clean up
}

// Execute method - performs the PID control
void TurnToHeadingPID::execute(double targetHeading, double timeOut)
{
    // ... PID control loop code ...
}
```

#### Parts Explained:

**1. Scope Resolution Operator** (`::`)
- `TurnToHeadingPID::` tells the compiler "this function belongs to the TurnToHeadingPID class"
- Required when implementing methods outside the class declaration

**2. Constructor Implementation**
- Takes parameters and assigns them to member variables
- Initializes all necessary values

**3. execute() Method**
- This is where the actual PID control logic lives
- Can access all private member variables (kp, ki, kd, etc.)

---

## How to Use the Class

### Step 1: Include the Header in main.cpp

```cpp
#include "vex.h"
#include "turnToHeadingPID.h"
```

### Step 2: Create an Instance (Object)

You have several options:

**Option A: Use Default Values**
```cpp
TurnToHeadingPID turnController;
// Uses: kp=0.5, ki=0.2, kd=0.3, threshold=2
```

**Option B: Custom Tuning Values**
```cpp
TurnToHeadingPID turnController(0.8, 0.1, 0.4, 1);
// kp=0.8, ki=0.1, kd=0.4, threshold=1
```

**Option C: Partial Custom Values** (thanks to default parameters)
```cpp
TurnToHeadingPID turnController(0.6);
// kp=0.6, ki=0.2 (default), kd=0.3 (default), threshold=2 (default)
```

### Step 3: Use the Object

```cpp
// Turn to 90 degrees with 3 second timeout
turnController.execute(90, 3000);

// Turn to 180 degrees with 5 second timeout
turnController.execute(180, 5000);

// Turn to 45 degrees with 2 second timeout
turnController.execute(45, 2000);
```

---

## Complete Example in main.cpp

```cpp
#include "vex.h"
#include "turnToHeadingPID.h"

using namespace vex;

int main() {
    vexcodeInit();
    
    // Create a PID controller with custom tuning
    TurnToHeadingPID turnPID(0.6, 0.15, 0.35, 2);
    
    // Turn to 90 degrees
    turnPID.execute(90, 3000);
    
    wait(1, seconds);
    
    // Turn back to 0 degrees
    turnPID.execute(0, 3000);
    
    wait(1, seconds);
    
    // Turn to 180 degrees
    turnPID.execute(180, 3000);
    
    return 0;
}
```

---

## Advantages Over Function-Based Approach

### Old Way (Function):
```cpp
turnToHeadingPID(90, 3000);  // Global variables, one set of tuning
```
- Can only have ONE set of PID tuning values
- State persists globally (can cause bugs)
- Harder to test and debug

### New Way (Class):
```cpp
TurnToHeadingPID fastTurn(1.0, 0.1, 0.5);
TurnToHeadingPID slowTurn(0.3, 0.05, 0.2);

fastTurn.execute(90, 2000);   // Quick, aggressive turn
slowTurn.execute(90, 5000);   // Slow, smooth turn
```
- Can have MULTIPLE controllers with different tuning
- Each instance has its own state (no interference)
- Clear, organized, and easier to maintain

---

## Key Concepts

### Encapsulation
The private variables (kp, ki, kd, etc.) are hidden from outside code. You can only interact with them through the public methods (constructor and execute). This prevents accidental modification.

### Constructor
Automatically called when you create an object. Sets up initial values so the object is ready to use.

### Methods
Functions that belong to a class. They can access and modify the object's private data.

### Instance
Each object created from a class is called an "instance". You can create multiple instances, each with its own data.

---

## Next Steps

1. **Test the class** in your main.cpp file
2. **Tune the PID values** by creating instances with different parameters
3. **Create more PID classes** for other movements (drive straight, etc.)
4. **Add features** like:
   - Setter methods to change kp, ki, kd after creation
   - Getter methods to read current values
   - Reset method to clear integral and previous error
