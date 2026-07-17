---
title: Your first auton
sidebar_position: 1
---

# Your first auton

Prereqs: [installed](../getting-started/installation), [config sheet filled](../getting-started/quickstart), PIDs at least roughly tuned ([auto-tuning](auto-tuning) makes that a button press).

## 1. Declare where you start

Every auton begins by teaching the robot its starting pose. Measure to the robot's **tracking center** (center of rotation):

```cpp
void autonomous() {
    chassis.setPose(36, 12, 90);   // x=36", y=12", facing "up" (+y)
```

Line the robot up against field walls/tiles the same way every match — consistency here is worth more than any tuning.

## 2. Move

```cpp
    chassis.driveDistance(24);        // one tile forward, heading held
    chassis.turnToHeading(0);         // face +x
    chassis.moveToPoint(72, 24);      // arc to field center-ish
    chassis.moveToPose(96, 48, 90);   // arrive there FACING +y
}
```

Notice you mixed relative (`driveDistance`) and absolute (`moveToPoint`) motions freely — they share the same pose. That's the core of AKLib.

## 3. React to what happened

```cpp
if (chassis.driveDistance(18, {.timeoutMs = 1200}) == aklib::ExitReason::Stalled) {
    // we reached the goal early — clamp it now
    clamp.extend();
}
```

## 4. Overlap actions with motion

```cpp
chassis.moveToPoint(72, 48, {.async = true});   // returns immediately
chassis.waitUntil(12);                          // ...until 12" remain
intake.move_voltage(12000);                     // spin up on approach
chassis.waitUntilDone();
```

## 5. Chain motions for speed

Settling costs 200–500 ms per motion. When the next motion continues in roughly the same direction, hand off early:

```cpp
chassis.moveToPoint(48, 24, {.minSpeed = 0.35, .earlyExitRange = 6});
chassis.moveToPoint(72, 48, {.minSpeed = 0.35, .earlyExitRange = 6});
chassis.moveToPoint(96, 60);   // final motion settles for real
```

`earlyExitRange` exits the motion (at speed) within that distance; `minSpeed` stops the controller from decelerating below a floor so momentum carries through. Full guide: [Motions](motions#chaining).

## A complete skeleton

```cpp
#include "aklib/api.hpp"
using namespace aklib;
using namespace aklib::literals;

extern Chassis chassis;

void autonomous() {
    chassis.setPose(36, 12, 90_deg);

    // rush the first goal, clamping on contact
    chassis.moveToPoint(36, 48, {.minSpeed = 0.4, .earlyExitRange = 8});
    if (chassis.moveToPoint(36, 60, {.maxSpeed = 0.6, .timeoutMs = 1200})
            == ExitReason::Stalled) {
        clamp.extend();
    }

    // score, then line up the next one facing the right way
    chassis.moveToPose(72, 48, 0_deg, {.reverse = true});
    chassis.turnToPoint(96, 48);
    chassis.driveDistance(20);
}
```

:::tip Practice-field debugging ritual
Print the pose and last exit reason after every motion during development:
```cpp
auto r = chassis.moveToPoint(72, 48);
auto p = chassis.pose();
printf("mtp -> %s  at (%.1f, %.1f, %.1f)\n",
       aklib::toString(r), p.x, p.y, p.thetaDeg());
```
Watching *why* motions end teaches you more than watching the robot.
:::
