---
title: Exit conditions
sidebar_position: 3
---

# Exit conditions

Deciding when a motion is *done* matters as much as the motion itself. Every AKLib motion runs the same exit logic and **returns the reason it ended**.

## The conditions

A motion exits when **any** of these fires:

| Reason | Fires when | Typical meaning |
|---|---|---|
| `Settled` | \|error\| < `smallError` for `smallTimeMs` | Clean success |
| `CloseEnough` | \|error\| < `bigError` for `bigTimeMs` | Near miss — stop wasting time |
| `Stalled` | speed ≈ 0 for `stallTimeMs` while away from target | Hit a wall / goal / robot |
| `EarlyExit` | \|error\| < `earlyExitRange` (per-call param) | Chained into next motion |
| `Timeout` | `timeoutMs` elapsed | Something went wrong |
| `Interrupted` | another motion or `cancelMotion()` took over | You changed plans |

## Defaults & overrides

The chassis keeps two default sets — lateral (inches) and angular (degrees):

```cpp
aklib::ChassisTunings tunings{
    // defaults shown; part of ChassisTunings
    .lateralExits = {.smallError = 1.0, .smallTimeMs = 150,
                     .bigError = 3.0,   .bigTimeMs = 500,
                     .stallVelocity = 0.1, .stallTimeMs = 350,
                     .timeoutMs = 5000},
    .angularExits = {.smallError = 1.5, .bigError = 5.0, .stallVelocity = 3.0},
};
```

Override per call when a motion needs different tolerances:

```cpp
// sloppy fast mid-route point:
chassis.moveToPoint(48, 48, {.exits = aklib::ExitConditions{
    .smallError = 2.5, .smallTimeMs = 80, .timeoutMs = 1200}});

// just the timeout (most common):
chassis.driveDistance(30, {.timeoutMs = 1500});
```

## Use the return value

```cpp
using aklib::ExitReason;

switch (chassis.driveDistance(30, {.timeoutMs = 1500})) {
    case ExitReason::Settled:
    case ExitReason::CloseEnough: break;            // carry on
    case ExitReason::Stalled:  scoreNow(); break;   // we hit the goal early
    case ExitReason::Timeout:  bailOut(); break;    // recovery branch
    default: break;
}
```

`aklib::toString(reason)` gives a printable name — log it during practice runs; it's free debugging.

## Wall resets with stall exits

The stall exit turns walls into calibration tools:

```cpp
// Ram the wall behind us, then trust the wall, not the drift:
chassis.driveDistance(-20, {.maxSpeed = 0.4, .timeoutMs = 1200});
// (exits Stalled when we're planted against the wall)
auto p = chassis.pose();
chassis.setPose(p.x, 8.0, p.thetaDeg());   // we KNOW y at this wall = 8"
```

## Tuning guidance

- `smallError` 1″ / 1.5° suits most robots. Tighter costs settle time and rarely buys real accuracy.
- Raise `smallTimeMs` if you see the robot "declare victory" while still rocking.
- `stallTimeMs = 0` disables stall detection for motions that legitimately push (goal rushes).
- **Never disable the timeout.** A hung auton is worse than a wrong one.
