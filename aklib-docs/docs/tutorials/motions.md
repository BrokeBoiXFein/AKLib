---
title: Motions in depth
sidebar_position: 2
---

# Motions in depth

Every motion: takes a [`MotionParams`](../api/motion-params) struct of optional knobs, runs at 10 ms, respects [exit conditions](../configuration/exit-conditions), and returns an `ExitReason`.

## driveDistance

```cpp
chassis.driveDistance(24);                       // forward 24"
chassis.driveDistance(-12);                      // back up 12"
chassis.driveDistance(24, {.maxSpeed = 0.5});    // gentle
```

Drives along the **current heading**, which is actively held by the heading PID the whole way (resists drivetrain asymmetry and defense). Internally the target is an absolute field point computed from the current pose — so odometry stays valid and you can follow with any absolute motion.

## turnToHeading / turnToPoint

```cpp
chassis.turnToHeading(90);                        // absolute heading (math deg)
chassis.turnToHeading(aklib::compass(90));        // compass fans
chassis.turnToPoint(96, 96);                      // aim at a field point
chassis.turnToPoint(96, 96, {.reverse = true});   // aim the BACK at it
```

`turnToPoint` recomputes the target angle from the live pose every loop, so it stays correct even if the robot gets bumped mid-turn.

## swingToHeading (tank only)

```cpp
chassis.swingToHeading(45, aklib::SwingSide::Right);  // right side locked
```

Pivots around one locked side instead of the center — useful for hooking around game elements or preserving wall alignment. The robot translates while it swings; odometry tracks it. On holonomic drives this falls back to `turnToHeading`.

## moveToPoint

```cpp
chassis.moveToPoint(72, 48);
chassis.moveToPoint(24, 48, {.reverse = true});   // drive there backwards
```

The odometry payoff motion: steers toward the point while scaling drive power by alignment (`cos` of the heading error), producing a smooth arc. Inside ~7″ it stops steering and lets the distance controller finish (prevents the end-of-motion spin). Position errors **don't compound** — every call self-corrects to absolute coordinates.

On holonomic drives the robot strafes directly at the point; pass `.face = 45` to hold a heading while translating.

## moveToPose (boomerang)

```cpp
chassis.moveToPose(96, 24, 0);                  // arrive FACING 0 deg
chassis.moveToPose(96, 24, 0, {.dlead = 0.4});  // tighter curve
```

Arrives at a point **facing a chosen heading**, in one curved motion, by chasing a "carrot" point pulled back from the target along the arrival heading. `dlead` (0–1, default 0.6) sets curve width: higher = wider, more deliberate approach; 0 degenerates to `moveToPoint`.

Boomerang's arrival heading is approximate (a few degrees). If the next action needs an exact heading, follow with a quick `turnToHeading`.

## follow (pure pursuit)

```cpp
chassis.follow(path, {.lookahead = 12});
```

See the [pure pursuit tutorial](pure-pursuit) and the [planner](path-planner).

## Chaining

```cpp
chassis.moveToPoint(48, 24, {.minSpeed = 0.35, .earlyExitRange = 6});
chassis.moveToPoint(72, 48);
```

- `earlyExitRange` — exit within this error, **without stopping the motors**; the next motion inherits the momentum.
- `minSpeed` — output floor so the controller can't decelerate to a crawl before the handoff.
- A chained handoff skips settling, so expect a couple inches of slop at the junction; end chains with a normally-settling motion (or any absolute motion — they self-correct).

## Async motions

```cpp
chassis.moveToPose(72, 48, 90, {.async = true});  // returns immediately

chassis.waitUntil(10);        // block until 10" (or 10 deg) remain
intake.move_voltage(12000);

chassis.waitUntilDone();      // block until finished; returns ExitReason
// or: chassis.cancelMotion();  // abort (motion returns Interrupted)
```

Starting any new motion automatically cancels the running one. Exactly one motion commands the drivetrain at a time — enforced internally, no task juggling on your side.

## Parameter quick reference

Common to all motions (see [MotionParams](../api/motion-params) for everything):

| Param | Default | Effect |
|---|---|---|
| `.maxSpeed` | 1.0 | output cap (fraction of full power) |
| `.minSpeed` | 0 | output floor for chaining |
| `.reverse` | false | do it backwards |
| `.earlyExitRange` | 0 | chained exit distance (in/deg) |
| `.timeoutMs` | 0 → default | per-motion time cap |
| `.async` | false | run in background |
| `.brakeAtEnd` | true | brake vs coast on finish |

:::caution Designated initializer order
C++ requires designated initializers in declaration order — `{.reverse = true, .lookahead = 12}` compiles, `{.lookahead = 12, .reverse = true}` does not. Order: `maxSpeed, minSpeed, reverse, earlyExitRange, timeoutMs, slewRate, async, brakeAtEnd, dlead, lookahead, profiled, face, ...`.
:::
