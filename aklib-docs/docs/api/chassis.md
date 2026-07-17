---
title: Chassis
sidebar_position: 1
---

# `aklib::Chassis`

The class your auton talks to. Owns the drivetrain, the odometry task, and every motion. Header: `aklib/chassis.hpp` (or just `aklib/api.hpp`).

## Construction

```cpp
Chassis(std::shared_ptr<Drivetrain> drivetrain,
        OdomSensors sensors,
        ChassisTunings tunings = {});
```

Tank drives with no vertical tracking wheel automatically get drive-motor odometry. See [Quickstart](../getting-started/quickstart) for the full setup sheet.

## Setup & state

| Method | Returns | Notes |
|---|---|---|
| `calibrate()` | `bool` | Call once in `initialize()`; blocks ~3 s; robot must be still. `false` = IMU failed. |
| `setPose(x, y, thetaDeg)` | — | Teach the robot where it is. Top of every auton. |
| `pose()` | `Pose` | `{x, y, theta(rad)}`; use `.thetaDeg()`. Thread-safe. |
| `odometry()` | `Odometry&` | speed(), angularSpeedDeg(), setPose(...) |
| `drivetrain()` | `shared_ptr<Drivetrain>` | cast to TankDrive/HolonomicDrive for driver control |
| `tunings()` | `ChassisTunings&` | live gain access (used by the auto-tuner) |

## Motions

All motions take an optional [`MotionParams`](motion-params) and return an [`ExitReason`](../configuration/exit-conditions).

| Method | Does |
|---|---|
| `driveDistance(dist, params)` | Straight line along current heading; negative = backward; heading actively held |
| `turnToHeading(deg, params)` | In-place turn to absolute heading |
| `turnToPoint(x, y, params)` | Face a field point (`.reverse` = face away); target re-computed live |
| `swingToHeading(deg, side, params)` | Turn with one side locked (`SwingSide::Left/Right`); tank only |
| `moveToPoint(x, y, params)` | Drive to a field point (arc on tank, strafe on holonomic) |
| `moveToPose(x, y, deg, params)` | Boomerang: arrive at the point facing `deg` |
| `follow(path, params)` | Pure pursuit over a [`Path`](path); ends with a settling moveToPoint |

## Async control

| Method | Does |
|---|---|
| `waitUntilDone()` | Block until the current motion ends; returns its ExitReason |
| `waitUntil(remaining)` | Block until remaining error < `remaining` (in or deg) |
| `cancelMotion()` | Current motion returns `Interrupted` |
| `isMoving()` | Is any motion running? |
| `lastExitReason()` | ExitReason of the most recent motion |

Starting any motion cancels the one already running — exactly one motion commands the drive at a time.

## ChassisTunings

```cpp
struct ChassisTunings {
    PidGains lateral;          // inches → output
    PidGains angular;          // degrees → output (turns)
    PidGains heading;          // degrees → output (drive-straight hold)
    ExitConditions lateralExits;
    ExitConditions angularExits;
    double slewRate = 5.0;     // output increase per second; 0 = off
};
```
