---
title: Odometry & sensors
sidebar_position: 3
---

# `aklib::Odometry` / `OdomSensors` / `TrackingWheel`

Headers: `aklib/odometry.hpp`, `aklib/sensors/tracking_wheel.hpp`. Concepts and combos: [Sensors & odometry](../configuration/sensors).

## OdomSensors

```cpp
struct OdomSensors {
    int imuPort = 0;        // 0 = none
    int imuPort2 = 0;       // optional 2nd IMU (averaged)
    double imuScale = 1.0;  // heading scale correction

    std::optional<TrackingWheel> vertical1;    // forward-rolling wheel
    std::optional<TrackingWheel> vertical2;    // opposite side (optional)
    std::optional<TrackingWheel> horizontal1;  // sideways-rolling wheel

    // auto-filled by Chassis for tank drives with no vertical1:
    std::optional<TrackingWheel> driveLeft, driveRight;
};
```

## TrackingWheel

```cpp
// V5 Rotation sensor (negate port to reverse)
TrackingWheel(int rotationPort, double wheelDiameter, double offset);

// ADI quadrature encoder
TrackingWheel(std::shared_ptr<pros::adi::Encoder>, double wheelDiameter, double offset);

// Drive motors as odometry input
TrackingWheel(std::shared_ptr<pros::MotorGroup>, double wheelDiameter,
              double offset, double wheelRpm);
```

| Method | Returns |
|---|---|
| `distance()` | inches rolled since start (signed) |
| `reset()` | zero the sensor |
| `offset()` | configured mounting offset |
| `isMotorBased()` | true for drive-motor wheels |

Offset sign rules (and how to verify them) are in [Sensors → the offset](../configuration/sensors#the-offset--read-this-twice).

## Odometry

Owned by the chassis; you normally touch it through `chassis.odometry()`.

| Method | Notes |
|---|---|
| `calibrate()` | starts the 10 ms tracking task; blocks for IMU calibration; `false` on IMU failure |
| `pose()` | current `Pose`, thread-safe |
| `setPose(x, y, thetaDeg)` / `setPose(Pose)` | overwrite the estimate |
| `speed()` | filtered field speed, in/s |
| `angularSpeedDeg()` | filtered turn rate, deg/s |
| `ready()` | has calibrate() completed? |

## Implementation notes

- Arc-based update (Pilons 5225A method) at a fixed 10 ms period via `delay_until` — no timing drift.
- Heading source priority: IMU(s) → dual vertical wheels → drive differential.
- The pose is guarded by a mutex; reads copy a consistent snapshot.
- `speed()`/`angularSpeedDeg()` are low-pass filtered (α = 0.3) — they feed stall detection.
