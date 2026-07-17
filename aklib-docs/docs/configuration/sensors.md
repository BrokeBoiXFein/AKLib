---
title: Sensors & odometry
sidebar_position: 2
---

# Sensors & odometry

Odometry is the background task (10 ms) that tracks the robot's pose. `OdomSensors` is a **menu**: fill in what you have, AKLib picks the best strategy automatically.

## What needs what

| You want | You need |
|---|---|
| `driveDistance`, `turnToHeading`, swings | drive motors; IMU strongly recommended |
| `moveToPoint`, `moveToPose`, `turnToPoint` | working odometry (any row below) |
| `follow` (pure pursuit) | working odometry |
| Accurate tracking through bumps & defense | vertical **and** horizontal tracking wheels |
| Holonomic motions | horizontal tracking wheel (mandatory) |
| `.profiled` motions | feedforward constants (no extra sensors) |
| Auto-tuner | working odometry |

## Sensor combos, best first

| Setup | Heading | Forward | Sideways slip | Verdict |
|---|---|---|---|---|
| IMU + vertical wheel + horizontal wheel | ✅ best | ✅ best | ✅ seen | **Recommended** |
| IMU + vertical wheel | ✅ best | ✅ best | ❌ blind | Great for tank |
| IMU only (drive motors fill in) | ✅ best | ⚠️ slips under load | ❌ blind | Fine to start; use slew |
| 2 vertical wheels (no IMU) | ⚠️ ok | ✅ best | ❌ blind | Fallback only |

Rules AKLib applies automatically:
- **Heading:** IMU if present (two IMUs are averaged), else two vertical wheels, else drive-motor differential.
- **Forward:** vertical wheel(s) if present, else the average of the drive sides.
- **Sideways:** horizontal wheel if present, else assumed zero.

## TrackingWheel construction

```cpp
// 1) V5 Rotation sensor (best) — negative port = reversed
aklib::TrackingWheel(-14, 2.0, -1.25);

// 2) ADI quadrature encoder
auto enc = std::make_shared<pros::adi::Encoder>('A', 'B');
aklib::TrackingWheel(enc, 2.75, 3.0);

// 3) Drive motors as a tracking wheel (AKLib does this for you when you
//    leave vertical1 empty on a tank drive — you rarely write this yourself)
aklib::TrackingWheel(leftMotors, 3.25, -6.25, 450);
```

### The offset — read this twice

`offset` is the signed distance (inches) from the **tracking center** (usually the robot's center of rotation) to the wheel, measured **perpendicular to the wheel's rolling direction**:

- **Vertical wheel** (rolls forward/back): negative = mounted **left** of center, positive = right.
- **Horizontal wheel** (rolls left/right): negative = mounted **behind** center, positive = in front.

**Verification:** spin the robot in place. The pose (x, y) should stay nearly still. If the position orbits in a circle, an offset has the wrong sign or a bad magnitude.

### Direction

A wheel is **reversed** (negate the port) if pushing the robot **forward** (vertical wheel) or **to the right** (horizontal wheel) makes its `distance()` reading go down.

## IMU options

```cpp
aklib::OdomSensors sensors{
    .imuPort  = 10,
    .imuPort2 = 20,      // optional 2nd IMU — readings averaged, ~halves drift
    .imuScale = 1.003,   // optional scale correction, see IMU calibration tutorial
    ...
};
```

`chassis.calibrate()` blocks ~3 s while the IMU calibrates — **the robot must be completely still.** It returns `false` if calibration failed (odometry falls back to encoder heading; check wiring).

## Runtime API

```cpp
aklib::Pose p = chassis.pose();     // {x, y, theta(rad)}; p.thetaDeg() for degrees
chassis.setPose(72, 24, 180);       // teach the robot where it is
chassis.odometry().speed();         // filtered speed, in/s
chassis.odometry().angularSpeedDeg();
```

`setPose` is how you: declare the auton start pose, apply wall-alignment resets, or integrate any external correction.
