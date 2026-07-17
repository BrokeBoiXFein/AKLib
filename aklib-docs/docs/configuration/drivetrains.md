---
title: Drivetrains
sidebar_position: 1
---

# Drivetrains

AKLib supports two drivetrain families behind one interface, so **every motion works on both**. Motions output a `ChassisSpeeds {vx, vy, omega}`; the drivetrain mixes it to wheel voltages.

## Tank (differential) — the VEX default

```cpp
auto drivetrain = std::make_shared<aklib::TankDrive>(
    leftMotors, rightMotors,
    aklib::DriveConfig{
        .wheelDiameter = 3.25,
        .trackWidth    = 12.5,
        .wheelRpm      = 450,
    });
```

Tank drives cannot strafe: the `vy` component is physically impossible and ignored. `driveDistance`, turns, `moveToPoint`, boomerang, pure pursuit and **swing turns** all work.

### Driver control

```cpp
void opcontrol() {
    pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
    auto tank = std::static_pointer_cast<aklib::TankDrive>(chassis.drivetrain());
    while (true) {
        // arcade: throttle + turn, each in [-1, 1]
        tank->arcade(ctrl.get_analog(ANALOG_LEFT_Y) / 127.0,
                     ctrl.get_analog(ANALOG_RIGHT_X) / 127.0);
        pros::delay(20);
    }
}
```

`tank->tank(left, right)` is also available for tank-stick control.

## Holonomic — X-drive & mecanum

```cpp
auto drivetrain = std::make_shared<aklib::HolonomicDrive>(
    frontLeft, frontRight, backLeft, backRight,   // one MotorGroup per corner
    aklib::HolonomicDrive::Kind::Mecanum,          // or Kind::XDrive
    aklib::DriveConfig{
        .wheelDiameter = 4.0,
        .trackWidth    = 13.0,
        .wheelRpm      = 200,
    });
```

Motor direction convention: **each corner group should spin its wheel "robot-forward" when commanded positive** — negate ports until that's true.

What changes on holonomic:

| Behavior | Tank | Holonomic |
|---|---|---|
| `moveToPoint` | arcs, robot must face (or back) the target | strafes straight at the point; heading independent |
| `moveToPose` | boomerang curve | strafes at the carrot while rotating to the target heading |
| `follow` | pure pursuit curvature steering | velocity vector at the lookahead; heading follows the path (or `.face`) |
| `.face` param | ignored | heading (deg) to hold while translating |
| `swingToHeading` | pivots on one side | falls back to `turnToHeading` |

:::warning Holonomic odometry needs a horizontal wheel
A holonomic robot strafes on purpose, and forward-only sensing can't see that motion. Configure a **horizontal tracking wheel** (see [Sensors](sensors)) or your pose will be wrong the first time you strafe.
:::

### Field-centric driver control

```cpp
auto holo = std::static_pointer_cast<aklib::HolonomicDrive>(chassis.drivetrain());
holo->drive(strafe, forward, turn,
            chassis.pose().theta, /*fieldCentric=*/true);
```

## DriveConfig reference

| Field | Meaning | Notes |
|---|---|---|
| `wheelDiameter` | driven wheel diameter, in | Measure; anti-static wheels run small |
| `trackWidth` | left↔right contact distance, in | Also used by pure pursuit's curvature mix |
| `wheelRpm` | wheel rpm after external gearing | cartridge rpm × (motor gear ÷ wheel gear) |
| `feedforward` | `{kS, kV, kA}` volts | Only for `.profiled` motions — see [Motion profiling](../tutorials/motion-profiling) |
