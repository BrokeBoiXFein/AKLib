---
title: Quickstart
sidebar_position: 2
---

# Quickstart — fill out the sheet

AKLib is configured by **one file that reads like a form**: `robot_config.cpp`. Every number you need to change is marked `<-- FILL IN`. This page walks each section.

## Section 1 — drive motors

```cpp
auto leftMotors = std::make_shared<pros::MotorGroup>(
    std::initializer_list<std::int8_t>{-1, -2, -3},   // left side ports
    pros::MotorGears::blue);                          // cartridge color

auto rightMotors = std::make_shared<pros::MotorGroup>(
    std::initializer_list<std::int8_t>{4, 5, 6},
    pros::MotorGears::blue);
```

- List every motor on each side.
- **Negative port = reversed motor.** A motor is reversed if commanding it forward would drive the robot backward. On a typical tank drive one whole side ends up negated.

## Section 2 — drivetrain dimensions

```cpp
auto drivetrain = std::make_shared<aklib::TankDrive>(
    leftMotors, rightMotors,
    aklib::DriveConfig{
        .wheelDiameter = 3.25,   // inches — MEASURE it, labels lie
        .trackWidth    = 12.5,   // left↔right wheel contact distance
        .wheelRpm      = 450,    // wheel rpm AFTER external gearing
    });
```

- `wheelRpm` example: blue (600 rpm) cartridge, 36-tooth motor gear driving a 48-tooth wheel gear → `600 × 36 / 48 = 450`.
- Building an X-drive or mecanum robot? See [Drivetrains](../configuration/drivetrains) for the `HolonomicDrive` constructor.

## Section 3 — odometry sensors

Fill in what your robot **has**; delete what it doesn't:

```cpp
aklib::OdomSensors sensors{
    .imuPort = 10,
    .vertical1   = aklib::TrackingWheel(-14, 2.0, -1.25),
    .horizontal1 = aklib::TrackingWheel(15, 2.0, -2.5),
};
```

`TrackingWheel(port, wheelDiameter, offset)`:
- **port** — Rotation sensor port, negative to reverse.
- **offset** — signed distance from the tracking center, in inches. Vertical wheels: negative = left of center. Horizontal wheels: negative = behind center.

No tracking wheels? Just set `.imuPort` and delete the rest — AKLib automatically uses the drive motors for distance (works, but slips under hard acceleration). The full compatibility table is in [Sensors & odometry](../configuration/sensors).

## Section 4 — tuning

```cpp
aklib::ChassisTunings tunings{
    .lateral = {.kP = 0.07,  .kI = 0, .kD = 0.30},
    .angular = {.kP = 0.025, .kI = 0, .kD = 0.16},
    .heading = {.kP = 0.012, .kI = 0, .kD = 0.05},
};
```

The defaults move most robots safely but every robot needs its own numbers. Two paths:
- **Automated** (recommended first stop): [Auto-tuning tutorial](../tutorials/auto-tuning) — one button press, the robot tunes itself.
- **Manual:** [Manual PID tuning](../tutorials/manual-pid-tuning).

## Section 5 — done

```cpp
aklib::Chassis chassis(drivetrain, sensors, tunings);
```

Then in `main.cpp`:

```cpp
extern aklib::Chassis chassis;

void initialize() {
    chassis.calibrate();     // still robot, ~3 seconds
}

void autonomous() {
    chassis.setPose(36, 12, 90);   // where the robot starts on the field
    chassis.driveDistance(24);
    chassis.turnToHeading(0);
    chassis.moveToPoint(72, 24);
}
```

**Sanity checks before trusting it:**
1. Push the robot forward by hand → `chassis.pose().y` (or x) should increase by roughly the pushed distance.
2. Spin the robot in place → position should barely change. If it orbits, a tracking-wheel `offset` has the wrong sign.
3. `driveDistance(24)` should stop within ~1" of a tape measure's 24.

Next: [your first auton](../tutorials/first-auton).
