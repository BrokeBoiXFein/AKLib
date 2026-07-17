---
title: IMU calibration
sidebar_position: 5
---

# IMU calibration

The IMU owns your heading, and heading accuracy is the single biggest lever on odometry quality: 1° of heading error becomes roughly 0.4″ of position error per 24″ driven. Ten minutes here pays for itself all season.

## Mounting

- Rigid mount near the center of rotation — standoffs into well-braced structure.
- Away from motor heat and high-vibration members (flywheel towers, intakes).
- Flat. The V5 IMU calibrates in whatever orientation it wakes up in, but a flat, repeatable mount removes a variable.

## Startup calibration

`chassis.calibrate()` (call once in `initialize()`):

- Takes ~2–3 s. **The robot must be completely still** — put it down before turning the program on, hands off.
- Returns `false` on failure (bad port, damaged sensor, robot moved). AKLib falls back to encoder heading and keeps working — but fix the root cause; encoder heading is much worse.

```cpp
void initialize() {
    if (!chassis.calibrate()) {
        pros::lcd::print(0, "IMU FAILED — check port/wiring");
    }
}
```

:::warning Match-day habit
Place the robot on the field *first*, then start the program. A robot calibrating while being carried starts every match with a garbage heading.
:::

## Measuring the scale factor

Every IMU has a small constant scale error (reads e.g. 358.6° after a true 360°). AKLib corrects it with `imuScale` — measure once per sensor:

1. Mark the robot's exact heading (tape on robot + tile).
2. Spin the robot **slowly by hand exactly 10 full rotations**, ending on the mark.
3. Read the raw rotation:

```cpp
// temporary test snippet
pros::Imu imu(10);
imu.reset(true);
// ...spin 10 turns by hand...
printf("raw rotation: %.2f (want 3600)\n", imu.get_rotation());
```

4. Compute and set:

```
imuScale = 3600.0 / |raw reading|
```

```cpp
aklib::OdomSensors sensors{
    .imuPort = 10,
    .imuScale = 1.0042,   // e.g. raw read 3585 → 3600/3585
};
```

Typical values land between 0.99 and 1.01. Larger than that: re-run the test (you probably missed the mark) or suspect the sensor.

## Two IMUs

```cpp
.imuPort = 10, .imuPort2 = 20,
```

Readings are averaged, roughly halving random drift. Worth a port on long skills runs; measure `imuScale` for the pair together (same 10-spin test, using the averaged output — or just average each sensor's individual scale).

## Verifying the result

Drive a long auton on a practice field, then check heading against a tile edge. A healthy setup holds heading within ~1° over a 60 s run. If drift is much worse: check mounting rigidity first, scale second, sensor health third.
