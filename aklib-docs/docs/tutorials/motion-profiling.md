---
title: Motion profiling & feedforward
sidebar_position: 8
---

# Motion profiling & feedforward

Plain PID motions *react* to error. A **motion profile** plans the whole velocity curve before moving (accelerate → cruise → brake), and **feedforward** converts that plan directly into voltage — the PID only cleans up leftovers. The result: motions that take the *same time, every time*, with no overshoot drama.

In AKLib this is opt-in per motion:

```cpp
chassis.driveDistance(48, {.profiled = true});
```

`.profiled` requires feedforward constants; without them the motion silently falls back to PID.

## Step 1 — characterize feedforward (once per robot)

The model: `voltage = kS·sgn(v) + kV·v + kA·a`

**Measure kS and kV** with a slow voltage ramp. Temporary test code:

```cpp
// Logs (voltage, steady-speed) pairs — run on ~10 ft of clear tiles.
void characterize() {
    auto tank = std::static_pointer_cast<aklib::TankDrive>(chassis.drivetrain());
    for (int mv = 1000; mv <= 8000; mv += 500) {
        tank->tank(mv / 12000.0, mv / 12000.0);
        pros::delay(1200);                       // settle at steady speed
        printf("%d, %.2f\n", mv, chassis.odometry().speed());
    }
    tank->stop();
}
```

Paste the CSV into any spreadsheet and fit a line of **voltage (V) vs speed (in/s)**:
- **kV = slope**
- **kS = intercept** (the voltage where speed crosses zero)

Sanity check: kV ≈ `12 / top_speed`. A 450 rpm, 3.25″ drive tops out ≈ 76 in/s → kV ≈ 0.157.

**kA**: start at 0. If profiled motions lag at the start and surge at the end, raise it in steps of 0.002 until the robot tracks the ramp cleanly.

```cpp
aklib::DriveConfig{
    ...
    .feedforward = {.kS = 0.9, .kV = 0.157, .kA = 0.004},
}
```

## Step 2 — use it

```cpp
chassis.driveDistance(48, {.profiled = true});
chassis.driveDistance(48, {.maxSpeed = 0.7, .profiled = true});  // gentler cruise
```

Under the hood: a trapezoidal profile is planned for the distance (accel to cruise in ~0.4 s, brake ~1.5× gentler), each loop feeds the planned velocity/acceleration through feedforward, and the lateral PID corrects planned-vs-actual position. Heading hold runs as usual.

## When to use profiled vs plain PID

| Situation | Use |
|---|---|
| Long straight rushes where repeatable timing matters | **profiled** |
| Short positioning hops (< ~1 ft) | plain PID (profile overhead isn't worth it) |
| Carrying game objects up high (tip risk) | profiled with low `.maxSpeed` |
| You haven't characterized yet | plain PID — it's the default for a reason |

## Battery note

Feedforward constants are measured in volts, but V5 motor commands are effectively battery-relative. Characterize at a realistic match battery level, and re-check constants if your battery habits change dramatically. (Slight day-to-day variation is normal and the correction PID absorbs it.)
