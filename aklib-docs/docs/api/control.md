---
title: Control primitives
sidebar_position: 7
---

# Control primitives

The building blocks under every motion — and they're all public, so you can use them for **your own mechanisms** (lifts, flywheels, arms). Headers under `aklib/control/`.

## Pid

```cpp
aklib::Pid lift(aklib::PidGains{.kP = 0.02, .kD = 0.08,
                                .integralZone = 5, .integralMax = 0.5});
lift.reset();                              // at the start of each movement
double out = lift.update(error, dt);       // once per loop; dt in seconds
```

- `integralZone`: only integrate when |error| is inside this window (0 = always).
- `integralMax`: clamp on the I-term's contribution (anti-windup).
- Unit-agnostic: maps "error units" → "output units".

## Settler + ExitConditions

```cpp
aklib::Settler settler({.smallError = 5, .smallTimeMs = 100,
                        .timeoutMs = 2000}, /*earlyExitRange=*/0);
while (true) {
    auto reason = settler.update(error, velocity, /*dtMs=*/10);
    if (reason != aklib::ExitReason::Running) break;
    ...
}
```

The exact settling logic every chassis motion uses — reuse it so your lift macros exit as robustly as your drive does. Details: [Exit conditions](../configuration/exit-conditions).

## SlewLimiter

```cpp
aklib::SlewLimiter slew(4.0);          // max +4.0 output/second
out = slew.calculate(desired, dt);
slew.reset(currentOutput);             // start ramping from a known output
```

Only limits **increases** in |output| by default — braking is never delayed.

## TrapezoidalProfile

```cpp
aklib::TrapezoidalProfile prof(distance, maxVel, maxAccel, maxDecel);
auto s = prof.at(t);        // {position, velocity, accel} at t seconds
prof.totalTime();           // planned duration
```

Great for "motion-magic"-style lift moves: plan the lift travel, feed `s.velocity` through a feedforward + position PID.

## Feedforward

```cpp
aklib::Feedforward ff({.kS = 0.9, .kV = 0.157, .kA = 0.004});
double volts = ff.calculate(velocity, accel);
```

`voltage = kS·sgn(v) + kV·v + kA·a`. Characterization procedure: [Motion profiling](../tutorials/motion-profiling).

## Example: a profiled lift macro

```cpp
void liftTo(double targetDeg) {
    aklib::TrapezoidalProfile prof(targetDeg - liftAngle(), 200, 600);
    aklib::Pid pid({.kP = 0.015, .kD = 0.04});
    aklib::Settler settler({.smallError = 2, .timeoutMs = 1500}, 0);
    const double start = liftAngle();
    uint32_t t0 = pros::millis(), now = t0;
    while (true) {
        double t = (pros::millis() - t0) / 1000.0;
        auto s = prof.at(t);
        double planned = start + s.position * aklib::sgn(targetDeg - start);
        double out = 0.9 * s.velocity / 200.0 + pid.update(planned - liftAngle(), 0.01);
        liftMotor.move_voltage(int(out * 12000));
        if (settler.update(targetDeg - liftAngle(), liftVelocity(), 10)
                != aklib::ExitReason::Running) break;
        pros::Task::delay_until(&now, 10);
    }
    liftMotor.brake();
}
```
