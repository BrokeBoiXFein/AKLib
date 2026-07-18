---
title: Auto-tuner
sidebar_position: 6
---

# `aklib::autotuneAngular` / `autotuneLateral`

Header: `aklib/autotune.hpp`. Usage guide: [Auto-tuning tutorial](../tutorials/auto-tuning).

```cpp
AutotuneResult autotuneAngular(Chassis&, double kP, const AutotuneOptions& = {});
AutotuneResult autotuneLateral(Chassis&, double kP, const AutotuneOptions& = {});
```

Both are **blocking** (~30–60 s). You supply kP; the tuner finds kD by bracketing the oscillation boundary (walk kD down from a stable start / up from an oscillating one) and golden-section searching the bracket. Every candidate runs a fixed-duration raw-PID test (alternating direction) and is scored on the full transient. Per-run metrics print to the PROS terminal.

## AutotuneOptions

| Field | Default | Meaning |
|---|---|---|
| `testMagnitude` | 0 → 90° / 24″ | size of each test motion |
| `testDurationMs` | 2500 | fixed observation window per run |
| `settleBetweenMs` | 400 | pause between runs |
| `kDStart` | 0 → `kP·2` | bracket starting point |
| `kDMin` / `kDMax` | 0 → `kP·0.5` / `kP·200` | bracket floor / ceiling |
| `growth` | 1.4 | kD multiplier per bracket step |
| `refineIters` | 6 | golden-section iterations |
| `settleTol` | 0.75 | deg/in counted as "at target" |
| `settleHoldMs` | 150 | continuous ms in band = settled |
| `ssWindowMs` | 300 | tail window for steady-state error |
| `overshootTol` | 1.5 | overshoot allowed to count as stable |
| `oscTol` | 1 | sign-changes allowed to count as stable |
| `wOvershoot` | 2.0 | cost per deg/in of overshoot (kD too low) |
| `wOscillation` | 3.0 | cost per oscillation (kD too low) |
| `wRise` | 2.0 | cost per second of rise time (kD too high) |
| `wSettle` | 1.0 | cost per second of settle time |
| `wSteady` | 4.0 | cost per deg/in of steady-state error |
| `abortCheck` | — | polled kill switch; return `true` to stop |
| `verbose` | true | per-run terminal output |

## AutotuneResult

```cpp
struct AutotuneResult {
    PidGains gains;    // {your kP, kI = 0, best kD found}
    double cost;       // best cost (comparable within a session)
    bool aborted;      // kill switch fired; gains hold best-so-far
    int runsExecuted;
};
```

The tuner **never writes gains anywhere** — assign `result.gains` to `chassis.tunings().angular/.lateral` for the session and copy the printed values into `robot_config.cpp` permanently.

## Per-run metrics (`AutotuneRun`, printed when verbose)

| Metric | Measures | Penalizes |
|---|---|---|
| `overshoot` | max error past the target | kD too low |
| `oscillations` | error zero-crossings | kD too low |
| `riseTimeMs` | first reach of the settle band | kD too high (early deceleration) |
| `settleTimeMs` | entered the band and stayed `settleHoldMs` | slow convergence |
| `steadyError` | mean \|error\| over the last `ssWindowMs` | stalling short |

## Notes

- Tests run raw PID → drivetrain with **no slew and no exit conditions**; the full `testDurationMs` window is always observed. This is deliberate — exit-condition-based scoring lets sluggish tunes end early and score well.
- The lateral tuner holds heading with `chassis.tunings().heading` during each shuttle, so tune angular before lateral.
- Keep kI = 0 while tuning (see the [tutorial](../tutorials/auto-tuning#why-ki-stays-0-during-tuning)); add a small kI afterward only for steady-state error.
- Don't run chassis motions while the tuner is running — it commands the drivetrain directly.
