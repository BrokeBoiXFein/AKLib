---
title: Auto-tuner
sidebar_position: 6
---

# `aklib::autotuneLateral` / `autotuneAngular`

Header: `aklib/autotune.hpp`. Usage guide: [Auto-tuning tutorial](../tutorials/auto-tuning).

```cpp
AutotuneResult autotuneLateral(Chassis&, const AutotuneOptions& = {});
AutotuneResult autotuneAngular(Chassis&, const AutotuneOptions& = {});
```

Both are **blocking** calls that repeatedly run a test motion (lateral: ±`testDistance` shuttle; angular: ±`testAngleDeg` spins), score each run, and coordinate-descend ("twiddle") on kP and kD. Results print to the PROS terminal after every run.

## AutotuneOptions

| Field | Default | Meaning |
|---|---|---|
| `iterations` | 8 | twiddle sweeps (each ≈ 2–4 runs per gain) |
| `testDistance` | 24 | lateral test length, in |
| `testAngleDeg` | 90 | angular test size, deg |
| `runTimeoutMs` | 2500 | per-run cap |
| `tuneKi` | false | also tune kI (rarely worth it on drivetrains) |
| `verbose` | true | print per-run telemetry |
| `overshootWeight` | 40 | scoring: penalty × overshoot² |
| `settleWeight` | 3 | scoring: penalty × settle seconds |

## AutotuneResult

```cpp
struct AutotuneResult {
    PidGains gains;     // best set found — copy into robot_config.cpp
    double score;       // lower = better (comparable within a session)
    bool success;       // false only if every run diverged
    int runsExecuted;
};
```

The tuner **never writes gains anywhere** — you assign `result.gains` yourself (to `chassis.tunings().lateral/.angular` for the session, and into `robot_config.cpp` permanently).

## Scoring

```
score = ITAE + overshootWeight · overshoot² + settleWeight · settle_seconds
ITAE  = ∫ t·|error| dt      (punishes error that lingers)
```

Safety: runs whose error exceeds 1.8× the test size are cancelled and scored divergent; gains are clamped ≥ 0; the seed comes from the chassis's current tunings.
