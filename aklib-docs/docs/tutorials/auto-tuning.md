---
title: Auto-tuning
sidebar_position: 4
---

# Auto-tuning

AKLib can tune its own PIDs: the robot repeatedly runs a test motion, **scores** each candidate gain set (error-over-time + overshoot + settle time), and walks the gains toward the best score (the *twiddle* / coordinate-descent algorithm) with safety rails.

## What you need

- Working odometry ([sensors](../configuration/sensors)) — the tuner measures with it.
- Space: the lateral tuner shuttles ±24″ along a line — give it **~6 ft of clear runway**. The angular tuner spins in place.
- A charged battery (gains tuned at 40% battery behave differently at 100%).
- The PROS terminal open to watch progress: `pros terminal`.

## Run it

Bind it to controller buttons in `opcontrol`:

```cpp
#include "aklib/autotune.hpp"

void opcontrol() {
    pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
    while (true) {
        if (ctrl.get_digital_new_press(DIGITAL_X)) {
            auto r = aklib::autotuneAngular(chassis);
            if (r.success) chassis.tunings().angular = r.gains;
        }
        if (ctrl.get_digital_new_press(DIGITAL_Y)) {
            auto r = aklib::autotuneLateral(chassis);
            if (r.success) chassis.tunings().lateral = r.gains;
        }
        pros::delay(20);
    }
}
```

**Tune angular first** — the lateral test relies on heading hold to shuttle straight.

## What you'll see

```
[autotune angular] baseline  kP=0.0250 kD=0.1600  score=311.2
[autotune angular] iter 1/8  kP=0.0350 kD=0.1600  best=268.4
[autotune angular] iter 2/8  kP=0.0350 kD=0.2240  best=224.9
...
[autotune angular] DONE. Copy into your ChassisTunings:
    .kP = 0.0392, .kI = 0.0000, .kD = 0.2417   (score 197.3)
```

**Copy the final numbers into `robot_config.cpp`** — the tuner deliberately does not persist anything itself, so you always know what your robot runs on.

A session is roughly 25–50 test runs, 3–8 minutes. Expect the robot to look *worse* during parts of it — trying bad gains on purpose is how it learns where the good ones are.

## Options

```cpp
aklib::AutotuneOptions opt{
    .iterations   = 8,      // more = finer convergence, longer session
    .testDistance = 24.0,   // lateral test length (in)
    .testAngleDeg = 90.0,   // angular test size (deg)
    .runTimeoutMs = 2500,
    .tuneKi       = false,  // usually leave off for drivetrains
};
auto r = aklib::autotuneLateral(chassis, opt);
```

Tune with the tests your autons resemble: if your routes are mostly short 12″ hops, set `testDistance = 12`.

## Safety rails (what stops a runaway)

- Any run whose error grows past 1.8× the test size is **cancelled and scored as divergent** — bad gains get abandoned, not repeated.
- Every run has a hard timeout.
- Gains are clamped non-negative.
- `result.success == false` means every run diverged — your *seed* gains are unstable; drop kP by ~4× and rerun.

## Good-practice checklist

- Re-run after any meaningful robot change (weight, wheels, gear ratio, motor count).
- Run on the same tile surface you compete on.
- After tuning, sanity-check with [manual-tuning symptoms](manual-pid-tuning#symptoms-table) — the auto-tuner finds a good *local* optimum; your eyes confirm it's the right kind of good.
