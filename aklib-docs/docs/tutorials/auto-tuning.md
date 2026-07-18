---
title: Auto-tuning
sidebar_position: 4
---

# Auto-tuning

AKLib's auto-tuner finds your **kD** for a **kP you choose**, by bracketing the oscillation boundary and then golden-section searching it. The division of labor matters: kP sets how aggressive the motion is (a judgment call you make in two minutes), and kD — the tedious one to hand-tune — is found by measurement.

Each candidate kD gets a **fixed-duration test run** (2.5 s by default) driven by a raw PID loop, and the whole transient is scored:

```
cost = 2.0·overshoot + 3.0·oscillations + 2.0·rise_s + 1.0·settle_s + 4.0·steadyErr
```

- **Overshoot & oscillations** punish kD too *low*.
- **Rise time** (how long until the robot first reaches the target band) punishes kD too *high* — an overdamped tune decelerates early and crawls in.
- **Steady-state error** catches tunes that stall short of the target.

Because every run is observed for the full window, a sluggish tune can't hide behind an early "close enough" exit — it just racks up rise-time and steady-error cost.

## Step 0 — choose kP by hand (2 minutes)

Bind a plain test motion to a button and raise kP until the motion is **brisk and overshoots a little** ("turns hard and rings once or twice"). Too low and the tuner can only find a slow tune; too high and no kD can stabilize it.

## Step 1 — run the tuner

```cpp
#include "aklib/autotune.hpp"

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    aklib::AutotuneOptions opt;
    // kill switch: hold B to stop (best-so-far result is kept)
    opt.abortCheck = [&] { return master.get_digital(DIGITAL_B); };

    while (true) {
        if (master.get_digital_new_press(DIGITAL_X)) {
            auto r = aklib::autotuneAngular(chassis, /*kP=*/0.03, opt);
            if (!r.aborted) chassis.tunings().angular = r.gains;
        }
        if (master.get_digital_new_press(DIGITAL_Y)) {
            auto r = aklib::autotuneLateral(chassis, /*kP=*/0.08, opt);
            if (!r.aborted) chassis.tunings().lateral = r.gains;
        }
        pros::delay(20);
    }
}
```

It's a **blocking** routine (~30–60 s). The robot alternates direction every run — turns +90° then −90°, or shuttles forward then back — so it never walks across the field. Give the lateral tuner ~2× the test distance of clear runway. **Tune angular first**; the lateral test uses the heading PID to shuttle straight.

## What you'll see (PROS terminal)

```
[tune angular] kP=0.0300  bracket [0.0150, 6.0000] from kD=0.0600
[tune angular BRACKET] kD=0.0600  over=3.10 osc=3 rise=610ms settle=2500ms ss=0.41  cost=...
[tune angular ASCEND ] kD=0.0840  over=1.20 osc=1 rise=590ms settle=980ms ss=0.22  cost=...
[tune angular REFINE ] kD=0.1310  over=0.40 osc=0 rise=640ms settle=760ms ss=0.15  cost=...
...
[tune angular] DONE — copy into ChassisTunings:
    .kP = 0.0300, .kI = 0, .kD = 0.1412    (cost 3.81, 14 runs)
```

Phases: **BRACKET** tests your starting kD (default `2·kP`); **DESCEND/ASCEND** walks kD down (if stable — hunting the smallest, fastest stable value) or up (if oscillating) until it straddles the oscillation boundary; **REFINE** golden-section searches that bracket. **Copy the final numbers into `robot_config.cpp`** — the tuner never writes anything itself.

## Why kI stays 0 during tuning

AKLib's PID only integrates inside `integralZone` — exactly the settling region the tuner measures — so a nonzero kI adds its own slow limit cycle and phase lag that corrupts the overshoot/oscillation metrics. Tune kD with kI = 0, then add a small kI afterward *only* if a persistent steady-state gap remains.

## Options worth knowing

```cpp
aklib::AutotuneOptions opt{
    .testMagnitude = 60,      // deg (angular) / in (lateral); 0 = auto 90/24
    .testDurationMs = 2500,   // must comfortably outlast settling
    .kDStart = 0,             // 0 = auto: kP * 2
    .refineIters = 6,         // more = finer kD, longer session
    .settleTol = 0.75,        // the "at target" band, deg/in
    .overshootTol = 1.5,      // what "oscillation gone" means in the bracket
};
```

The cost weights (`wOvershoot`, `wOscillation`, `wRise`, `wSettle`, `wSteady`) are exposed too — raise `wOvershoot` if you'd rather land conservative, raise `wRise` if the results feel lazy.

## Good practice

- Same tiles you compete on, reasonably charged battery.
- Re-run after meaningful robot changes (mass, wheels, ratio).
- `testMagnitude` should resemble your real motions — tuning 90° turns is the right default; tune at 45° if your auton is all small adjustments.
- The tuner drives raw PID output with **no slew**, so expect the test motions to snap harder than your normal motions do. That's intentional — it exposes the true dynamics.
