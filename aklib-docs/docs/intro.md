---
id: intro
title: AKLib
slug: /
sidebar_position: 1
---

# AKLib

**AKLib** is a motion and tracking library for VEX V5, written in C++ on [PROS 4](https://pros.cs.purdue.edu/v5/pros-4/index.html). It combines the two things teams usually have to pick between:

- **LemLib-style absolute motions** — `moveToPoint`, `moveToPose` (boomerang), `turnToPoint`, pure pursuit — powered by continuous odometry.
- **EZ-Template-style relative motions and ergonomics** — "drive 24 inches straight", swing turns, robust exit conditions, and a fast tuning workflow (including a **fully automated PID tuner**).

The trick that makes the combination work: *every* motion, relative or absolute, reads and updates the **same odometry pose**. Drive 24 inches, then call `moveToPoint(72, 48)` — it just works, because the "drive 24 inches" was tracked on the field the whole time.

## Highlights

```cpp
#include "aklib/api.hpp"
using namespace aklib::literals;

void autonomous() {
    chassis.setPose(36, 12, 90_deg);

    chassis.driveDistance(24);                       // relative, heading held
    chassis.moveToPoint(72, 48);                     // absolute, same pose
    chassis.moveToPose(96, 24, 0_deg);               // arrive facing a heading
    chassis.follow(rushPath, {.lookahead = 12});     // pure pursuit

    // every motion tells you WHY it ended:
    if (chassis.driveDistance(20) == aklib::ExitReason::Stalled) {
        // we hit the goal — score!
    }
}
```

- Tank **and holonomic** (X-drive / mecanum) drivetrains behind one interface
- Odometry with **any sensor combo** — tracking wheels, drive encoders, one or two IMUs — with sensible automatic fallbacks
- **Motion chaining**: early exit + minimum speed + async motions with `waitUntil`
- **Auto PID tuning**: the robot runs test motions and converges on gains itself
- **Imperial by default, metric whenever you want**: `60_cm`, `1.2_m`, `1.57_rad` convert at compile time
- A **visual path planner** (`planner/aklib-planner.html`) that exports ready-to-paste C++
- A **brain-screen auton selector** (robodash-style cards, SD-card persistence) built in

## Where to go next

1. [Installation](getting-started/installation) — add AKLib to a PROS project (5 minutes).
2. [Quickstart](getting-started/quickstart) — fill out the config sheet and drive.
3. [Your first auton](tutorials/first-auton) — a guided routine using every motion type.
4. [Auto-tuning](tutorials/auto-tuning) — let the robot tune its own PIDs.

:::info What needs what
Every feature's sensor requirements are spelled out in [Configuration → Sensors & odometry](configuration/sensors). The short version: an IMU alone gets you started; adding tracking wheels makes everything better.
:::
