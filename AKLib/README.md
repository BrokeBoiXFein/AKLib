# AKLib

A motion & tracking library for VEX V5 on [PROS 4](https://pros.cs.purdue.edu/v5/pros-4/index.html), combining LemLib-style pose-based motions with EZ-Template-style relative motions and tuning ergonomics — all backed by one shared odometry pose, so every kind of motion chains with every other.

> Full documentation (tutorials, API reference, tuning guides): see the `aklib-docs/` Docusaurus site.
> Visual path planning: open `planner/aklib-planner.html` in a browser.

## Features

- **Odometry** with flexible sensor combos (tracking wheels, drive encoders, 1–2 IMUs) and automatic fallbacks
- **Motions:** `driveDistance`, `turnToHeading`, `turnToPoint`, `swingToHeading`, `moveToPoint`, `moveToPose` (boomerang), `follow` (pure pursuit)
- **Motion chaining:** early exit, minimum speed, async + `waitUntil`/cancel
- **Exit conditions** on every motion (settled / close-enough / stalled / timeout), with the reason returned to your auton
- **Automated PID tuning** (`autotuneLateral` / `autotuneAngular`) — the robot tunes itself
- **Tank and holonomic** (X-drive / mecanum) drivetrains behind one interface
- **1-D motion profiles + feedforward** for repeatable fast motions (`.profiled = true`)
- **Brain-screen auton selector** (robodash-style tappable cards, SD-card persistence) — see `examples/selector_example.cpp`
- **Imperial by default, metric literals built in:** `24_in`, `60_cm`, `1.2_m`, `90_deg`, `1.57_rad`, `1.5_tiles`

## Installation

1. Create/open a PROS 4 project.
2. Copy `include/aklib/` into your project's `include/` folder.
3. Copy `src/aklib/` into your project's `src/` folder.
4. `#include "aklib/api.hpp"` and fill out `examples/robot_config.cpp` (copy it into your `src/`).

## 60-second start

```cpp
#include "aklib/api.hpp"
using namespace aklib::literals;

// see examples/robot_config.cpp for the full fill-in-the-blanks sheet
extern aklib::Chassis chassis;

void initialize() { chassis.calibrate(); }

void autonomous() {
    chassis.setPose(36, 12, 90_deg);
    chassis.driveDistance(24);            // EZ-style, odometry-tracked
    chassis.moveToPoint(72, 48);          // LemLib-style, same pose
    chassis.moveToPose(96, 24, 0_deg);    // boomerang: arrive facing +x
}
```

## What needs what

| Feature | Requires |
|---|---|
| `driveDistance`, `turnToHeading`, `swingToHeading` | drive motors (+ IMU strongly recommended) |
| `moveToPoint`, `moveToPose`, `turnToPoint`, `follow` | odometry (IMU + any wheel source; see `odometry.hpp` table) |
| `.profiled = true` motions | feedforward constants in `DriveConfig` |
| `swingToHeading` | tank drive |
| Holonomic strafing motions | `HolonomicDrive` + horizontal tracking wheel |
| Auto-tuner | working odometry + ~6 ft clear space |

## Conventions

- **Units:** inches & degrees everywhere; metric via literals (`_m`, `_cm`, `_mm`, `_rad`).
- **Coordinates:** +x right, +y up (viewed from above); heading 0° = +x, counterclockwise positive. Compass fans: wrap headings in `aklib::compass(deg)`.
- **Every motion returns an `ExitReason`** — check it, it's free debugging.

## Layout

```
include/aklib/    headers (api.hpp includes everything)
src/aklib/        implementations
examples/         robot_config.cpp (the config sheet), autons.cpp
planner/          aklib-planner.html — visual path/auton planner
```

## License / status

Educational project, first release. Algorithms are standard (Pilons odometry, LemLib-style seek motions, boomerang, pure pursuit, twiddle tuning) — see the docs' Resources page for references. Not yet field-hardened: test in a controlled space before competition use.
