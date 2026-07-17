---
title: Installation
sidebar_position: 1
---

# Installation

## Prerequisites

- [PROS 4](https://pros.cs.purdue.edu/v5/pros-4/index.html) installed (the VS Code extension is the easiest way).
- A V5 brain, and ideally an inertial sensor (IMU) on the robot.

## Option A — start from the template (recommended)

The AKLib repo ships a ready-to-build PROS project with **everything already wired** — nothing to paste:

```
AKLib/example-project/
```

Open it in VS Code (with the PROS extension), hit **Build**, download, and it works immediately:

- `initialize()` calibrates the chassis and puts the **auton selector** on the brain screen, preloaded with the example routines from `autons.cpp` (tap a card to pick; saved to SD).
- `autonomous()` runs whatever the selector picked.
- `opcontrol()` is arcade drive that auto-detects tank vs holonomic from your config.

Your whole workflow from there is three files:
1. `src/robot_config.cpp` — put in *your* ports, dimensions, and gains (the fill-in sheet).
2. `src/autons.cpp` + `include/autons.hpp` — write your real routines.
3. The selector list at the top of `src/main.cpp` — swap the example cards for yours.

## Option B — add AKLib to an existing project

1. Copy `AKLib/include/aklib/` into your project's `include/` folder.
2. Copy `AKLib/src/aklib/` into your project's `src/` folder.
3. Copy `AKLib/examples/robot_config.cpp` and `autons.cpp` into your `src/`, and `autons.hpp` into your `include/`.
4. Either copy `AKLib/examples/main.cpp` over your `main.cpp` (it's the fully wired one from Option A), or graft the pieces you want into your existing main.
5. Build. That's it — AKLib has no dependencies beyond the PROS kernel.

```
your-project/
├── include/
│   ├── aklib/           ← copied
│   ├── autons.hpp       ← copied
│   └── main.h
├── src/
│   ├── aklib/           ← copied
│   ├── robot_config.cpp ← copied (your robot's numbers live here)
│   ├── autons.cpp       ← copied
│   └── main.cpp         ← wired: calibrate + selector + driver control
```

:::caution LLEMU conflicts with the selector
If you keep your own `main.cpp`, remove any `pros::lcd::*` (LLEMU) calls — the legacy LCD draws over the same screen the auton selector uses.
:::

## Verify

Build and download. The robot should sit still ~3 s (IMU calibration), then show the selector with the example autons on the brain screen, and the sticks should drive it. If all three happen, you're installed. Next: [Quickstart](quickstart) to put your robot's numbers in.

:::note OkapiLib?
AKLib is self-contained and does not depend on OkapiLib. They coexist fine if your project already uses Okapi for something else.
:::
