---
title: Installation
sidebar_position: 1
---

# Installation

## Prerequisites

- [PROS 4](https://pros.cs.purdue.edu/v5/pros-4/index.html) installed (the VS Code extension is the easiest way).
- A V5 brain, and ideally an inertial sensor (IMU) on the robot.

## Option A — start from the example project

The AKLib repo ships a ready-to-build PROS project with the library already wired in:

```
AKLib/example-project/
```

Open it in VS Code (with the PROS extension), hit **Build**, and you have a compiling baseline. Then edit `src/robot_config.cpp` for your robot.

## Option B — add AKLib to an existing project

1. Copy `AKLib/include/aklib/` into your project's `include/` folder.
2. Copy `AKLib/src/aklib/` into your project's `src/` folder.
3. Copy `AKLib/examples/robot_config.cpp` into your `src/` folder.
4. Build. That's it — AKLib has no dependencies beyond the PROS kernel.

```
your-project/
├── include/
│   ├── aklib/          ← copied
│   └── main.h
├── src/
│   ├── aklib/          ← copied
│   ├── robot_config.cpp ← copied (your robot's numbers live here)
│   └── main.cpp
```

## Verify

Add to `main.cpp`:

```cpp
#include "aklib/api.hpp"

extern aklib::Chassis chassis;   // defined in robot_config.cpp

void initialize() {
    chassis.calibrate();         // robot must be still (~3 s)
}
```

Build and download. If it compiles and the IMU calibrates (robot sits still for ~3 seconds at program start), you're installed. Next: [Quickstart](quickstart).

:::note OkapiLib?
AKLib is self-contained and does not depend on OkapiLib. They coexist fine if your project already uses Okapi for something else.
:::
