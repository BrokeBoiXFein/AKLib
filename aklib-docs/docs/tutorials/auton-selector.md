---
title: Auton selector
sidebar_position: 9
---

# Auton selector

AKLib ships a brain-screen autonomous selector: EZ-Template's job with a [robodash](https://github.com/unwieldycat/robodash)-style look — a header bar, tappable routine cards with color tags, the current pick highlighted, and the selection **saved to the SD card** so it survives power cycles. Colors follow the Affogato theme.

:::tip Already wired in the template
If you started from `AKLib/example-project`, all of the setup below is already done in `main.cpp` — the selector appears on boot with the example autons preloaded. This page is for understanding it and swapping in your own routines.
:::

## Setup

```cpp
#include "aklib/api.hpp"
#include <memory>

extern aklib::Chassis chassis;
std::unique_ptr<aklib::Selector> selector;

void initialize() {
    chassis.calibrate();
    selector = std::make_unique<aklib::Selector>(
        std::vector<aklib::Selector::Routine>{
            // {card text, function, left-edge tag color (optional)}
            {"Left AWP",   leftAwp,   0xC8485A},
            {"Right rush", rightRush, 0x5894C8},
            {"Skills",     skills,    0xECB93C},
            {"Do nothing", [] {}},
        });
}

void autonomous() {
    if (selector) selector->run();
}
```

That's the whole integration. Build it **in `initialize()`, after `calibrate()`** — the display is guaranteed ready there and the IMU wait doesn't hide the UI.

## How it behaves

- **Tap a card** to select it: the card fills with the accent color and the header shows `✓ <name>`.
- **SD persistence:** with an SD card in the brain, the selection is written to `/usd/aklib_auton.txt` and restored on the next boot. Queue for a match, power on, and the right auton is already selected. No SD card → selection just doesn't persist (everything else works).
- **No selection** (fresh boot, no SD): `run()` does nothing, and the header says "no auton selected" — a safe default.
- **Tag colors** are free-form; using your notebook's EDP process colors (as above) makes routines instantly recognizable to your own team.

## Options

```cpp
aklib::SelectorOptions opts{
    .title = "2502A  ·  Autonomous",
    .accent = 0xB0703C,                    // highlight color
    .background = 0x26170D,
    .savePath = "/usd/aklib_auton.txt",    // nullptr disables persistence
};
selector = std::make_unique<aklib::Selector>(routines, opts);
```

## Tips

- Keep card names short — they ellipsize past ~20 characters.
- More routines than fit on screen? The card grid scrolls vertically.
- `selector->setVisible(false)` hides the whole selector if you want to swap in your own debug screen during driver control.
- Pre-match ritual: glance at the header. If it doesn't show a ✓ and the routine you expect, tap before the queue closes.

## Requirements

Nothing beyond PROS — LVGL ships with the kernel (AKLib's selector targets the LVGL 9 API used by current PROS 4 kernels). SD card optional.
