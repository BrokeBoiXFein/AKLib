---
title: Selector
sidebar_position: 9
---

# `aklib::Selector`

Brain-screen autonomous selector. Header: `aklib/selector.hpp`. Usage guide: [Auton selector tutorial](../tutorials/auton-selector).

## Types

```cpp
struct Selector::Routine {
    std::string name;             // card text
    std::function<void()> run;    // your auton function
    std::uint32_t tag = 0;        // optional left-edge tag color (0 = none)
};

struct SelectorOptions {
    const char* title = "AKLib  ·  Autonomous";
    std::uint32_t accent = 0xB0703C;      // highlight (caramel)
    std::uint32_t background = 0x26170D;  // screen background (espresso)
    const char* savePath = "/usd/aklib_auton.txt";  // nullptr = no persistence
};
```

## Interface

```cpp
Selector(std::vector<Routine> routines, SelectorOptions opts = {});
```

Builds the LVGL UI immediately — construct in `initialize()`. Restores any saved selection from the SD card. Non-copyable; the destructor removes the UI.

| Method | Does |
|---|---|
| `run()` | Execute the selected routine (call in `autonomous()`); no-op when nothing is selected |
| `selectedIndex()` | Current index, or −1 |
| `selectedName()` | Current name, or `""` |
| `select(int)` | Programmatic selection (same effect as a tap, including SD save) |
| `setVisible(bool)` | Show/hide the whole selector |

## Behavior notes

- Selection is persisted **by routine name**, so reordering routines between builds keeps the right one selected; renaming clears it.
- SD I/O is best-effort: no card, no error — persistence is just skipped.
- Implemented against the LVGL 9 API bundled with current PROS 4 kernels.
