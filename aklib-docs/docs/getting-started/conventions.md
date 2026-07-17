---
title: Units & coordinates
sidebar_position: 3
---

# Units & coordinates

## The one rule

**Every plain number AKLib takes or returns is imperial: inches and degrees.** Timeouts are milliseconds.

## Metric, whenever you want it

You never convert by hand — the literals do it at compile time (zero runtime cost):

```cpp
using namespace aklib::literals;

chassis.driveDistance(60_cm);        // = driveDistance(23.62)
chassis.moveToPoint(1.2_m, 0.9_m);   // meters → inches
chassis.turnToHeading(1.57_rad);     // radians → degrees
chassis.driveDistance(1.5_tiles);    // 1 tile = 24 in
```

| Literal | Converts | Example |
|---|---|---|
| `_in` | inches (identity) | `24_in` |
| `_ft` | feet → in | `2_ft` = 24 |
| `_m`, `_cm`, `_mm` | metric → in | `60_cm` ≈ 23.6 |
| `_deg` | degrees (identity) | `90_deg` |
| `_rad` | radians → deg | `1.57_rad` ≈ 90 |
| `_tiles` | field tiles → in | `1.5_tiles` = 36 |

Functions `aklib::units::toMeters(in)`, `toCm(in)`, `toRadians(deg)` etc. convert the other way for telemetry/printing.

## Coordinates & headings

- **+x is right, +y is up** when looking down at the field. `(0, 0)` is the bottom-left field corner (by convention — odometry only cares that you're consistent).
- **Heading 0° points along +x**, and **counterclockwise is positive** (standard math convention, so `atan2` and every formula in the library work without sign gymnastics).

```
        90°
         ↑ +y
         │
180° ────┼────→ 0°   +x
         │
        -90° (= 270°)
```

### "But I think in compass headings"

If 0° = up / clockwise-positive (the LemLib convention) is how your team thinks, wrap the number:

```cpp
chassis.turnToHeading(aklib::compass(90));   // compass 90 = "east" = math 0
```

## Angle wrapping

AKLib wraps every heading error internally to (-180°, 180°], so `turnToHeading(179)` from `-179` turns **2 degrees**, not 358. You never need to pre-wrap targets.

## Field reference numbers

| Thing | Value |
|---|---|
| Field | 144 × 144 in (12 × 12 ft) |
| One foam tile | 24 in |
| Common wheels | 2.75″, 3.25″, 4″ (measure yours!) |
