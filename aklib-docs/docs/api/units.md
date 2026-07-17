---
title: Units & pose types
sidebar_position: 8
---

# Units, `Point`, `Pose`

Headers: `aklib/units.hpp`, `aklib/pose.hpp`. Conventions: [Units & coordinates](../getting-started/conventions).

## Literals

```cpp
using namespace aklib::literals;
24_in  2_ft  1.2_m  60_cm  15_mm  1.5_tiles     // → inches
90_deg  1.57_rad                                 // → degrees
```

All `constexpr` — compile-time conversion, zero runtime cost.

## Conversion functions

```cpp
namespace aklib::units {
    fromFeet(v)  fromMeters(v)  fromCm(v)  fromMm(v)  fromRadians(v)  // → in/deg
    toFeet(in)   toMeters(in)   toCm(in)   toMm(in)   toRadians(deg)  // ← in/deg
}
aklib::compass(deg);   // compass heading (0=up, CW+) → math heading
```

## Point

```cpp
struct Point { double x, y; };
p.distTo(q);        // inches
p.angleTo(q);       // radians, math-standard
p.rotatedBy(rad);   // rotate about origin, CCW+
p + q; p - q; p * s; p.dot(q); p.norm();
```

## Pose

```cpp
struct Pose { double x, y; double theta; };   // theta in RADIANS internally
pose.thetaDeg();                    // degrees out
Pose::fromDeg(x, y, thetaDeg);      // degrees in
pose.distTo(pt);  pose.angleTo(pt);
pose.toLocal(fieldPt);              // field point → robot frame (+x fwd, +y left)
```

:::note Why radians inside, degrees outside?
Math (`atan2`, rotations) is radian-native; humans and the AKLib API are degree-native. `Pose` stores radians so algorithm code never converts; the API boundary (`setPose`, `turnToHeading`, `thetaDeg()`) speaks degrees so you never see radians unless you ask.
:::

## Free helpers

```cpp
aklib::wrapAngle(rad);   // → (-π, π]
aklib::wrapDeg(deg);     // → (-180, 180]
aklib::degToRad(d);  aklib::radToDeg(r);
aklib::clamp(v, lo, hi);  aklib::sgn(v);  aklib::sinc(x);
```
