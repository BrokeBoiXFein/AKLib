---
title: MotionParams
sidebar_position: 2
---

# `aklib::MotionParams`

The per-call knobs accepted by every motion. Use designated initializers and set only what you need:

```cpp
chassis.moveToPoint(72, 48, {.maxSpeed = 0.8, .reverse = true});
```

:::caution
C++ requires designated initializers **in declaration order** — the order below.
:::

| Field | Type | Default | Meaning |
|---|---|---|---|
| `maxSpeed` | double | 1.0 | Output cap, fraction of full power (0–1] |
| `minSpeed` | double | 0 | Output floor — keeps momentum through [chained](../tutorials/motions#chaining) motions |
| `reverse` | bool | false | Drive backward toward the target / follow path backward |
| `earlyExitRange` | double | 0 | Exit at speed within this error (in or deg). 0 = off |
| `timeoutMs` | int | 0 | Motion time cap; 0 = use exit-conditions default |
| `slewRate` | double | −1 | −1 = chassis default, 0 = disable, else output/second |
| `async` | bool | false | Return immediately; control with `waitUntil`/`waitUntilDone`/`cancelMotion` |
| `brakeAtEnd` | bool | true | Brake (true) or coast (false) when the motion ends |
| `dlead` | double | 0.6 | **moveToPose only:** boomerang curve width (0 = straight, 1 = very wide) |
| `lookahead` | double | 10 | **follow only:** pure pursuit lookahead radius, inches |
| `profiled` | bool | false | **driveDistance/turnToHeading:** trapezoid + feedforward instead of pure PID (needs [FF constants](../tutorials/motion-profiling)) |
| `face` | optional double | — | **holonomic only:** heading (deg) to hold while translating |
| `lateralGains` | optional PidGains | — | Override the lateral PID for this call |
| `angularGains` | optional PidGains | — | Override the angular PID for this call |
| `exits` | optional ExitConditions | — | Override the [exit conditions](../configuration/exit-conditions) for this call |

## Recipes

```cpp
// chained mid-route waypoint
{.minSpeed = 0.35, .earlyExitRange = 6}

// careful approach with a deadline
{.maxSpeed = 0.5, .timeoutMs = 1500}

// backward boomerang with a tight curve
{.reverse = true, .dlead = 0.4}

// background motion you'll react to
{.async = true}

// one-off gain experiment without touching config
{.lateralGains = aklib::PidGains{.kP = 0.1, .kD = 0.4}}
```
