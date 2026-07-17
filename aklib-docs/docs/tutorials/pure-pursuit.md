---
title: Pure pursuit
sidebar_position: 6
---

# Pure pursuit

Pure pursuit follows a pre-planned **path** by continuously chasing a *lookahead point* that slides along the path ahead of the robot. Because progress is measured by **position, not time**, getting bumped doesn't break anything — the robot just keeps following from wherever it ends up.

## Requirements

- Working odometry ([any combo](../configuration/sensors)).
- A `Path` — draw one in the [AKLib Planner](path-planner) (easiest) or build in code.

## Building a path in code

Paths are chained cubic Béziers: 4 control points for the first segment, +3 per additional segment (each new segment starts where the last ended).

```cpp
aklib::Path rush = aklib::Path::bezier({
    {12, 36}, {30, 36}, {40, 52}, {56, 52},   // P0  C0  C1  P1
    {66, 52}, {72, 40}, {72, 24},             //     C0  C1  P1  (segment 2)
}, {
    .maxSpeed = 0.9,   // speed cap along the path (fraction)
    .turnK    = 1.2,   // corner slowdown: speed = min(max, turnK/curvature)
});
```

Declare paths `static` (or at file scope) so they're generated once, not every auton run. Generation pre-computes point spacing, curvature-aware speeds, and a deceleration pass — the follower just reads the plan.

## Following

```cpp
chassis.follow(rush, {.lookahead = 12});
chassis.follow(rush, {.reverse = true, .lookahead = 12});  // back down the path
```

When the remaining path is shorter than the lookahead, AKLib hands off to `moveToPoint` on the final point, so path ends settle exactly like any point motion (same exit conditions, same chaining options).

## Tuning lookahead — the one knob

| Lookahead | Behavior |
|---|---|
| Small (6–8″) | hugs the path tightly, but oscillates at speed |
| Medium (10–14″) | the right answer for most VEX robots |
| Large (16″+) | silky smooth, cuts corners hard |

Start at 12″. If the robot weaves along straights → increase. If it rounds off corners you need → decrease, or slow the path (`turnK` down / `maxSpeed` down).

## Path options reference

| Option | Default | Meaning |
|---|---|---|
| `spacing` | 0.5 | inches between samples |
| `maxSpeed` | 1.0 | speed cap (fraction of full output) |
| `turnK` | 1.0 | corner slowdown aggressiveness (lower = slower corners) |
| `decel` | 0.06 | braking limit for the backward speed pass |
| `endSpeed` | 0 | speed at the path's end (>0 when chaining onward) |

## Behavior notes

- **Corner cutting is inherent** to pure pursuit (the lookahead chord shortcuts curvature). Plan paths with clearance, or drop the lookahead for tight sections.
- **Holonomic drives** follow the path as a velocity vector and keep their heading on the path tangent — or pin it with `.face = 90` to strafe along the path facing a goal.
- Stall/timeout exits work mid-path (`ExitReason::Stalled` if you get body-blocked).
