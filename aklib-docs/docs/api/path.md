---
title: Path
sidebar_position: 5
---

# `aklib::Path`

Header: `aklib/path.hpp`. Concepts: [Pure pursuit tutorial](../tutorials/pure-pursuit). Most paths come from the [Planner](../tutorials/path-planner)'s code export.

## Building

```cpp
// Chained cubic Béziers: 4 points, then +3 per extra segment
static Path route = Path::bezier({
    {12, 36}, {30, 36}, {40, 52}, {56, 52},
    {66, 52}, {72, 40}, {72, 24},
}, {.maxSpeed = 0.9, .turnK = 1.2});

// Straight line (programmatic paths)
static Path lane = Path::line({12, 12}, {12, 96});
```

### PathOptions

| Field | Default | Meaning |
|---|---|---|
| `spacing` | 0.5 | inches between sampled points |
| `maxSpeed` | 1.0 | speed cap, fraction of full output |
| `turnK` | 1.0 | corner slowdown: `speed = min(maxSpeed, turnK / |curvature|)` |
| `decel` | 0.06 | backward-pass braking limit (fraction per inch) |
| `endSpeed` | 0 | planned speed at the path end (>0 to chain onward) |

Generation samples the Béziers densely, resamples to even arc-length spacing, computes curvature per point, assigns curvature-limited speeds, and runs a backward pass so the plan never demands more braking than `decel`. All at construction — declare paths `static` so this runs once.

## Inspection

```cpp
struct Path::PathPoint { Point p; double speed; double distance; };

path.points();          // the sampled plan
path.length();          // total arc length, inches
path.back();            // final point
path.empty();
```

## Follower support (used by `Chassis::follow`)

```cpp
std::size_t closestIndex(const Point& robot, std::size_t fromIndex = 0) const;
Point lookaheadPoint(const Point& robot, double lookahead,
                     std::size_t& progressIndex) const;
```

`lookaheadPoint` walks monotonically forward (never re-acquires earlier path on self-crossing routes), interpolates the exact circle crossing, falls back to the closest point when the robot is far off the path, and returns the final point once the remaining path is within the lookahead.
