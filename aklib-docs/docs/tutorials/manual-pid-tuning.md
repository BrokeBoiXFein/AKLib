---
title: Manual PID tuning
sidebar_position: 3
---

# Manual PID tuning

Prefer the [auto-tuner](auto-tuning)? Run it first — then use this page to understand and refine what it found. Manual tuning is still worth learning: it builds the intuition you need when something misbehaves at a competition.

## Setup

1. Robot on a competition-like surface (foam tiles), battery reasonably charged.
2. A repeatable test you can trigger from the controller:

```cpp
void opcontrol() {
    pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
    while (true) {
        if (ctrl.get_digital_new_press(DIGITAL_A)) {
            chassis.driveDistance(24, {.timeoutMs = 3000});
            chassis.driveDistance(-24, {.timeoutMs = 3000});
        }
        if (ctrl.get_digital_new_press(DIGITAL_B)) {
            chassis.turnToHeading(chassis.pose().thetaDeg() + 90,
                                  {.timeoutMs = 2500});
        }
        pros::delay(20);
    }
}
```

3. Watch numbers, not vibes — print error over time (`pros terminal`):

```cpp
// inside a test, run async and sample:
chassis.driveDistance(24, {.async = true});
while (chassis.isMoving()) {
    printf("%.2f\n", 24.0 - /* distance traveled */ 0);  // or log pose
    pros::delay(20);
}
```

## The recipe (same for lateral and angular)

Tune in this order, one gain at a time. Start from AKLib's defaults.

### 1. kP — raise until slight oscillation

Double kP until the robot **overshoots and oscillates once or twice**, then back off ~20%. Symptoms:
- Crawls to the target, never quite arrives → kP too low.
- Slams past and wobbles repeatedly → kP too high.

### 2. kD — damp the oscillation

Raise kD until the overshoot from step 1 disappears and the approach is crisp. Symptoms:
- Still overshoots → more kD.
- Motion gets jittery/grindy, especially near the end → too much kD (kD amplifies sensor noise).

Typical useful ratio lands around kD ≈ 3–8× kP for drives (inches) and 5–10× kP for turns (degrees) — but treat that as a sanity check, not a rule.

### 3. kI — only if a persistent gap remains

If the robot consistently stops a fixed half-inch/degree short (friction), add a tiny kI. AKLib's PID only integrates inside `integralZone` and clamps the term (`integralMax`), so windup is controlled — but most drivetrains are better at kI = 0 with slightly more kP.

### 4. Cross-check both distances

A tune that's perfect at 24″ can be lazy at 6″ and violent at 48″. Check short and long motions; if they disagree badly, bias toward the distance your autons actually use, or pass per-call gain overrides:

```cpp
chassis.moveToPoint(30, 30, {.lateralGains = aklib::PidGains{.kP = 0.10, .kD = 0.35}});
```

## The heading PID

`tunings.heading` keeps `driveDistance` straight. Tune it *after* lateral: command a long straight drive and nudge the robot mid-motion — it should recover its line without fishtailing. Too high = snake-walk; too low = drifts off line.

## Slew interaction

Tune with your real `slewRate` enabled (it changes the effective response). If motions start with wheelspin, lower `slewRate`; wheel slip poisons motor-encoder odometry.

## Symptoms table

| Symptom | Likely fix |
|---|---|
| Slow crawl at end of motion | +kP, or add small kI, or raise `minSpeed` |
| Overshoot then return | +kD (or −kP) |
| Rapid shudder near target | −kD |
| Consistently short by same amount | small kI or +kP |
| Fishtails while driving straight | −heading kP or +heading kD |
| Wheelies / wheelspin at launch | lower `slewRate` |
