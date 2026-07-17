---
title: Resources & further reading
sidebar_position: 99
---

# Resources & further reading

AKLib's algorithms are standard, well-documented robotics techniques. If you want to understand (or extend) what's under the hood:

## The algorithms in AKLib

- **Odometry** — Pilons 5225A, *Introduction to Position Tracking*: the arc-based update AKLib implements.
- **Boomerang controller** — VEX community technique; see the VRC forum "boomerang controller" threads and Desmos demos for `dlead` intuition.
- **Pure pursuit** — R. Craig Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm* (CMU-RI-TR-92-01).
- **Feedforward & profiling** — [WPILib docs](https://docs.wpilib.org/en/stable/index.html): feedforward characterization, trapezoid profiles.
- **Twiddle (auto-tuning)** — coordinate descent for controller gains; popularized by Sebastian Thrun's AI for Robotics course.

## Libraries worth studying

- [LemLib](https://github.com/LemLib/LemLib) — the reference for pose-based VEX motion.
- [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) — the reference for tuning ergonomics and exit conditions.
- [OkapiLib](https://github.com/OkapiLib/OkapiLib) — typed units and motion profiling in C++.
- [vexide](https://github.com/vexide/vexide) + [evian](https://github.com/vexide/evian) — the Rust side of this ecosystem.

## Deeper theory

- *Controls Engineering in FRC* (Tyler Veness) — free book; PID through state-space with robotics framing.
- [Purdue SIGBots (BLRS) Wiki](https://wiki.purduesigbots.com/) — the best general VEX knowledge base.
- *Probabilistic Robotics* (Thrun, Burgard, Fox) — for when you're ready to add MCL to your localization stack.

## Roadmap ideas (not yet in AKLib)

Planned/possible extensions, roughly in order of payoff:

1. Distance-sensor wall resets → pose-estimator interface
2. 2-D trajectories + RAMSETE/LTV follower (the profiling stack is already here)
3. Monte Carlo Localization with distance sensors
4. Brain-screen field dashboard + auton selector
5. A Rust/vexide port sharing this design
