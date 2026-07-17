---
title: Drivetrain
sidebar_position: 4
---

# `aklib::Drivetrain` / `TankDrive` / `HolonomicDrive`

Header: `aklib/drivetrain.hpp`. Concepts: [Configuration → Drivetrains](../configuration/drivetrains).

## The interface

```cpp
class Drivetrain {
    virtual void applyNormalized(const ChassisSpeeds& speeds) = 0;
    virtual void stop(bool hold = false) = 0;
    virtual double averageWheelSpeed() = 0;   // in/s, for stall detection
    virtual bool holonomic() const = 0;
    const DriveConfig& config() const;
};
```

Motions only ever see this interface — that's what makes tank and holonomic interchangeable. `ChassisSpeeds{vx, vy, omega}` components are normalized [-1, 1]; if a mix would exceed 100% on any wheel, all wheels scale down proportionally (turning authority is preserved).

## DriveConfig

```cpp
struct DriveConfig {
    double wheelDiameter = 3.25;      // in
    double trackWidth = 12.0;         // in
    double wheelRpm = 450;            // after external gearing
    FeedforwardGains feedforward{};   // optional, for .profiled motions
    double maxSpeed() const;          // derived top speed, in/s
};
```

## TankDrive

```cpp
TankDrive(std::shared_ptr<pros::MotorGroup> left,
          std::shared_ptr<pros::MotorGroup> right,
          const DriveConfig& cfg);

void tank(double leftOut, double rightOut);   // driver control, [-1, 1]
void arcade(double throttle, double turn);
std::shared_ptr<pros::MotorGroup> leftGroup();
std::shared_ptr<pros::MotorGroup> rightGroup();
```

## HolonomicDrive

```cpp
HolonomicDrive(fl, fr, bl, br,                 // one MotorGroup per corner
               HolonomicDrive::Kind kind,      // XDrive | Mecanum
               const DriveConfig& cfg);

// driver control; set fieldCentric=true and pass the heading for
// field-centric strafing
void drive(double strafe, double forward, double turn,
           double headingRad = 0, bool fieldCentric = false);
```

Corner convention: each group spins its wheel robot-forward when commanded positive — negate ports until true.

## ChassisSpeeds

```cpp
struct ChassisSpeeds {
    double vx;     // + = robot forward
    double vy;     // + = robot LEFT (tank drives ignore this)
    double omega;  // + = counterclockwise
};
```
