#pragma once
/**
 * \file control/pid.hpp
 * The PID controller every AKLib motion is built on.
 *
 * WHAT EACH GAIN DOES (tuning cheat-sheet — full guide in the docs):
 *   kP  main muscle. Too low = sluggish/undershoots. Too high = oscillates.
 *   kI  fixes small persistent error near the target. Usually 0 for drives.
 *       Only ever integrates inside `integralZone` of the target, and the
 *       accumulated term is clamped to `integralMax` (anti-windup).
 *   kD  the brakes. Damps oscillation from kP. Too high = jittery.
 *
 * UNITS: the PID is unit-agnostic — it maps "error units" to "output units".
 * In AKLib motions, error is inches (lateral) or degrees (angular) and output
 * is a normalized command in [-1, 1] that the drivetrain scales to voltage.
 */

#include "aklib/pose.hpp"

namespace aklib {

/** One tunable gain set. Motions take these by value so you can pass
 *  different sets for different situations (short vs long moves, etc.). */
struct PidGains {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    /** Only integrate when |error| < integralZone (0 = integrate always).
     *  Same units as error (inches or degrees). */
    double integralZone = 0;
    /** Cap on |kI * accumulated error| contribution (anti-windup). */
    double integralMax = 1.0;
};

class Pid {
public:
    Pid() = default;
    explicit Pid(const PidGains& gains) : gains_(gains) {}

    /** Call once per loop. `error` = target - measurement. `dt` in seconds. */
    double update(double error, double dt) {
        // -- I term: only accumulate near the target, clamp the result -------
        if (gains_.kI != 0 &&
            (gains_.integralZone <= 0 || std::fabs(error) < gains_.integralZone)) {
            integral_ += error * dt;
            const double cap = gains_.integralMax / gains_.kI;
            integral_ = clamp(integral_, -cap, cap);
        }

        // -- D term on error. First loop has no history: derivative = 0. -----
        double derivative = 0;
        if (hasPrev_ && dt > 0) derivative = (error - prevError_) / dt;
        prevError_ = error;
        hasPrev_ = true;

        return gains_.kP * error + gains_.kI * integral_ + gains_.kD * derivative;
    }

    /** Clear history. Call at the start of every motion (AKLib motions do). */
    void reset() {
        integral_ = 0;
        prevError_ = 0;
        hasPrev_ = false;
    }

    void setGains(const PidGains& g) { gains_ = g; }
    const PidGains& gains() const { return gains_; }

private:
    PidGains gains_{};
    double integral_ = 0;
    double prevError_ = 0;
    bool hasPrev_ = false;
};

} // namespace aklib
