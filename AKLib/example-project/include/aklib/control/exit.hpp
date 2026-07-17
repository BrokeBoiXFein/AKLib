#pragma once
/**
 * \file control/exit.hpp
 * Exit conditions: how a motion decides it is DONE (EZ-Template style).
 *
 * A motion ends when ANY of these fires:
 *   Settled     |error| < smallError continuously for smallTimeMs  (success)
 *   CloseEnough |error| < bigError   continuously for bigTimeMs    (near miss,
 *               stop wasting time oscillating)
 *   Stalled     robot velocity ~ 0 for stallTimeMs while NOT settled
 *               (we hit a wall / a goal / another robot)
 *   EarlyExit   |error| < earlyExitRange (motion chaining: hand off at speed)
 *   Timeout     the per-motion timeoutMs elapsed. EVERY motion has one.
 *
 * Every Chassis motion returns the ExitReason so your auton can react:
 *     if (chassis.driveDistance(30) == ExitReason::Stalled) { ...we hit it... }
 */

#include <cmath>

namespace aklib {

enum class ExitReason {
    Running,      ///< not done yet (internal)
    Settled,      ///< within smallError for smallTimeMs — clean finish
    CloseEnough,  ///< within bigError for bigTimeMs — acceptable finish
    Stalled,      ///< velocity ~0 away from the target — we hit something
    EarlyExit,    ///< crossed earlyExitRange — chained into the next motion
    Timeout,      ///< ran out of time
    Interrupted,  ///< another motion / cancel() took over
};

/** Human-readable name, for logging. */
inline const char* toString(ExitReason r) {
    switch (r) {
        case ExitReason::Running:     return "Running";
        case ExitReason::Settled:     return "Settled";
        case ExitReason::CloseEnough: return "CloseEnough";
        case ExitReason::Stalled:     return "Stalled";
        case ExitReason::EarlyExit:   return "EarlyExit";
        case ExitReason::Timeout:     return "Timeout";
        case ExitReason::Interrupted: return "Interrupted";
    }
    return "?";
}

/**
 * Tolerances for one KIND of motion. AKLib keeps two default sets on the
 * chassis: one lateral (units = inches) and one angular (units = degrees).
 * Any motion call can override any field.
 *
 * Values of 0 disable that particular condition (except timeout — a timeout
 * of 0 means "use the chassis default", never "no timeout").
 */
struct ExitConditions {
    double smallError = 1.0;   ///< in or deg: "on target"
    int smallTimeMs = 150;     ///< must hold smallError this long
    double bigError = 3.0;     ///< in or deg: "close enough"
    int bigTimeMs = 500;       ///< must hold bigError this long
    double stallVelocity = 0.1;///< |velocity| below this counts as stalled
                               ///< (units: in/s or deg/s to match the motion)
    int stallTimeMs = 350;     ///< must be stalled this long (0 = disabled)
    int timeoutMs = 5000;      ///< absolute cap for the motion
};

/**
 * The runtime that evaluates ExitConditions. One Settler is created per
 * motion invocation; feed it every loop.
 */
class Settler {
public:
    Settler(const ExitConditions& c, double earlyExitRange = 0)
        : c_(c), earlyExitRange_(earlyExitRange) {}

    /**
     * \param error     distance/angle remaining (motion units)
     * \param velocity  current speed in the same units per second
     * \param dtMs      milliseconds since last call
     * \return ExitReason::Running until a condition fires.
     */
    ExitReason update(double error, double velocity, int dtMs) {
        elapsedMs_ += dtMs;
        const double aerr = std::fabs(error);

        // Early exit (motion chaining) — checked first so a chained motion
        // hands off at full speed instead of settling.
        if (earlyExitRange_ > 0 && aerr < earlyExitRange_) return ExitReason::EarlyExit;

        // Settled
        if (c_.smallError > 0 && aerr < c_.smallError) smallMs_ += dtMs; else smallMs_ = 0;
        if (c_.smallError > 0 && smallMs_ >= c_.smallTimeMs) return ExitReason::Settled;

        // Close enough
        if (c_.bigError > 0 && aerr < c_.bigError) bigMs_ += dtMs; else bigMs_ = 0;
        if (c_.bigError > 0 && bigMs_ >= c_.bigTimeMs) return ExitReason::CloseEnough;

        // Stalled: not moving, and not because we're on target
        if (c_.stallTimeMs > 0 && std::fabs(velocity) < c_.stallVelocity &&
            aerr > c_.bigError) {
            stallMs_ += dtMs;
        } else {
            stallMs_ = 0;
        }
        // Grace period so "hasn't started moving yet" doesn't read as a stall.
        if (stallMs_ >= c_.stallTimeMs && elapsedMs_ > 500) return ExitReason::Stalled;

        // Timeout
        if (c_.timeoutMs > 0 && elapsedMs_ >= c_.timeoutMs) return ExitReason::Timeout;

        return ExitReason::Running;
    }

private:
    ExitConditions c_;
    double earlyExitRange_;
    int elapsedMs_ = 0, smallMs_ = 0, bigMs_ = 0, stallMs_ = 0;
};

} // namespace aklib
