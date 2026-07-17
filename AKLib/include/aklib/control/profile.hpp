#pragma once
/**
 * \file control/profile.hpp
 * 1-D trapezoidal motion profile: plan velocity over a move BEFORE moving,
 * instead of letting a PID figure it out reactively.
 *
 * Used by Chassis::driveDistance / turnToHeading when you pass
 * `.profiled = true` (requires drivetrain feedforward constants, see
 * feedforward.hpp). The profile answers: "t seconds into the move, where
 * should I be, how fast should I be going, and how hard accelerating?"
 * Feedforward turns that into voltage; a small PID cleans up the residual.
 *
 * Units are caller's choice (inches or degrees) — just be consistent:
 * maxVel in units/s, maxAccel & maxDecel in units/s^2.
 */

#include <cmath>

namespace aklib {

class TrapezoidalProfile {
public:
    struct State {
        double position = 0;  ///< units traveled since start (always >= 0)
        double velocity = 0;  ///< units/s
        double accel = 0;     ///< units/s^2
    };

    TrapezoidalProfile() = default;

    /**
     * Plan a move of |distance| units (sign handled by the caller).
     * \param maxVel   cruise speed cap, units/s
     * \param maxAccel acceleration limit, units/s^2
     * \param maxDecel deceleration limit, units/s^2 (pass 0 to reuse maxAccel;
     *                 gentler decel = less nose-dive and better accuracy)
     */
    TrapezoidalProfile(double distance, double maxVel, double maxAccel,
                       double maxDecel = 0) {
        dist_ = std::fabs(distance);
        acc_ = std::fabs(maxAccel);
        dec_ = maxDecel > 0 ? std::fabs(maxDecel) : acc_;
        vmax_ = std::fabs(maxVel);

        // Can we even reach vmax? Peak velocity of the triangle profile that
        // exactly covers dist_ with our accel/decel:
        const double vPeak = std::sqrt(2.0 * dist_ * acc_ * dec_ / (acc_ + dec_));
        cruiseVel_ = vPeak < vmax_ ? vPeak : vmax_;

        tAccel_ = cruiseVel_ / acc_;
        tDecel_ = cruiseVel_ / dec_;
        const double dAccel = 0.5 * cruiseVel_ * tAccel_;
        const double dDecel = 0.5 * cruiseVel_ * tDecel_;
        const double dCruise = dist_ - dAccel - dDecel;
        tCruise_ = dCruise > 0 ? dCruise / cruiseVel_ : 0;
    }

    /** Total planned duration in seconds. Handy for auton choreography. */
    double totalTime() const { return tAccel_ + tCruise_ + tDecel_; }

    /** Planned distance. */
    double distance() const { return dist_; }

    /** Sample the plan at time t (seconds since motion start). */
    State at(double t) const {
        State s;
        if (t <= 0) return s;

        if (t < tAccel_) {                       // accelerating
            s.accel = acc_;
            s.velocity = acc_ * t;
            s.position = 0.5 * acc_ * t * t;
        } else if (t < tAccel_ + tCruise_) {     // cruising
            const double tc = t - tAccel_;
            s.velocity = cruiseVel_;
            s.position = 0.5 * cruiseVel_ * tAccel_ + cruiseVel_ * tc;
        } else if (t < totalTime()) {            // decelerating
            const double td = t - tAccel_ - tCruise_;
            s.accel = -dec_;
            s.velocity = cruiseVel_ - dec_ * td;
            s.position = dist_ - 0.5 * dec_ * (tDecel_ - td) * (tDecel_ - td);
        } else {                                 // done
            s.position = dist_;
        }
        return s;
    }

private:
    double dist_ = 0, vmax_ = 0, acc_ = 1, dec_ = 1;
    double cruiseVel_ = 0, tAccel_ = 0, tCruise_ = 0, tDecel_ = 0;
};

} // namespace aklib
