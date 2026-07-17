#pragma once
/**
 * \file control/slew.hpp
 * Slew rate limiter: caps how fast a command may INCREASE, preventing wheel
 * slip / wheelies at the start of motions (which also protects odometry
 * accuracy, especially motor-encoder odometry).
 *
 * By default only increases in |output| are limited — the robot may always
 * slow down as fast as the controller asks (you never want slew to prevent
 * braking).
 */

#include "aklib/pose.hpp"

namespace aklib {

class SlewLimiter {
public:
    /**
     * \param ratePerSec  max allowed increase in output per second, in output
     *                    units. AKLib motions use normalized output [-1,1], so
     *                    a rate of 4.0 means 0 -> full power in 0.25 s.
     *                    A rate <= 0 disables slew (pass-through).
     * \param limitDecrease  also limit how fast output may fall (rarely wanted)
     */
    explicit SlewLimiter(double ratePerSec = 0, bool limitDecrease = false)
        : rate_(ratePerSec), limitDecrease_(limitDecrease) {}

    /** Call once per loop with the raw desired output; returns limited output. */
    double calculate(double desired, double dt) {
        if (rate_ <= 0) { last_ = desired; return desired; }
        const double maxStep = rate_ * dt;
        double out = desired;

        const bool increasing = std::fabs(desired) > std::fabs(last_);
        if (increasing || limitDecrease_) {
            out = last_ + clamp(desired - last_, -maxStep, maxStep);
        }
        last_ = out;
        return out;
    }

    /** Start the ramp from a known output (0 at motion start, or the current
     *  output when chaining so momentum carries through). */
    void reset(double startingOutput = 0) { last_ = startingOutput; }

private:
    double rate_;
    bool limitDecrease_;
    double last_ = 0;
};

} // namespace aklib
