#pragma once
/**
 * \file control/feedforward.hpp
 * Simple motor feedforward: predicts the voltage a velocity/acceleration
 * needs instead of waiting for error to build up.
 *
 *      voltage = kS * sgn(v)  +  kV * v  +  kA * a
 *
 *   kS  volts to overcome static friction (robot starts creeping)
 *   kV  volts per (in/s) of cruise speed. Rough starting point:
 *          kV ~= 12.0 / theoretical_top_speed_in_per_s
 *   kA  volts per (in/s^2) of acceleration. Tune last, start ~0.
 *
 * Required for: profiled motions (.profiled = true). Not required for plain
 * PID motions. The docs tuning tutorial has the measurement procedure.
 */

#include "aklib/pose.hpp"

namespace aklib {

struct FeedforwardGains {
    double kS = 0;  ///< volts
    double kV = 0;  ///< volts per unit/s
    double kA = 0;  ///< volts per unit/s^2
};

class Feedforward {
public:
    Feedforward() = default;
    explicit Feedforward(const FeedforwardGains& g) : g_(g) {}

    /** Predicted voltage (V) for a target velocity (+optional acceleration). */
    double calculate(double velocity, double accel = 0) const {
        return g_.kS * sgn(velocity) + g_.kV * velocity + g_.kA * accel;
    }

    bool configured() const { return g_.kV != 0; }
    const FeedforwardGains& gains() const { return g_; }

private:
    FeedforwardGains g_{};
};

} // namespace aklib
