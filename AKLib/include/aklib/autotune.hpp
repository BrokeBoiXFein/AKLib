#pragma once
/**
 * \file autotune.hpp
 * Automated PID tuning. The robot repeatedly drives a test motion, scores how
 * well each candidate gain set performed (overshoot, settle time, error over
 * time), nudges the gains, and converges on tuned values — the "twiddle"
 * (coordinate-descent) algorithm with safety rails.
 *
 * ============================================================================
 *  HOW TO USE (full walkthrough in docs > Tutorials > Auto-tuning)
 * ============================================================================
 *  1. Put the robot on a competition-like surface with clear space:
 *       - lateral tuning: >= 2x testDistance of straight run-off (default 24"
 *         drive => give it ~6 ft)
 *       - angular tuning: just needs room to spin in place
 *  2. Fresh-ish battery. Gains tuned at 40% battery behave differently at 100%.
 *  3. Call from opcontrol (e.g. bind to a controller button):
 *
 *       auto result = aklib::autotuneAngular(chassis);
 *       if (result.success) chassis.tunings().angular = result.gains;
 *       auto latResult = aklib::autotuneLateral(chassis);
 *       if (latResult.success) chassis.tunings().lateral = latResult.gains;
 *
 *  4. Watch the PROS terminal (`pros terminal`): every run prints gains and
 *     score; the final line prints the numbers to copy into your
 *     ChassisTunings in robot_config.cpp.
 *
 *  The robot returns toward its start pose between runs automatically.
 *  Expect a session to take 3-8 minutes depending on `iterations`.
 */

#include "aklib/chassis.hpp"
#include "aklib/control/pid.hpp"

namespace aklib {

struct AutotuneOptions {
    /** Twiddle sweeps. More = finer convergence, longer session. Each
     *  iteration runs the test motion ~2-4 times per tuned gain. */
    int iterations = 8;
    /** Lateral test drive length, inches. */
    double testDistance = 24.0;
    /** Angular test turn size, degrees. */
    double testAngleDeg = 90.0;
    /** Per-run cap, ms — runs that haven't settled by now score badly. */
    int runTimeoutMs = 2500;
    /** Also tune kI (rarely needed for drivetrains; default off). */
    bool tuneKi = false;
    /** Print per-run telemetry to the terminal. */
    bool verbose = true;

    // Scoring weights: score = ITAE + overshootW * overshoot^2
    //                        + settleW * settle_seconds. Lower = better.
    double overshootWeight = 40.0;
    double settleWeight = 3.0;
};

struct AutotuneResult {
    PidGains gains{};   ///< best gain set found
    double score = 0;   ///< its score (for comparing sessions)
    bool success = false;
    int runsExecuted = 0;
};

/** Tune the LATERAL (drive straight) PID. Robot drives testDistance forward
 *  and back repeatedly. Returns the best gains found (does NOT write them to
 *  the chassis — assign result.gains yourself so you stay in control). */
AutotuneResult autotuneLateral(Chassis& chassis, const AutotuneOptions& = {});

/** Tune the ANGULAR (turn) PID. Robot turns +/- testAngleDeg in place. */
AutotuneResult autotuneAngular(Chassis& chassis, const AutotuneOptions& = {});

} // namespace aklib
