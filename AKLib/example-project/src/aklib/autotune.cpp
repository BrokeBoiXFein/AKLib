/**
 * \file autotune.cpp
 * Bracket + golden-section kD tuner. The algorithm:
 *
 *  - Each candidate kD gets a FIXED-DURATION test run (2.5 s default) driven
 *    by a raw PID loop straight to the drivetrain — no exit conditions, no
 *    slew — so the full transient is observed and slow tunes can't hide
 *    behind an early "close enough" exit.
 *  - Per run we measure: overshoot (max |error| after crossing to the far
 *    side of the target), oscillations (error sign flips), rise time (first
 *    reach of the settle band — stays large when an overdamped kD
 *    decelerates early and crawls in), settle time (entered the band and
 *    stayed settleHold ms), steady-state error (mean |error| over the tail).
 *  - Phase 1 brackets the oscillation boundary: stable start -> walk kD DOWN
 *    until it oscillates (finds the smallest stable kD, i.e. the fastest);
 *    oscillating start -> climb UP until stable.
 *  - Phase 2 golden-section searches the bracket, minimizing the weighted
 *    cost. Lowest-cost kD wins.
 *
 *  Test motions alternate direction so the robot stays where it started.
 */

#include "aklib/autotune.hpp"

#include <cmath>
#include <cstdio>

#include "pros/rtos.hpp"

namespace aklib {

namespace {

constexpr int LOOP_MS = 10;

/** Everything one axis (angular vs lateral) must provide to be tuned. */
struct Axis {
    const char* name;
    double magnitude;                        // deg or in per test
    std::function<void(double dir)> begin;   // capture a new target from the pose
    std::function<double()> error;           // signed remaining error
    std::function<void(double out)> apply;   // drive with normalized PID output
    std::function<void()> stop;
};

struct TunerState {
    const AutotuneOptions* opt;
    Axis* axis;
    double kP;
    double dir = 1.0;
    double bestKD = 0;
    double bestCost = 1e18;
    bool aborted = false;
    int runs = 0;
};

/** One fixed-duration test of a kD candidate. Ports the reference tuner's
 *  measurement loop onto AKLib's Pid + Drivetrain. */
AutotuneRun runTest(TunerState& st, double kD) {
    const AutotuneOptions& o = *st.opt;
    AutotuneRun r;
    r.riseTimeMs = o.testDurationMs;
    r.settleTimeMs = o.testDurationMs;

    st.axis->begin(st.dir);
    st.dir = -st.dir;   // alternate so the robot never drifts across the field
    st.runs++;

    Pid pid(PidGains{.kP = st.kP, .kI = 0, .kD = kD});

    int initialSign = 0, lastSign = 0;
    bool reached = false, settled = false;
    double settleEnter = -1;
    double ssSum = 0;
    int ssCount = 0;

    uint32_t now = pros::millis();
    const uint32_t t0 = now;

    while (true) {
        const double elapsed = pros::millis() - t0;
        if (elapsed >= o.testDurationMs) break;
        if (o.abortCheck && o.abortCheck()) { st.aborted = true; break; }

        const double err = st.axis->error();
        const double out = clamp(pid.update(err, LOOP_MS / 1000.0), -1.0, 1.0);
        st.axis->apply(out);

        const int s = (err > 0) - (err < 0);
        if (initialSign == 0 && s != 0) { initialSign = s; lastSign = s; }

        // oscillation = error crossing zero (sign flip)
        if (s != 0 && lastSign != 0 && s != lastSign) r.oscillations++;
        if (s != 0) lastSign = s;

        // overshoot = error on the opposite side of where it started
        if (initialSign != 0 && s == -initialSign) {
            r.overshoot = std::max(r.overshoot, std::fabs(err));
        }

        // rise = first moment inside the target band
        if (!reached && std::fabs(err) <= o.settleTol) {
            reached = true;
            r.riseTimeMs = elapsed;
        }

        // settle = stayed inside the band continuously for settleHold
        if (std::fabs(err) <= o.settleTol) {
            if (settleEnter < 0) settleEnter = elapsed;
            if (!settled && elapsed - settleEnter >= o.settleHoldMs) {
                settled = true;
                r.settleTimeMs = settleEnter;
            }
        } else {
            settleEnter = -1;
        }

        // steady-state error = mean |error| over the tail window
        if (elapsed >= o.testDurationMs - o.ssWindowMs) {
            ssSum += std::fabs(err);
            ssCount++;
        }

        pros::Task::delay_until(&now, LOOP_MS);
    }

    st.axis->stop();
    if (ssCount > 0) r.steadyError = ssSum / ssCount;

    r.cost = o.wOvershoot * r.overshoot +
             o.wOscillation * r.oscillations +
             o.wRise * (r.riseTimeMs / 1000.0) +
             o.wSettle * (r.settleTimeMs / 1000.0) +
             o.wSteady * r.steadyError;
    return r;
}

/** Run one candidate, track the best, report, pause. */
AutotuneRun testAt(TunerState& st, double kD, const char* phase) {
    const AutotuneRun r = runTest(st, kD);
    if (r.cost < st.bestCost) { st.bestCost = r.cost; st.bestKD = kD; }
    if (st.opt->verbose) {
        std::printf("[tune %s %-7s] kD=%-8.4f over=%.2f osc=%d rise=%.0fms "
                    "settle=%.0fms ss=%.2f  cost=%.2f  (best kD=%.4f @ %.2f)\n",
                    st.axis->name, phase, kD, r.overshoot, r.oscillations,
                    r.riseTimeMs, r.settleTimeMs, r.steadyError, r.cost,
                    st.bestKD, st.bestCost);
    }
    if (!st.aborted) pros::delay(st.opt->settleBetweenMs);
    return r;
}

bool isStable(const AutotuneOptions& o, const AutotuneRun& r) {
    return r.overshoot <= o.overshootTol && r.oscillations <= o.oscTol;
}

AutotuneResult tune(Axis axis, double kP, const AutotuneOptions& opt) {
    TunerState st{&opt, &axis, kP};

    // Auto-derived bracket bounds (scale-free: proportional to kP).
    double kDStart = opt.kDStart > 0 ? opt.kDStart : kP * 2.0;
    double kDMin = opt.kDMin > 0 ? opt.kDMin : kP * 0.5;
    double kDMax = opt.kDMax > 0 ? opt.kDMax : kP * 200.0;
    if (kDStart <= 0) kDStart = 1.0;
    if (kDMin <= 0) kDMin = 0.1;
    if (kDMax <= 0) kDMax = 200.0;
    st.bestKD = kDStart;

    if (opt.verbose) {
        std::printf("[tune %s] kP=%.4f  bracket [%.4f, %.4f] from kD=%.4f\n",
                    axis.name, kP, kDMin, kDMax, kDStart);
    }

    // ── Phase 1: bracket the optimum. We want kDLow to OSCILLATE and kDHigh
    // to be STABLE, so the refine search straddles the boundary. ────────────
    double kDLow = kDMin;
    double kDHigh = kDMax;

    const AutotuneRun r0 = testAt(st, kDStart, "BRACKET");
    if (isStable(opt, r0)) {
        // Already stable — may be TOO high (decelerating early). Walk DOWN to
        // find the smallest stable kD instead of trusting the starting value.
        kDHigh = kDStart;
        for (double kD = kDStart / opt.growth; kD >= kDMin && !st.aborted;
             kD /= opt.growth) {
            const AutotuneRun r = testAt(st, kD, "DESCEND");
            if (!isStable(opt, r)) { kDLow = kD; break; }
            kDHigh = kD;
        }
    } else {
        // Oscillating — climb UP until the oscillation is gone.
        kDLow = kDStart;
        bool found = false;
        for (double kD = kDStart * opt.growth; kD <= kDMax && !st.aborted;
             kD *= opt.growth) {
            const AutotuneRun r = testAt(st, kD, "ASCEND");
            if (isStable(opt, r)) { kDHigh = kD; found = true; break; }
            kDLow = kD;
        }
        if (!found) kDHigh = kDMax;
    }

    // ── Phase 2: golden-section refine on [kDLow, kDHigh], minimizing cost ──
    double a = kDLow, b = kDHigh;
    constexpr double invphi = 0.6180339887;
    double c = b - invphi * (b - a);
    double d = a + invphi * (b - a);
    double fc = st.aborted ? 0 : testAt(st, c, "REFINE").cost;
    double fd = st.aborted ? 0 : testAt(st, d, "REFINE").cost;

    for (int i = 0; i < opt.refineIters && !st.aborted; i++) {
        if (fc < fd) {
            b = d; d = c; fd = fc;
            c = b - invphi * (b - a);
            fc = testAt(st, c, "REFINE").cost;
        } else {
            a = c; c = d; fc = fd;
            d = a + invphi * (b - a);
            fd = testAt(st, d, "REFINE").cost;
        }
    }

    axis.stop();

    AutotuneResult result;
    result.gains = PidGains{.kP = kP, .kI = 0, .kD = st.bestKD};
    result.cost = st.bestCost;
    result.aborted = st.aborted;
    result.runsExecuted = st.runs;

    if (opt.verbose) {
        std::printf("[tune %s] %s — copy into ChassisTunings:\n"
                    "    .kP = %.4f, .kI = 0, .kD = %.4f    (cost %.2f, %d runs)\n",
                    axis.name, st.aborted ? "ABORTED, best so far" : "DONE",
                    kP, st.bestKD, st.bestCost, st.runs);
    }
    return result;
}

} // namespace

// ---------------------------------------------------------------------------
// Angular: turn in place to +/- testMagnitude, error in degrees.
// ---------------------------------------------------------------------------
AutotuneResult autotuneAngular(Chassis& chassis, double kP,
                               const AutotuneOptions& options) {
    const double mag = options.testMagnitude > 0 ? options.testMagnitude : 90.0;
    auto drive = chassis.drivetrain();
    double target = 0;

    Axis axis{
        "angular", mag,
        [&](double dir) { target = chassis.pose().thetaDeg() + dir * mag; },
        [&] { return wrapDeg(target - chassis.pose().thetaDeg()); },
        [&](double out) { drive->applyNormalized({0, 0, out}); },
        [&] { drive->stop(false); },
    };
    return tune(std::move(axis), kP, options);
}

// ---------------------------------------------------------------------------
// Lateral: shuttle +/- testMagnitude along the starting heading, error in
// inches (signed projection onto the axis). The chassis heading PID keeps the
// runs straight so the shuttle stays on one line.
// ---------------------------------------------------------------------------
AutotuneResult autotuneLateral(Chassis& chassis, double kP,
                               const AutotuneOptions& options) {
    const double mag = options.testMagnitude > 0 ? options.testMagnitude : 24.0;
    auto drive = chassis.drivetrain();
    Point axisDir{1, 0};
    Point target{0, 0};
    double holdHeadingDeg = 0;
    Pid headPid(chassis.tunings().heading);

    Axis axis{
        "lateral", mag,
        [&](double dir) {
            const Pose p = chassis.pose();
            axisDir = {std::cos(p.theta), std::sin(p.theta)};
            target = p.point() + axisDir * (dir * mag);
            holdHeadingDeg = p.thetaDeg();
            headPid.reset();
        },
        [&] { return (target - chassis.pose().point()).dot(axisDir); },
        [&](double out) {
            const double errHead =
                wrapDeg(holdHeadingDeg - chassis.pose().thetaDeg());
            drive->applyNormalized(
                {out, 0, headPid.update(errHead, LOOP_MS / 1000.0)});
        },
        [&] { drive->stop(false); },
    };
    return tune(std::move(axis), kP, options);
}

} // namespace aklib
