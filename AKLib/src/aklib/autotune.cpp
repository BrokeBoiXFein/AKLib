/**
 * \file autotune.cpp
 * Twiddle (coordinate descent) auto-tuner.
 *
 * One "objective run" = execute the test motion with candidate gains while
 * sampling error every 10 ms, then compute a score:
 *
 *     score = ITAE + overshootWeight * overshoot^2 + settleWeight * seconds
 *
 *     ITAE  = integral of time * |error|  — punishes error that LINGERS,
 *             which is exactly what a slow or oscillating tune does.
 *     overshoot — how far past the target the robot blew through.
 *
 * Twiddle then walks each gain up/down, keeping changes that lower the score
 * and shrinking the step size when neither direction helps. It converges to
 * a local optimum — which, seeded from AKLib's conservative defaults, is
 * reliably a competition-quality tune.
 */

#include "aklib/autotune.hpp"

#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

#include "pros/rtos.hpp"

namespace aklib {

namespace {

struct RunScore {
    double score = 1e18;
    double overshoot = 0;
    double settleSec = 0;
    bool aborted = false;
};

/** Score one test motion. `errorFn` returns the signed remaining error in
 *  motion units; `startMotion` kicks off the async motion. */
RunScore scoreRun(Chassis& chassis, const AutotuneOptions& opt,
                  double targetMagnitude,
                  const std::function<void()>& startMotion,
                  const std::function<double()>& errorFn) {
    RunScore r;
    startMotion();

    double itae = 0;
    const uint32_t t0 = pros::millis();

    while (chassis.isMoving()) {
        const double t = (pros::millis() - t0) / 1000.0;
        const double err = errorFn();

        itae += t * std::fabs(err) * 0.01;  // dt = 10 ms
        // err < 0 means we're past the target: overshoot.
        r.overshoot = std::max(r.overshoot, -err);

        // Safety rail: error growing far beyond the start = divergent gains.
        if (std::fabs(err) > 1.8 * targetMagnitude) {
            chassis.cancelMotion();
            r.aborted = true;
        }
        pros::delay(10);
    }

    r.settleSec = (pros::millis() - t0) / 1000.0;
    r.score = r.aborted
                  ? 1e12
                  : itae + opt.overshootWeight * r.overshoot * r.overshoot +
                        opt.settleWeight * r.settleSec;
    return r;
}

/** Generic twiddle over the gains used by `evaluate`. */
AutotuneResult twiddle(PidGains seed, const AutotuneOptions& opt,
                       const std::function<double(const PidGains&)>& evaluate,
                       const char* label) {
    AutotuneResult result;

    // Parameter vector: [kP, kD, (kI)]. Steps start at 40% of the seed (or a
    // small floor for gains seeded at zero).
    std::vector<double> g = {seed.kP, seed.kD};
    std::vector<double> dg = {std::max(seed.kP * 0.4, 0.005),
                              std::max(seed.kD * 0.4, 0.01)};
    if (opt.tuneKi) {
        g.push_back(seed.kI);
        dg.push_back(std::max(seed.kI * 0.4, 0.001));
    }

    auto toGains = [&](const std::vector<double>& v) {
        PidGains out = seed;
        out.kP = std::max(v[0], 0.0);
        out.kD = std::max(v[1], 0.0);
        if (opt.tuneKi) out.kI = std::max(v[2], 0.0);
        return out;
    };

    double best = evaluate(toGains(g));
    result.runsExecuted++;
    if (opt.verbose) {
        std::printf("[autotune %s] baseline  kP=%.4f kD=%.4f  score=%.1f\n",
                    label, g[0], g[1], best);
    }

    for (int it = 0; it < opt.iterations; it++) {
        for (std::size_t i = 0; i < g.size(); i++) {
            // Try stepping this gain UP.
            g[i] += dg[i];
            double s = evaluate(toGains(g));
            result.runsExecuted++;
            if (s < best) {
                best = s;
                dg[i] *= 1.25;  // that helped — be bolder
            } else {
                // Try stepping DOWN instead.
                g[i] -= 2 * dg[i];
                if (g[i] < 0) g[i] = 0;
                s = evaluate(toGains(g));
                result.runsExecuted++;
                if (s < best) {
                    best = s;
                    dg[i] *= 1.25;
                } else {
                    // Neither helped: restore and shrink the step.
                    g[i] += dg[i];
                    dg[i] *= 0.6;
                }
            }
        }
        if (opt.verbose) {
            std::printf("[autotune %s] iter %d/%d  kP=%.4f kD=%.4f  best=%.1f\n",
                        label, it + 1, opt.iterations, g[0], g[1], best);
        }
        // Converged: steps have shrunk below 3% of the gains.
        const double rel = dg[0] / std::max(g[0], 1e-9) +
                           dg[1] / std::max(g[1], 1e-9);
        if (rel < 0.06) break;
    }

    result.gains = toGains(g);
    result.score = best;
    result.success = best < 1e11;  // false only if every run diverged
    if (opt.verbose) {
        std::printf("[autotune %s] DONE. Copy into your ChassisTunings:\n"
                    "    .kP = %.4f, .kI = %.4f, .kD = %.4f   (score %.1f)\n",
                    label, result.gains.kP, result.gains.kI, result.gains.kD,
                    best);
    }
    return result;
}

} // namespace

// ---------------------------------------------------------------------------
// Lateral: drive testDistance out on even runs, back on odd runs, so the
// robot shuttles in place instead of marching across the room.
// ---------------------------------------------------------------------------
AutotuneResult autotuneLateral(Chassis& chassis, const AutotuneOptions& opt) {
    bool forward = true;

    auto evaluate = [&](const PidGains& gains) {
        const double dist = forward ? opt.testDistance : -opt.testDistance;
        forward = !forward;

        const Pose start = chassis.pose();
        const Point axis{std::cos(start.theta), std::sin(start.theta)};
        const Point target = start.point() + axis * dist;

        MotionParams p;
        p.async = true;
        p.timeoutMs = opt.runTimeoutMs;
        p.lateralGains = gains;

        auto run = scoreRun(
            chassis, opt, std::fabs(dist),
            [&] { chassis.driveDistance(dist, p); },
            [&] {
                // Signed remaining error along the test axis; negative once
                // we've overshot the target.
                const Point cur = chassis.pose().point();
                return (target - cur).dot(axis) * sgn(dist);
            });

        pros::delay(250);  // let the chassis fully stop between runs
        return run.score;
    };

    return twiddle(chassis.tunings().lateral, opt, evaluate, "lateral");
}

// ---------------------------------------------------------------------------
// Angular: alternate +testAngle / -testAngle so heading stays bounded.
// ---------------------------------------------------------------------------
AutotuneResult autotuneAngular(Chassis& chassis, const AutotuneOptions& opt) {
    bool ccw = true;

    auto evaluate = [&](const PidGains& gains) {
        const double delta = ccw ? opt.testAngleDeg : -opt.testAngleDeg;
        ccw = !ccw;

        const double targetDeg = chassis.pose().thetaDeg() + delta;

        MotionParams p;
        p.async = true;
        p.timeoutMs = opt.runTimeoutMs;
        p.angularGains = gains;

        auto run = scoreRun(
            chassis, opt, std::fabs(delta),
            [&] { chassis.turnToHeading(targetDeg, p); },
            [&] {
                return wrapDeg(targetDeg - chassis.pose().thetaDeg()) *
                       sgn(delta);
            });

        pros::delay(250);
        return run.score;
    };

    return twiddle(chassis.tunings().angular, opt, evaluate, "angular");
}

} // namespace aklib
