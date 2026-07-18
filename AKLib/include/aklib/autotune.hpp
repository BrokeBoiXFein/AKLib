#pragma once
/**
 * \file autotune.hpp
 * PID auto-tuner: bracket + golden-section search on kD, with kP chosen by
 * YOU. (This replaced an earlier twiddle-based tuner that scored motions by
 * their exit conditions — a sluggish tune could exit "close enough" early and
 * score well, so it converged on gains that were far too low.)
 *
 * ============================================================================
 *  HOW IT WORKS
 * ============================================================================
 *  You pick kP (see "choosing kP" below). The tuner repeatedly runs a test
 *  motion — turns in place for angular, a straight shuttle for lateral,
 *  alternating direction so the robot never walks across the field — and
 *  watches each run for a FIXED duration (no exit conditions, so slow tunes
 *  can't hide):
 *
 *    Phase 1 (BRACKET): if the starting kD is stable, walk DOWN until
 *        oscillation appears (finds the smallest — fastest — stable kD
 *        region); if it oscillates, climb UP until oscillation is gone.
 *    Phase 2 (REFINE): golden-section search inside that bracket, minimizing
 *        cost = wOvershoot·overshoot + wOscillation·oscillations
 *             + wRise·rise_s + wSettle·settle_s + wSteady·steadyErr
 *
 *  Every run prints its metrics to the PROS terminal; the best kD found is
 *  returned (and printed) at the end.
 *
 * ============================================================================
 *  CHOOSING kP (do this first, by hand — it takes 2 minutes)
 * ============================================================================
 *  Raise kP until the test motion is brisk and overshoots a little.
 *  Too low = the tuner can only find a slow tune; too high = no kD can
 *  stabilize it. "Turns hard and rings once or twice" is the sweet spot.
 *
 * ============================================================================
 *  kI WHILE TUNING: keep it 0.
 * ============================================================================
 *  AKLib's PID only integrates inside integralZone — exactly the settling
 *  region the tuner measures — so a nonzero kI adds its own slow limit cycle
 *  that corrupts the overshoot/oscillation metrics. Tune kD with kI = 0,
 *  then add a small kI afterward purely to erase steady-state error.
 *
 * ============================================================================
 *  RUNNING IT
 * ============================================================================
 *  Blocking, ~30–60 s. Trigger once from driver control with clear floor
 *  space (lateral needs ~2× testMagnitude of runway):
 *
 *      if (master.get_digital_new_press(DIGITAL_X)) {
 *          auto r = aklib::autotuneAngular(chassis, 0.03);
 *          if (!r.aborted) chassis.tunings().angular = r.gains;
 *      }
 *
 *  Kill switch: pass an abortCheck (e.g. holding B stops it, keeping the
 *  best-so-far result):
 *
 *      aklib::AutotuneOptions opt;
 *      opt.abortCheck = [&] { return master.get_digital(DIGITAL_B); };
 *
 *  Copy the printed kD into your ChassisTunings when it's done.
 */

#include <functional>

#include "aklib/chassis.hpp"
#include "aklib/control/pid.hpp"

namespace aklib {

/** Search + scoring configuration. Every 0 on a kD field means "auto,
 *  derived from kP" — the defaults are scale-free, so the same options work
 *  for angular (degrees) and lateral (inches) tuning. */
struct AutotuneOptions {
    // ── test motion ─────────────────────────────────────────────────────
    /** Size of each test: degrees (angular) or inches (lateral).
     *  0 = auto: 90 deg / 24 in. */
    double testMagnitude = 0;
    /** Observation window per run, ms. Must comfortably outlast settling. */
    int testDurationMs = 2500;
    /** Pause between runs, ms. */
    int settleBetweenMs = 400;

    // ── bracket search ──────────────────────────────────────────────────
    double kDStart = 0;   ///< 0 = auto: kP * 2
    double kDMin = 0;     ///< bracket floor. 0 = auto: kP * 0.5
    double kDMax = 0;     ///< bracket ceiling. 0 = auto: kP * 200
    double growth = 1.4;  ///< multiply/divide kD by this per bracket step
    int refineIters = 6;  ///< golden-section iterations

    // ── per-run measurement ─────────────────────────────────────────────
    double settleTol = 0.75; ///< deg or in counted as "at target"
    int settleHoldMs = 150;  ///< continuous ms within settleTol = settled
    int ssWindowMs = 300;    ///< tail window averaged for steady-state error

    // ── stability thresholds (what "oscillation gone" means) ────────────
    double overshootTol = 1.5; ///< deg/in of overshoot allowed
    int oscTol = 1;            ///< error sign-changes allowed

    // ── cost weights ────────────────────────────────────────────────────
    double wOvershoot = 2.0;   ///< per deg/in of overshoot   (punishes kD too LOW)
    double wOscillation = 3.0; ///< per oscillation           (punishes kD too LOW)
    double wRise = 2.0;        ///< per second of rise time   (punishes kD too HIGH)
    double wSettle = 1.0;      ///< per second of settle time
    double wSteady = 4.0;      ///< per deg/in of steady-state error

    /** Optional kill switch: polled every loop; return true to stop the
     *  tuner (best-so-far result is kept). */
    std::function<bool()> abortCheck;

    /** Print per-run metrics to the PROS terminal. */
    bool verbose = true;
};

/** What one test run measured (also printed per run when verbose). */
struct AutotuneRun {
    double overshoot = 0;   ///< deg/in past the target
    int oscillations = 0;   ///< error zero-crossings
    double riseTimeMs = 0;  ///< first reach of the target band (large = kD too high, decelerated early)
    double settleTimeMs = 0;///< entered the band and stayed
    double steadyError = 0; ///< avg |error| over the tail window
    double cost = 0;
};

struct AutotuneResult {
    PidGains gains;   ///< {your kP, kI = 0, best kD found}
    double cost = 0;  ///< its cost (comparable within a session)
    bool aborted = false;
    int runsExecuted = 0;
};

/** Tune the TURN kD for your chosen kP. Robot turns +/- testMagnitude
 *  in place, alternating direction. */
AutotuneResult autotuneAngular(Chassis& chassis, double kP,
                               const AutotuneOptions& options = {});

/** Tune the DRIVE-STRAIGHT kD for your chosen kP. Robot shuttles
 *  +/- testMagnitude along its current heading (heading is held with the
 *  chassis's heading PID during each run). */
AutotuneResult autotuneLateral(Chassis& chassis, double kP,
                               const AutotuneOptions& options = {});

} // namespace aklib
