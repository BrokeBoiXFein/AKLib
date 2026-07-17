/**
 * \file chassis.cpp
 * All AKLib motions. Each motion is a 10 ms loop that:
 *   1. reads the pose from odometry,
 *   2. computes ChassisSpeeds via its algorithm,
 *   3. applies them to the drivetrain,
 *   4. asks the Settler if it's done.
 * One motion owns the drivetrain at a time; starting a new one cancels the
 * old one (ExitReason::Interrupted).
 */

#include "aklib/chassis.hpp"

#include <cmath>
#include <mutex>

#include "pros/rtos.hpp"

namespace aklib {

namespace {
constexpr int LOOP_MS = 10;
constexpr double SETTLE_RADIUS = 7.0;  // in: freeze steering this close to a point
}

// ===========================================================================
// Construction / setup
// ===========================================================================

Chassis::Chassis(std::shared_ptr<Drivetrain> drivetrain, OdomSensors sensors,
                 ChassisTunings tunings)
    : drive_(std::move(drivetrain)),
      odom_([&]() -> OdomSensors {
          // No vertical tracking wheel? Fall back to drive-motor odometry
          // automatically (tank only). Slip-prone but better than nothing.
          if (!sensors.vertical1) {
              if (auto* tank = dynamic_cast<TankDrive*>(drive_.get())) {
                  const auto& cfg = drive_->config();
                  sensors.driveLeft = TrackingWheel(
                      tank->leftGroup(), cfg.wheelDiameter,
                      -cfg.trackWidth / 2.0, cfg.wheelRpm);
                  sensors.driveRight = TrackingWheel(
                      tank->rightGroup(), cfg.wheelDiameter,
                      cfg.trackWidth / 2.0, cfg.wheelRpm);
              }
          }
          return sensors;
      }()),
      tunings_(tunings) {}

bool Chassis::calibrate() { return odom_.calibrate(); }

void Chassis::setPose(double x, double y, double thetaDeg) {
    odom_.setPose(x, y, thetaDeg);
}

// ===========================================================================
// Motion plumbing
// ===========================================================================

bool Chassis::takeOwnership_() {
    // Ask any running motion to stop, then claim the (single) motion slot.
    const std::lock_guard<pros::Mutex> lock(motionMutex_);
    if (motionActive_) {
        cancelRequested_ = true;
        while (motionActive_) pros::delay(5);
    }
    cancelRequested_ = false;
    motionActive_ = true;
    return true;
}

ExitReason Chassis::runMotion_(std::function<ExitReason()> body, bool async) {
    takeOwnership_();
    if (!async) {
        const ExitReason r = body();
        lastExit_ = r;
        motionActive_ = false;
        return r;
    }
    // Async: run the identical body in a task; caller returns immediately.
    asyncTask_ = std::make_unique<pros::Task>(
        [this, body = std::move(body)] {
            const ExitReason r = body();
            lastExit_ = r;
            motionActive_ = false;
        },
        "aklib-motion");
    return ExitReason::Running;
}

ExitReason Chassis::waitUntilDone() {
    while (motionActive_) pros::delay(10);
    return lastExit_;
}

void Chassis::waitUntil(double remaining) {
    while (motionActive_ && remainingError_ > remaining) pros::delay(10);
}

void Chassis::cancelMotion() {
    if (motionActive_) cancelRequested_ = true;
}

ExitConditions Chassis::resolveExits_(const MotionParams& p,
                                      const ExitConditions& defaults) const {
    ExitConditions e = p.exits.value_or(defaults);
    if (p.timeoutMs > 0) e.timeoutMs = p.timeoutMs;
    return e;
}

double Chassis::resolveSlew_(const MotionParams& p) const {
    return p.slewRate < 0 ? tunings_.slewRate : p.slewRate;
}

namespace {
/** Cap + floor a normalized output while keeping its sign. */
double capAndFloor(double out, double maxSpeed, double minSpeed) {
    out = clamp(out, -maxSpeed, maxSpeed);
    if (minSpeed > 0 && out != 0 && std::fabs(out) < minSpeed) {
        out = sgn(out) * minSpeed;
    }
    return out;
}
} // namespace

// ===========================================================================
// driveDistance — straight line along the current heading, heading held
// ===========================================================================

ExitReason Chassis::driveDistance(double distance, const MotionParams& params) {
    MotionParams p = params;
    return runMotion_([this, distance, p] {
        return driveDistanceImpl_(distance, p);
    }, params.async);
}

ExitReason Chassis::driveDistanceImpl_(double distance, MotionParams p) {
    const Pose start = pose();
    const double headingRad = start.theta;
    const Point h{std::cos(headingRad), std::sin(headingRad)};  // forward axis
    const Point target = start.point() + h * distance;

    Pid latPid(p.lateralGains.value_or(tunings_.lateral));
    Pid headPid(p.angularGains.value_or(tunings_.heading));
    SlewLimiter slew(resolveSlew_(p));
    Settler settler(resolveExits_(p, tunings_.lateralExits), p.earlyExitRange);

    // Optional trapezoidal profile (needs feedforward constants).
    const Feedforward ff(drive_->config().feedforward);
    const bool profiled = p.profiled && ff.configured();
    TrapezoidalProfile profile;
    if (profiled) {
        const double vTop = drive_->config().maxSpeed() * p.maxSpeed;
        // Sensible default limits: reach top speed in ~0.4 s, brake gentler.
        profile = TrapezoidalProfile(distance, vTop, vTop / 0.4, vTop / 0.6);
    }

    uint32_t now = pros::millis();
    const uint32_t t0 = now;
    ExitReason reason = ExitReason::Running;

    while (reason == ExitReason::Running) {
        if (cancelRequested_) { reason = ExitReason::Interrupted; break; }

        const Pose cur = pose();
        // Signed error along the start heading axis: + means "keep going
        // in the commanded direction".
        const double errForward = (target - cur.point()).dot(h);

        double vx;
        if (profiled) {
            // Follow the plan: feedforward from planned velocity/accel plus a
            // small PID on planned-vs-actual position.
            const double t = (pros::millis() - t0) / 1000.0;
            const auto s = profile.at(t);
            const double plannedProgress = s.position * sgn(distance);
            const double progress = (cur.point() - start.point()).dot(h);
            const double ffNorm =
                ff.calculate(s.velocity, s.accel) / 12.0 * sgn(distance);
            vx = ffNorm + latPid.update(plannedProgress - progress, LOOP_MS / 1000.0);
        } else {
            vx = latPid.update(errForward, LOOP_MS / 1000.0);
        }
        vx = slew.calculate(vx, LOOP_MS / 1000.0);
        vx = capAndFloor(vx, p.maxSpeed, p.minSpeed);

        // Hold the starting heading the whole way (resists drift/defense).
        const double errHeadDeg = wrapDeg(radToDeg(headingRad) - cur.thetaDeg());
        const double omega = headPid.update(errHeadDeg, LOOP_MS / 1000.0);

        drive_->applyNormalized({vx, 0, omega});

        remainingError_ = std::fabs(errForward);
        reason = settler.update(errForward, odom_.speed(), LOOP_MS);
        pros::Task::delay_until(&now, LOOP_MS);
    }

    // Chained handoff keeps the motors running; everything else stops them.
    if (reason != ExitReason::EarlyExit) drive_->stop(p.brakeAtEnd);
    return reason;
}

// ===========================================================================
// Turns (in place + swings). lockSide: -1 lock left, +1 lock right, 0 none.
// ===========================================================================

ExitReason Chassis::turnToHeading(double thetaDeg, const MotionParams& params) {
    MotionParams p = params;
    return runMotion_([this, thetaDeg, p] {
        return turnToHeadingImpl_(thetaDeg, p, 0);
    }, params.async);
}

ExitReason Chassis::turnToPoint(double x, double y, const MotionParams& params) {
    MotionParams p = params;
    // NAN target signals "recompute from the point each loop" (see impl).
    return runMotion_([this, x, y, p]() -> ExitReason {
        MotionParams q = p;
        q.face = std::nullopt;
        // Recompute the target heading live so drift mid-turn is corrected.
        Pid angPid(q.angularGains.value_or(tunings_.angular));
        SlewLimiter slew(resolveSlew_(q));
        Settler settler(resolveExits_(q, tunings_.angularExits), q.earlyExitRange);
        uint32_t now = pros::millis();
        ExitReason reason = ExitReason::Running;
        while (reason == ExitReason::Running) {
            if (cancelRequested_) { reason = ExitReason::Interrupted; break; }
            const Pose cur = pose();
            double targetDeg = radToDeg(cur.angleTo({x, y}));
            if (q.reverse) targetDeg += 180.0;  // point the BACK at it
            const double errDeg = wrapDeg(targetDeg - cur.thetaDeg());
            double omega = angPid.update(errDeg, LOOP_MS / 1000.0);
            omega = slew.calculate(omega, LOOP_MS / 1000.0);
            omega = capAndFloor(omega, q.maxSpeed, q.minSpeed);
            drive_->applyNormalized({0, 0, omega});
            remainingError_ = std::fabs(errDeg);
            reason = settler.update(errDeg, odom_.angularSpeedDeg(), LOOP_MS);
            pros::Task::delay_until(&now, LOOP_MS);
        }
        if (reason != ExitReason::EarlyExit) drive_->stop(p.brakeAtEnd);
        return reason;
    }, params.async);
}

ExitReason Chassis::swingToHeading(double thetaDeg, SwingSide lockedSide,
                                   const MotionParams& params) {
    MotionParams p = params;
    const int lock = lockedSide == SwingSide::Left ? -1 : 1;
    return runMotion_([this, thetaDeg, p, lock] {
        return turnToHeadingImpl_(thetaDeg, p, lock);
    }, params.async);
}

ExitReason Chassis::turnToHeadingImpl_(double thetaDeg, MotionParams p,
                                       int lockSide) {
    Pid angPid(p.angularGains.value_or(tunings_.angular));
    SlewLimiter slew(resolveSlew_(p));
    Settler settler(resolveExits_(p, tunings_.angularExits), p.earlyExitRange);

    auto* tank = dynamic_cast<TankDrive*>(drive_.get());
    if (lockSide != 0 && !tank) lockSide = 0;  // swings are tank-only

    uint32_t now = pros::millis();
    ExitReason reason = ExitReason::Running;

    while (reason == ExitReason::Running) {
        if (cancelRequested_) { reason = ExitReason::Interrupted; break; }

        const Pose cur = pose();
        const double errDeg = wrapDeg(thetaDeg - cur.thetaDeg());
        double omega = angPid.update(errDeg, LOOP_MS / 1000.0);
        omega = slew.calculate(omega, LOOP_MS / 1000.0);
        omega = capAndFloor(omega, p.maxSpeed, p.minSpeed);

        if (lockSide == 0) {
            drive_->applyNormalized({0, 0, omega});
        } else if (lockSide < 0) {
            // Left side planted; drive the right. omega CCW+ -> right forward.
            tank->tank(0, clamp(2.0 * omega, -p.maxSpeed, p.maxSpeed));
        } else {
            // Right side planted; drive the left. CCW+ -> left backward.
            tank->tank(clamp(-2.0 * omega, -p.maxSpeed, p.maxSpeed), 0);
        }

        remainingError_ = std::fabs(errDeg);
        reason = settler.update(errDeg, odom_.angularSpeedDeg(), LOOP_MS);
        pros::Task::delay_until(&now, LOOP_MS);
    }

    if (reason != ExitReason::EarlyExit) drive_->stop(p.brakeAtEnd);
    return reason;
}

// ===========================================================================
// moveToPoint — drive to (x, y). The odometry payoff motion.
// ===========================================================================

ExitReason Chassis::moveToPoint(double x, double y, const MotionParams& params) {
    MotionParams p = params;
    return runMotion_([this, x, y, p] { return moveToPointImpl_(x, y, p); },
                      params.async);
}

ExitReason Chassis::moveToPointImpl_(double x, double y, MotionParams p) {
    const Point target{x, y};
    Pid latPid(p.lateralGains.value_or(tunings_.lateral));
    Pid angPid(p.angularGains.value_or(tunings_.angular));
    Pid headPid(p.angularGains.value_or(tunings_.heading));
    SlewLimiter slew(resolveSlew_(p));
    Settler settler(resolveExits_(p, tunings_.lateralExits), p.earlyExitRange);

    const double holdHeadingDeg = p.face.value_or(pose().thetaDeg());
    uint32_t now = pros::millis();
    ExitReason reason = ExitReason::Running;

    while (reason == ExitReason::Running) {
        if (cancelRequested_) { reason = ExitReason::Interrupted; break; }

        const Pose cur = pose();
        const double d = cur.distTo(target);
        ChassisSpeeds out{};

        if (drive_->holonomic()) {
            // Holonomic: strafe straight at the point; heading is independent.
            const Point local = cur.toLocal(target);        // robot frame
            const double mag = capAndFloor(
                slew.calculate(latPid.update(d, LOOP_MS / 1000.0),
                               LOOP_MS / 1000.0),
                p.maxSpeed, p.minSpeed);
            const double n = std::max(local.norm(), 1e-6);
            out.vx = mag * local.x / n;
            out.vy = mag * local.y / n;
            const double errHead = wrapDeg(holdHeadingDeg - cur.thetaDeg());
            out.omega = headPid.update(errHead, LOOP_MS / 1000.0);
        } else {
            // Tank: steer toward the point, drive scaled by alignment.
            double targetAngle = cur.angleTo(target);
            if (p.reverse) targetAngle += units::PI;  // lead with the back
            const double eAng = wrapAngle(targetAngle - cur.theta);

            double lat = latPid.update(d, LOOP_MS / 1000.0);
            // KEY TRICK: scale forward output by alignment. Badly misaligned
            // (cos < 0) even drives away briefly — that's what makes the
            // approach a smooth arc instead of stop-turn-go.
            double vx = lat * std::cos(eAng);
            if (p.reverse) vx = -vx;

            double omega;
            if (d < SETTLE_RADIUS) {
                // atan2 goes unstable on top of the point: stop steering,
                // let the lateral controller finish the last few inches.
                omega = 0;
            } else {
                omega = angPid.update(radToDeg(eAng), LOOP_MS / 1000.0);
            }

            vx = slew.calculate(vx, LOOP_MS / 1000.0);
            out.vx = capAndFloor(vx, p.maxSpeed, p.minSpeed);
            out.omega = omega;
        }

        drive_->applyNormalized(out);
        remainingError_ = d;
        reason = settler.update(d, odom_.speed(), LOOP_MS);
        pros::Task::delay_until(&now, LOOP_MS);
    }

    if (reason != ExitReason::EarlyExit) drive_->stop(p.brakeAtEnd);
    return reason;
}

// ===========================================================================
// moveToPose — boomerang: arrive at (x, y) FACING thetaDeg
// ===========================================================================

ExitReason Chassis::moveToPose(double x, double y, double thetaDeg,
                               const MotionParams& params) {
    MotionParams p = params;
    return runMotion_([this, x, y, thetaDeg, p] {
        return moveToPoseImpl_(x, y, thetaDeg, p);
    }, params.async);
}

ExitReason Chassis::moveToPoseImpl_(double x, double y, double thetaDeg,
                                    MotionParams p) {
    const Point target{x, y};
    const double thetaRad = degToRad(thetaDeg);
    Pid latPid(p.lateralGains.value_or(tunings_.lateral));
    Pid angPid(p.angularGains.value_or(tunings_.angular));
    SlewLimiter slew(resolveSlew_(p));
    Settler settler(resolveExits_(p, tunings_.lateralExits), p.earlyExitRange);

    uint32_t now = pros::millis();
    ExitReason reason = ExitReason::Running;

    while (reason == ExitReason::Running) {
        if (cancelRequested_) { reason = ExitReason::Interrupted; break; }

        const Pose cur = pose();
        const double h = cur.distTo(target);

        // THE BOOMERANG: a "carrot" pulled back from the target along the
        // desired arrival heading. Far away the carrot shapes the approach;
        // as h -> 0 it converges to the target, arriving aligned.
        const Point carrot{target.x - h * p.dlead * std::cos(thetaRad),
                           target.y - h * p.dlead * std::sin(thetaRad)};

        ChassisSpeeds out{};
        if (drive_->holonomic()) {
            // Strafe at the carrot; rotate toward the final heading directly.
            const Point local = cur.toLocal(carrot);
            const double mag = capAndFloor(
                slew.calculate(latPid.update(h, LOOP_MS / 1000.0),
                               LOOP_MS / 1000.0),
                p.maxSpeed, p.minSpeed);
            const double n = std::max(local.norm(), 1e-6);
            out.vx = mag * local.x / n;
            out.vy = mag * local.y / n;
            out.omega = angPid.update(wrapDeg(thetaDeg - cur.thetaDeg()),
                                      LOOP_MS / 1000.0);
        } else {
            double targetAngle = cur.angleTo(carrot);
            if (p.reverse) targetAngle += units::PI;
            const double eAng = wrapAngle(targetAngle - cur.theta);

            // Distance PID runs on distance to the TARGET (not the carrot) so
            // speed tapers correctly at arrival.
            double vx = latPid.update(h, LOOP_MS / 1000.0) * std::cos(eAng);
            if (p.reverse) vx = -vx;

            double omega;
            if (h < SETTLE_RADIUS) {
                // Close in: aim the FINAL heading (this is what makes the
                // arrival heading stick) and stop chasing the carrot.
                omega = angPid.update(
                    wrapDeg(thetaDeg + (p.reverse ? 180.0 : 0.0) -
                            cur.thetaDeg()),
                    LOOP_MS / 1000.0) * 0.7;
            } else {
                omega = angPid.update(radToDeg(eAng), LOOP_MS / 1000.0);
            }

            vx = slew.calculate(vx, LOOP_MS / 1000.0);
            out.vx = capAndFloor(vx, p.maxSpeed, p.minSpeed);
            out.omega = omega;
        }

        drive_->applyNormalized(out);
        remainingError_ = h;
        reason = settler.update(h, odom_.speed(), LOOP_MS);
        pros::Task::delay_until(&now, LOOP_MS);
    }

    if (reason != ExitReason::EarlyExit) drive_->stop(p.brakeAtEnd);
    return reason;
}

// ===========================================================================
// follow — pure pursuit over a Path
// ===========================================================================

ExitReason Chassis::follow(const Path& path, const MotionParams& params) {
    MotionParams p = params;
    return runMotion_([this, path, p] { return followImpl_(path, p); },
                      params.async);
}

ExitReason Chassis::followImpl_(const Path& path, MotionParams p) {
    if (path.empty()) return ExitReason::Settled;

    const double L = p.lookahead > 0 ? p.lookahead : 10.0;
    SlewLimiter slew(resolveSlew_(p));
    Settler settler(resolveExits_(p, tunings_.lateralExits), 0);
    Pid headPid(tunings_.heading);

    std::size_t pathIndex = 0;
    uint32_t now = pros::millis();
    ExitReason reason = ExitReason::Running;

    while (reason == ExitReason::Running) {
        if (cancelRequested_) { reason = ExitReason::Interrupted; break; }

        const Pose cur = pose();
        const Point look = path.lookaheadPoint(cur.point(), L, pathIndex);
        const auto& pathPt = path.points()[pathIndex];
        const double remainingOnPath = path.length() - pathPt.distance;

        // Hand the endgame to moveToPoint: pure pursuit has nothing to chase
        // once the lookahead reaches the end of the path.
        if (remainingOnPath <= L) break;

        // Planned speed at this point in the path (already curvature- and
        // decel-aware from Path::finalize_), capped by the caller.
        const double s = capAndFloor(
            slew.calculate(std::min(pathPt.speed, p.maxSpeed), LOOP_MS / 1000.0),
            p.maxSpeed, p.minSpeed);

        ChassisSpeeds out{};
        if (drive_->holonomic()) {
            // Holonomic: velocity vector straight at the lookahead point;
            // heading follows params.face or the path direction.
            const Point local = cur.toLocal(look);
            const double n = std::max(local.norm(), 1e-6);
            out.vx = s * local.x / n;
            out.vy = s * local.y / n;
            const std::size_t ahead =
                std::min(pathIndex + 4, path.points().size() - 1);
            const double faceDeg = p.face.value_or(radToDeg(
                pathPt.p.angleTo(path.points()[ahead].p)));
            out.omega = headPid.update(wrapDeg(faceDeg - cur.thetaDeg()),
                                       LOOP_MS / 1000.0);
        } else {
            // Tank pure pursuit. Work in the frame the robot is driving in
            // (flip 180 degrees when following in reverse).
            Pose frame = cur;
            if (p.reverse) frame.theta = wrapAngle(frame.theta + units::PI);
            const Point local = frame.toLocal(look);
            // Curvature of the arc through the robot and the lookahead point:
            // kappa = 2 * lateral_offset / L^2. local.y is +left, and a
            // leftward point needs a CCW (+omega) turn — signs line up.
            const double kappa = 2.0 * local.y / (L * L);
            out.vx = p.reverse ? -s : s;
            // Differential mix: omega such that left/right = v(1 -/+ kT/2).
            out.omega = s * kappa * (drive_->config().trackWidth / 2.0);
        }

        drive_->applyNormalized(out);
        remainingError_ = remainingOnPath;
        // Along a path the settler mostly watches for stalls and timeout.
        reason = settler.update(remainingOnPath, odom_.speed(), LOOP_MS);
        pros::Task::delay_until(&now, LOOP_MS);
    }

    if (reason != ExitReason::Running) {
        // Stall / timeout / cancel mid-path.
        if (reason != ExitReason::EarlyExit) drive_->stop(p.brakeAtEnd);
        return reason;
    }

    // Finish: settle onto the final point with the standard point motion.
    MotionParams endParams = p;
    endParams.async = false;
    endParams.minSpeed = 0;
    return moveToPointImpl_(path.back().p.x, path.back().p.y, endParams);
}

} // namespace aklib
