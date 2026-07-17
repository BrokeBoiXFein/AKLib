#pragma once
/**
 * \file pose.hpp
 * Point/Pose types and the small geometry toolbox every algorithm shares.
 *
 * Conventions (see units.hpp):
 *   - Positions in inches. +x is "east"/right, +y is "north"/up when looking
 *     at the field from above. (0,0) defaults to the field's bottom-left
 *     corner, but odometry only cares that you're consistent.
 *   - Pose::theta is stored in RADIANS, math-standard (0 = +x, CCW+).
 *     This is internal; every public Chassis function speaks degrees.
 *     Use Pose::thetaDeg() / Pose::fromDeg() at the boundary.
 */

#include <cmath>

namespace aklib {

// ---------------------------------------------------------------------------
// Angle helpers (radians unless the name says Deg)
// ---------------------------------------------------------------------------

/** Wrap any angle to (-pi, pi]. Feed EVERY heading error through this. */
inline double wrapAngle(double rad) {
    return std::atan2(std::sin(rad), std::cos(rad));
}

/** Wrap any angle to (-180, 180]. Degrees version for user-facing math. */
inline double wrapDeg(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}

inline double degToRad(double deg) { return deg * 3.14159265358979323846 / 180.0; }
inline double radToDeg(double rad) { return rad * 180.0 / 3.14159265358979323846; }

/** sin(x)/x with the x->0 limit handled (used by curvature math). */
inline double sinc(double x) {
    if (std::fabs(x) < 1e-9) return 1.0;
    return std::sin(x) / x;
}

/** Clamp v into [lo, hi]. */
inline double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/** -1, 0, or +1. */
inline double sgn(double v) { return (v > 0) - (v < 0); }

// ---------------------------------------------------------------------------
// Point: a field location (inches)
// ---------------------------------------------------------------------------
struct Point {
    double x = 0;
    double y = 0;

    Point operator+(const Point& o) const { return {x + o.x, y + o.y}; }
    Point operator-(const Point& o) const { return {x - o.x, y - o.y}; }
    Point operator*(double s) const { return {x * s, y * s}; }

    /** Straight-line distance to another point. */
    double distTo(const Point& o) const { return std::hypot(o.x - x, o.y - y); }

    /** Angle (radians, math-standard) of the ray from this point to o. */
    double angleTo(const Point& o) const { return std::atan2(o.y - y, o.x - x); }

    /** This point rotated about the origin by `rad` radians (CCW+). */
    Point rotatedBy(double rad) const {
        const double c = std::cos(rad), s = std::sin(rad);
        return {x * c - y * s, x * s + y * c};
    }

    double dot(const Point& o) const { return x * o.x + y * o.y; }
    double norm() const { return std::hypot(x, y); }
};

// ---------------------------------------------------------------------------
// Pose: position + heading. The single source of truth for "where am I".
// ---------------------------------------------------------------------------
struct Pose {
    double x = 0;      ///< inches
    double y = 0;      ///< inches
    double theta = 0;  ///< RADIANS, math-standard (internal representation)

    Point point() const { return {x, y}; }
    double thetaDeg() const { return radToDeg(theta); }

    /** Build a pose from user units (inches + degrees). */
    static Pose fromDeg(double x, double y, double thetaDeg) {
        return {x, y, degToRad(thetaDeg)};
    }

    double distTo(const Point& p) const { return point().distTo(p); }
    double angleTo(const Point& p) const { return point().angleTo(p); }

    /**
     * Express a field-frame point in this pose's local frame:
     * +x = robot forward, +y = robot left. Used by pure pursuit's
     * curvature math and holonomic motion mixing.
     */
    Point toLocal(const Point& fieldPt) const {
        return (fieldPt - point()).rotatedBy(-theta);
    }
};

/**
 * ChassisSpeeds: what every motion controller OUTPUTS, and what every
 * drivetrain knows how to execute. Keeping this as the universal interface is
 * what lets tank and holonomic drives share all motion code.
 *
 *   vx    forward speed, in/s (robot frame: + = toward robot front)
 *   vy    strafe speed,  in/s (robot frame: + = toward robot left).
 *         Tank drivetrains simply cannot produce vy and ignore it.
 *   omega rotation, rad/s (CCW+)
 *
 * Motions may also run in "open voltage" mode where vx/vy/omega hold
 * normalized outputs in [-1, 1] instead of physical speeds; the drivetrain is
 * told which mode via the apply() call it receives.
 */
struct ChassisSpeeds {
    double vx = 0;
    double vy = 0;
    double omega = 0;
};

} // namespace aklib
