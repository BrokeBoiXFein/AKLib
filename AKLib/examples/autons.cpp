/**
 * \file autons.cpp
 * Example autonomous routines showing every AKLib motion and how they chain.
 * Assumes the `chassis` object from robot_config.cpp.
 */

#include "aklib/api.hpp"

using namespace aklib;
using namespace aklib::literals;

extern aklib::Chassis chassis;

// ─────────────────────────────────────────────────────────────────────────────
// 1. EZ-Template-style auton: think in tiles, drive relative distances.
//    Because odometry runs underneath, the pose stays valid the whole time.
// ─────────────────────────────────────────────────────────────────────────────
void simpleAuton() {
    chassis.setPose(36, 12, 90_deg);        // where the robot starts (x, y, heading)

    chassis.driveDistance(24);              // forward one tile
    chassis.turnToHeading(0_deg);           // face +x
    chassis.driveDistance(1.5_tiles);       // 36 inches
    chassis.driveDistance(-12);             // back up
    chassis.swingToHeading(90_deg, SwingSide::Right);  // pivot on right wheels
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. LemLib-style auton: absolute field coordinates. Mix freely with the
//    relative motions above — they share the same pose.
// ─────────────────────────────────────────────────────────────────────────────
void pointAuton() {
    chassis.setPose(12, 12, 45_deg);

    chassis.moveToPoint(36, 36);                          // arc to a point
    chassis.moveToPoint(12, 60, {.reverse = true});       // drive there backwards
    chassis.moveToPose(60, 60, 0_deg, {.dlead = 0.5});    // arrive FACING +x
    chassis.turnToPoint(72, 72);                          // aim at a goal
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Motion chaining: flow through waypoints without stopping.
//    earlyExitRange hands off before settling; minSpeed keeps momentum.
// ─────────────────────────────────────────────────────────────────────────────
void chainedAuton() {
    chassis.setPose(12, 12, 0_deg);

    chassis.moveToPoint(40, 12, {.minSpeed = 0.35, .earlyExitRange = 6});
    chassis.moveToPoint(60, 30, {.minSpeed = 0.35, .earlyExitRange = 6});
    chassis.moveToPoint(72, 48);            // last one settles normally

    // React to WHY a motion ended:
    if (chassis.driveDistance(20, {.timeoutMs = 1500}) == ExitReason::Stalled) {
        // We hit the goal early — great, score now instead of pushing.
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Async motions: do things WHILE driving.
// ─────────────────────────────────────────────────────────────────────────────
void asyncAuton() {
    chassis.setPose(12, 12, 0_deg);

    chassis.moveToPoint(48, 24, {.async = true});
    chassis.waitUntil(10);                  // ...until 10" from the target
    // spin up intake here
    chassis.waitUntilDone();

    chassis.moveToPose(72, 48, 90_deg, {.async = true});
    pros::delay(500);
    // changed our mind:
    chassis.cancelMotion();
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Pure pursuit: follow a path from the AKLib Planner.
//    (This block is exactly what the planner's "Export C++" button emits.)
// ─────────────────────────────────────────────────────────────────────────────
void pathAuton() {
    chassis.setPose(12, 36, 0_deg);

    aklib::Path sweep = aklib::Path::bezier({
        {12, 36}, {30, 36}, {40, 52}, {56, 52},   // P0 C0 C1 P1
        {66, 52}, {72, 40}, {72, 24},             // C0 C1 P1 (segment 2)
    }, {.maxSpeed = 0.9, .turnK = 1.2});

    chassis.follow(sweep, {.lookahead = 12});
    chassis.follow(sweep, {.reverse = true, .lookahead = 12});  // and back
    // NOTE: designated initializers must appear in the order the fields are
    // declared in MotionParams (C++ rule): maxSpeed, minSpeed, reverse,
    // earlyExitRange, timeoutMs, slewRate, async, brakeAtEnd, dlead,
    // lookahead, profiled, face, then gain/exit overrides.
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Metric team? Everything accepts metric literals.
// ─────────────────────────────────────────────────────────────────────────────
void metricAuton() {
    chassis.setPose(30_cm, 30_cm, 0.5_rad);
    chassis.driveDistance(0.6_m);
    chassis.moveToPoint(1.2_m, 90_cm);
}
