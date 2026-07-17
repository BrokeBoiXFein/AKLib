#pragma once
/**
 * \file chassis.hpp
 * Chassis: the class your auton talks to. Owns the drivetrain, the odometry
 * task, and every motion algorithm.
 *
 * ============================================================================
 *  FEATURE -> REQUIREMENTS CHEAT SHEET
 * ============================================================================
 *  driveDistance / turnToHeading / swing   encoders + IMU recommended
 *  moveToPoint / moveToPose / turnToPoint  odometry (any combo, see
 *                                          odometry.hpp table)
 *  follow (pure pursuit)                   odometry + a Path
 *  .profiled = true motions                feedforward constants in
 *                                          DriveConfig (see docs tutorial)
 *  swingToHeading                          tank drive only
 *  holonomic strafing motions              HolonomicDrive + horizontal wheel
 *
 * See examples/robot_config.cpp for the fill-in-the-blanks setup sheet.
 */

#include <atomic>
#include <functional>
#include <memory>
#include <optional>

#include "pros/rtos.hpp"

#include "aklib/control/exit.hpp"
#include "aklib/control/pid.hpp"
#include "aklib/control/profile.hpp"
#include "aklib/control/slew.hpp"
#include "aklib/drivetrain.hpp"
#include "aklib/odometry.hpp"
#include "aklib/path.hpp"
#include "aklib/units.hpp"

namespace aklib {

/**
 * Default gains + tolerances for the chassis. Set once at construction; any
 * motion call can override any piece via MotionParams.
 * Starting values below are deliberately conservative — run the auto-tuner
 * (aklib/autotune.hpp) or the manual tuning tutorial to get YOUR numbers.
 */
struct ChassisTunings {
    /** Lateral PID: error in inches -> output in [-1, 1]. */
    PidGains lateral{.kP = 0.07, .kI = 0, .kD = 0.30};
    /** Angular PID: error in degrees -> output in [-1, 1]. */
    PidGains angular{.kP = 0.025, .kI = 0, .kD = 0.16};
    /** Heading-hold PID used to keep drives straight (often softer than the
     *  turning PID). */
    PidGains heading{.kP = 0.012, .kI = 0, .kD = 0.05};

    ExitConditions lateralExits{};                    // inches
    ExitConditions angularExits{.smallError = 1.5,    // degrees
                                .bigError = 5.0,
                                .stallVelocity = 3.0};

    /** Default slew: max output increase per second (0 = off). */
    double slewRate = 5.0;
};

/**
 * Per-call knobs. Every field is optional — designated initializers make
 * calls read nicely:
 *
 *   chassis.moveToPoint(72, 48, {.maxSpeed = 0.8, .reverse = true});
 *   chassis.driveDistance(24, {.earlyExitRange = 3, .minSpeed = 0.3});
 */
struct MotionParams {
    double maxSpeed = 1.0;       ///< output cap, fraction of full power (0..1]
    double minSpeed = 0.0;       ///< output floor — keeps momentum for chaining
    bool reverse = false;        ///< drive backwards toward the target
    double earlyExitRange = 0;   ///< exit (at speed) within this error [in/deg]
    int timeoutMs = 0;           ///< 0 = use the exit-conditions default
    double slewRate = -1;        ///< -1 = chassis default, 0 = disable
    bool async = false;          ///< return immediately; motion runs in a task
    bool brakeAtEnd = true;      ///< brake (true) or coast (false) on finish

    // --- motion-specific -----------------------------------------------------
    double dlead = 0.6;          ///< boomerang aggressiveness (moveToPose)
    double lookahead = 10.0;     ///< pure pursuit lookahead radius, inches
    bool profiled = false;       ///< driveDistance/turnToHeading: trapezoid+FF
    std::optional<double> face;  ///< holonomic only: heading (deg) to hold
                                 ///< while translating (default: keep current)

    // --- overrides (omit to use chassis defaults) ----------------------------
    std::optional<PidGains> lateralGains;
    std::optional<PidGains> angularGains;
    std::optional<ExitConditions> exits;
};

/** Which side stays planted during a swing turn. */
enum class SwingSide { Left, Right };

class Chassis {
public:
    /**
     * \param drivetrain  a TankDrive or HolonomicDrive (see drivetrain.hpp)
     * \param sensors     the odometry sensor menu (see odometry.hpp).
     *                    Tank drives with no vertical tracking wheel
     *                    automatically fall back to drive-motor odometry.
     * \param tunings     PID gains / exit conditions / slew defaults
     */
    Chassis(std::shared_ptr<Drivetrain> drivetrain, OdomSensors sensors,
            ChassisTunings tunings = {});

    // =========================================================================
    // Setup / state
    // =========================================================================

    /** Calibrate IMU + start odometry. Call ONCE in initialize(); blocks ~3 s.
     *  \return false if the IMU failed (check wiring / port). */
    bool calibrate();

    /** Tell the robot where it starts (inches, degrees). Call at the top of
     *  every autonomous routine. */
    void setPose(double x, double y, double thetaDeg);

    /** Where am I? (inches; .thetaDeg() for heading in degrees) */
    Pose pose() const { return odom_.pose(); }

    Odometry& odometry() { return odom_; }
    std::shared_ptr<Drivetrain> drivetrain() { return drive_; }
    ChassisTunings& tunings() { return tunings_; }

    // =========================================================================
    // Motions — every one returns WHY it ended (see ExitReason)
    // =========================================================================

    /** Drive `distance` inches along the CURRENT heading (negative = back up).
     *  Heading is actively held the whole way. Odometry keeps tracking, so
     *  you can mix this freely with moveToPoint. */
    ExitReason driveDistance(double distance, const MotionParams& params = {});

    /** Rotate in place to an absolute heading (degrees, math convention —
     *  wrap compass numbers in aklib::compass()). */
    ExitReason turnToHeading(double thetaDeg, const MotionParams& params = {});

    /** Rotate in place to face a field point (or directly away from it with
     *  .reverse = true). */
    ExitReason turnToPoint(double x, double y, const MotionParams& params = {});

    /** Swing turn: pivot to a heading with one side of a TANK drive locked.
     *  On holonomic drives this falls back to turnToHeading. */
    ExitReason swingToHeading(double thetaDeg, SwingSide lockedSide,
                              const MotionParams& params = {});

    /** Drive to a field point from wherever we are. Requires odometry. */
    ExitReason moveToPoint(double x, double y, const MotionParams& params = {});

    /** Boomerang: arrive at a point FACING thetaDeg, in one curved motion.
     *  Tune curve shape with params.dlead (0 = straight-ish, 1 = very wide). */
    ExitReason moveToPose(double x, double y, double thetaDeg,
                          const MotionParams& params = {});

    /** Pure pursuit: follow a Path (build with the AKLib Planner or
     *  Path::bezier). Finishes with a moveToPoint onto the final point. */
    ExitReason follow(const Path& path, const MotionParams& params = {});

    // =========================================================================
    // Async control (for params.async = true)
    // =========================================================================

    /** Block until the current async motion finishes; returns its ExitReason. */
    ExitReason waitUntilDone();

    /** Block until the current motion's remaining error drops below
     *  `remaining` (inches or degrees, matching the motion). Then you can
     *  fire an intake, cancel, chain, etc. */
    void waitUntil(double remaining);

    /** Cancel the current motion (it returns ExitReason::Interrupted). */
    void cancelMotion();

    /** Is a motion currently running? */
    bool isMoving() const { return motionActive_; }

    /** ExitReason of the most recently completed motion. */
    ExitReason lastExitReason() const { return lastExit_; }

private:
    // The blocking cores. Public functions route here (possibly via a task).
    ExitReason driveDistanceImpl_(double distance, MotionParams p);
    ExitReason turnToHeadingImpl_(double thetaDeg, MotionParams p,
                                  int lockSide /* -1 left, 0 none, +1 right */);
    ExitReason moveToPointImpl_(double x, double y, MotionParams p);
    ExitReason moveToPoseImpl_(double x, double y, double thetaDeg,
                               MotionParams p);
    ExitReason followImpl_(const Path& path, MotionParams p);

    // Shared motion plumbing.
    ExitReason runMotion_(std::function<ExitReason()> body, bool async);
    bool takeOwnership_();       ///< cancel current motion, become the owner
    void finishMotion_(ExitReason r, bool brake);

    ExitConditions resolveExits_(const MotionParams& p,
                                 const ExitConditions& defaults) const;
    double resolveSlew_(const MotionParams& p) const;

    std::shared_ptr<Drivetrain> drive_;
    Odometry odom_;
    ChassisTunings tunings_;

    // Motion-ownership state (one motion may command the drive at a time).
    pros::Mutex motionMutex_;
    std::atomic<bool> motionActive_{false};
    std::atomic<bool> cancelRequested_{false};
    std::atomic<double> remainingError_{0};
    ExitReason lastExit_ = ExitReason::Settled;
    std::unique_ptr<pros::Task> asyncTask_;
};

} // namespace aklib
