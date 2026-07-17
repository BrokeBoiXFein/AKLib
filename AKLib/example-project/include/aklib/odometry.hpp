#pragma once
/**
 * \file odometry.hpp
 * Odometry: the background task that keeps track of where the robot is.
 * Everything pose-based (moveToPoint, boomerang, pure pursuit) reads from it.
 *
 * ============================================================================
 *  SENSOR COMBOS — what you can plug in, best first
 * ============================================================================
 *  The OdomSensors struct below is a menu. Fill in what your robot has and
 *  leave the rest as std::nullopt / nullptr. Requirements per feature:
 *
 *  | Setup                                   | Heading | Fwd dist | Sideways |
 *  |------------------------------------------|--------|----------|----------|
 *  | IMU + vertical wheel + horizontal wheel  | best   | best     | yes      | <- recommended
 *  | IMU + vertical wheel                     | best   | best     | no       |
 *  | IMU + drive motors (no tracking wheels)  | best   | slip-prone| no      | <- zero extra hardware
 *  | 2 vertical wheels (L+R), no IMU          | ok     | best     | no       |
 *  | 2 vertical + horizontal, no IMU          | ok     | best     | yes      |
 *
 *  "Sideways" = detects being pushed laterally / drift during turns.
 *  Holonomic drives REQUIRE a horizontal wheel (or strafe-capable motor
 *  odometry via the drivetrain) because they translate sideways on purpose.
 *
 *  Rules the tracker applies automatically:
 *   - Heading: IMU if present (avg of both if two), else difference of the
 *     two vertical wheels, else drive-motor differential (last resort).
 *   - Forward: vertical wheel 1 if present (avg with wheel 2 if both),
 *     else drive-motor average.
 *   - Sideways: horizontal wheel if present, else assumed zero.
 */

#include <memory>
#include <optional>

#include "pros/imu.hpp"
#include "pros/rtos.hpp"

#include "aklib/pose.hpp"
#include "aklib/sensors/tracking_wheel.hpp"

namespace aklib {

/**
 * The sensor menu. Construct with designated initializers so it reads like a
 * checklist (only name what you have):
 *
 *   aklib::OdomSensors sensors {
 *       .imuPort = 10,
 *       .vertical1 = aklib::TrackingWheel(-14, 2.75, -1.2),
 *       .horizontal1 = aklib::TrackingWheel(15, 2.75, -2.5),
 *   };
 */
struct OdomSensors {
    /** Port of the V5 inertial sensor. 0 = no IMU. */
    int imuPort = 0;
    /** Optional second IMU: readings are averaged (halves drift). 0 = none. */
    int imuPort2 = 0;
    /** IMU scale correction: true_rotation / imu_reported_rotation. Measure by
     *  spinning the robot 10 full turns (see docs "Calibrating the IMU"). */
    double imuScale = 1.0;

    /** Vertical (forward-rolling) tracking wheel. */
    std::optional<TrackingWheel> vertical1;
    /** Second vertical wheel, opposite side (enables encoder heading). */
    std::optional<TrackingWheel> vertical2;
    /** Horizontal (sideways-rolling) tracking wheel. */
    std::optional<TrackingWheel> horizontal1;

    /**
     * Fallback: use drive motors as vertical wheels when you have no vertical
     * tracking wheel. Filled in for you by Chassis if you leave vertical1
     * empty — you normally never set these two yourself.
     */
    std::optional<TrackingWheel> driveLeft;
    std::optional<TrackingWheel> driveRight;
};

class Odometry {
public:
    explicit Odometry(OdomSensors sensors);

    /**
     * Calibrate the IMU (robot MUST be still, takes ~2-3 s), zero all
     * tracking wheels, and start the 10 ms background tracking task.
     * Call once in initialize(). Blocks until calibration finishes.
     * \return false if the IMU failed to calibrate (odometry falls back to
     *         encoder heading and keeps going — but check your wiring!).
     */
    bool calibrate();

    /** Current best pose estimate. Thread-safe. */
    Pose pose() const;

    /** Overwrite the pose (inches, degrees). Call at the start of every auton
     *  to tell the robot where it starts, and after any external correction. */
    void setPose(double x, double y, double thetaDeg);
    void setPose(const Pose& p);

    /** Filtered field-frame speed (in/s) and turn rate (deg/s). Used by
     *  settling/stall detection and available for your own logic. */
    double speed() const;
    double angularSpeedDeg() const;

    /** True once calibrate() has completed. */
    bool ready() const { return ready_; }

    /** Stop the tracking task (normally never needed). */
    ~Odometry();

    Odometry(const Odometry&) = delete;
    Odometry& operator=(const Odometry&) = delete;

private:
    void trackingLoop_();   ///< the 10 ms task body
    double readHeadingRad_(); ///< heading from best available source

    OdomSensors s_;
    std::unique_ptr<pros::Imu> imu_;
    std::unique_ptr<pros::Imu> imu2_;
    std::unique_ptr<pros::Task> task_;

    mutable pros::Mutex mutex_;
    Pose pose_{};
    double speed_ = 0;
    double angSpeedDeg_ = 0;
    bool ready_ = false;

    // per-iteration state
    double lastVert1_ = 0, lastVert2_ = 0, lastHorz_ = 0;
    double lastDriveL_ = 0, lastDriveR_ = 0;
    double lastHeading_ = 0;
};

} // namespace aklib
