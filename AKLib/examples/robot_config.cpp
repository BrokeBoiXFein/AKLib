/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  AKLib ROBOT CONFIG SHEET                                                 ║
 * ║  Fill in the blanks below and your robot is ready to drive.               ║
 * ║  Every number you need to change is marked  <-- FILL IN                   ║
 * ║  Full explanations: docs > Getting Started > Quickstart                   ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Copy this file into your PROS project's src/ folder (rename freely) and
 * declare `extern aklib::Chassis chassis;` in a header to use it elsewhere.
 */

#include "aklib/api.hpp"

using namespace aklib::literals;  // enables 24_in, 90_deg, 60_cm, 1.5_tiles...

// ═══════════════════════════════════════════════════════════════════════════
//  SECTION 1 — DRIVE MOTORS
//  List the smart ports on each side. NEGATIVE port = that motor is reversed.
//  (A motor is "reversed" if commanding it forward drives the robot backward.)
// ═══════════════════════════════════════════════════════════════════════════

auto leftMotors = std::make_shared<pros::MotorGroup>(
    std::initializer_list<std::int8_t>{-1, -2, -3},   // <-- FILL IN: left side ports
    pros::MotorGears::blue);                          // <-- FILL IN: cartridge (red/green/blue)

auto rightMotors = std::make_shared<pros::MotorGroup>(
    std::initializer_list<std::int8_t>{4, 5, 6},      // <-- FILL IN: right side ports
    pros::MotorGears::blue);                          // <-- FILL IN: cartridge

// ═══════════════════════════════════════════════════════════════════════════
//  SECTION 2 — DRIVETRAIN DIMENSIONS
// ═══════════════════════════════════════════════════════════════════════════

auto drivetrain = std::make_shared<aklib::TankDrive>(
    leftMotors, rightMotors,
    aklib::DriveConfig{
        .wheelDiameter = 3.25,   // <-- FILL IN: driven wheel diameter (inches)
                                 //     Metric wheel? write e.g.  8.5_cm
        .trackWidth = 12.5,      // <-- FILL IN: left-right wheel contact distance (in)
        .wheelRpm = 450,         // <-- FILL IN: wheel RPM after external gearing
                                 //     e.g. 600 rpm motor, 36:48 down  => 450
        // OPTIONAL (only for .profiled motions — leave zeroed to start):
        .feedforward = {.kS = 0, .kV = 0, .kA = 0},
    });

/* ── HOLONOMIC INSTEAD? ──────────────────────────────────────────────────────
 * Delete the TankDrive above and use (one MotorGroup per corner):
 *
 * auto drivetrain = std::make_shared<aklib::HolonomicDrive>(
 *     fl, fr, bl, br,
 *     aklib::HolonomicDrive::Kind::Mecanum,   // or Kind::XDrive
 *     aklib::DriveConfig{.wheelDiameter = 4.0, .trackWidth = 13.0,
 *                        .wheelRpm = 200});
 *
 * NOTE: holonomic odometry NEEDS a horizontal tracking wheel (Section 3),
 * because the robot strafes on purpose and forward-only sensing can't see it.
 * ────────────────────────────────────────────────────────────────────────── */

// ═══════════════════════════════════════════════════════════════════════════
//  SECTION 3 — ODOMETRY SENSORS  (fill in what you HAVE, delete what you don't)
//
//  Minimum viable:      just the IMU (drive motors fill in for wheels)
//  Recommended:         IMU + vertical wheel + horizontal wheel
//  See the sensor-combo table in docs > Configuration > Sensors.
//
//  TrackingWheel(port, wheelDiameter, offset)
//    port    Rotation sensor smart port; NEGATIVE = reversed. A wheel is
//            reversed if pushing the robot forward (or right, for horizontal
//            wheels) makes its reading go DOWN.
//    offset  signed distance from tracking center, inches:
//              vertical wheel:   negative = left of center,  positive = right
//              horizontal wheel: negative = behind center,   positive = front
// ═══════════════════════════════════════════════════════════════════════════

aklib::OdomSensors sensors{
    .imuPort = 10,          // <-- FILL IN: inertial sensor port (0 = none)
    .imuScale = 1.0,        //     optional: docs > Tutorials > IMU calibration

    .vertical1 = aklib::TrackingWheel(-14, 2.0, -1.25),
    //                        ^port  ^diam  ^offset   <-- FILL IN (or delete
    //                                                    this line entirely)

    .horizontal1 = aklib::TrackingWheel(15, 2.0, -2.5),
    //                                                <-- FILL IN (or delete)
};

// ═══════════════════════════════════════════════════════════════════════════
//  SECTION 4 — TUNING
//  These defaults MOVE, but every robot needs its own numbers. Two options:
//    a) automated:  docs > Tutorials > Auto-tuning  (one button press)
//    b) manual:     docs > Tutorials > Manual PID tuning
// ═══════════════════════════════════════════════════════════════════════════

aklib::ChassisTunings tunings{
    .lateral = {.kP = 0.07, .kI = 0, .kD = 0.30},    // <-- TUNE (inches -> output)
    .angular = {.kP = 0.025, .kI = 0, .kD = 0.16},   // <-- TUNE (degrees -> output)
    .heading = {.kP = 0.012, .kI = 0, .kD = 0.05},   // <-- TUNE (drive-straight hold)
    // Exit tolerances: the defaults (1" / 1.5 deg) suit most robots — see
    // docs > Configuration > Exit conditions before touching these.
};

// ═══════════════════════════════════════════════════════════════════════════
//  SECTION 5 — THE CHASSIS  (nothing to fill in — it's assembled from above)
// ═══════════════════════════════════════════════════════════════════════════

aklib::Chassis chassis(drivetrain, sensors, tunings);

/**
 * Call chassis.calibrate() ONCE in initialize(), with the robot sitting
 * still (IMU calibration takes ~3 seconds):
 *
 *   void initialize() {
 *       chassis.calibrate();
 *   }
 */
