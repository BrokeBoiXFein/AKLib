/**
 * \file drivetrain.cpp
 * Tank and holonomic mixing: ChassisSpeeds -> per-wheel voltages.
 */

#include "aklib/drivetrain.hpp"

#include <algorithm>
#include <cmath>

namespace aklib {

namespace {

constexpr double MAX_MV = 12000.0;  // V5 motors take millivolts, +/-12000

/** Scale a set of wheel outputs so the largest magnitude is <= 1, preserving
 *  their ratios (keeps turn authority when saturated). */
void normalizeInPlace(std::vector<double>& outs) {
    double biggest = 1.0;
    for (double o : outs) biggest = std::max(biggest, std::fabs(o));
    for (double& o : outs) o /= biggest;
}

} // namespace

// ---------------------------------------------------------------------------
// TankDrive
// ---------------------------------------------------------------------------

TankDrive::TankDrive(std::shared_ptr<pros::MotorGroup> left,
                     std::shared_ptr<pros::MotorGroup> right,
                     const DriveConfig& cfg)
    : Drivetrain(cfg), left_(std::move(left)), right_(std::move(right)) {}

void TankDrive::applyNormalized(const ChassisSpeeds& speeds) {
    // vy is physically impossible on tank; motions know this via holonomic()
    // and fold everything into vx/omega before we get here.
    std::vector<double> outs = {speeds.vx - speeds.omega,   // left
                                speeds.vx + speeds.omega};  // right
    // omega is CCW+: a positive (counterclockwise) turn means the RIGHT side
    // speeds up and the left slows down.
    normalizeInPlace(outs);
    left_->move_voltage(static_cast<int>(outs[0] * MAX_MV));
    right_->move_voltage(static_cast<int>(outs[1] * MAX_MV));
}

void TankDrive::tank(double leftOut, double rightOut) {
    left_->move_voltage(static_cast<int>(clamp(leftOut, -1.0, 1.0) * MAX_MV));
    right_->move_voltage(static_cast<int>(clamp(rightOut, -1.0, 1.0) * MAX_MV));
}

void TankDrive::arcade(double throttle, double turn) {
    // Driver-facing arcade uses "turn right = positive stick", which is a
    // clockwise (negative omega) rotation in our convention.
    applyNormalized({throttle, 0, -turn});
}

void TankDrive::stop(bool hold) {
    const auto mode = hold ? pros::MotorBrake::hold : pros::MotorBrake::coast;
    left_->set_brake_mode_all(mode);
    right_->set_brake_mode_all(mode);
    left_->brake();
    right_->brake();
}

double TankDrive::averageWheelSpeed() {
    // Motor velocity is reported in RPM of the motor; convert to wheel in/s.
    auto avg = [](const std::vector<double>& v) {
        double s = 0;
        for (double x : v) s += std::fabs(x);
        return v.empty() ? 0 : s / v.size();
    };
    const double motorRpm =
        (avg(left_->get_actual_velocity_all()) +
         avg(right_->get_actual_velocity_all())) / 2.0;
    // get_actual_velocity is cartridge-relative RPM; approximate wheel speed
    // via configured wheelRpm assuming full output ~= cartridge max.
    double cartridge = 200;
    switch (left_->get_gearing()) {
        case pros::MotorGears::red:  cartridge = 100; break;
        case pros::MotorGears::green: cartridge = 200; break;
        case pros::MotorGears::blue: cartridge = 600; break;
        default: break;
    }
    const double wheelRpm = motorRpm * (cfg_.wheelRpm / cartridge);
    return wheelRpm / 60.0 * cfg_.wheelDiameter * units::PI;
}

// ---------------------------------------------------------------------------
// HolonomicDrive
// ---------------------------------------------------------------------------

HolonomicDrive::HolonomicDrive(std::shared_ptr<pros::MotorGroup> frontLeft,
                               std::shared_ptr<pros::MotorGroup> frontRight,
                               std::shared_ptr<pros::MotorGroup> backLeft,
                               std::shared_ptr<pros::MotorGroup> backRight,
                               Kind kind, const DriveConfig& cfg)
    : Drivetrain(cfg), fl_(std::move(frontLeft)), fr_(std::move(frontRight)),
      bl_(std::move(backLeft)), br_(std::move(backRight)), kind_(kind) {}

void HolonomicDrive::applyNormalized(const ChassisSpeeds& speeds) {
    // Standard holonomic mixing. Convention reminder:
    //   vx = forward, vy = LEFT (math standard), omega = CCW+.
    // Mecanum/X mixing formulas are usually written with strafe-right and
    // clockwise-turn positive, so flip signs at the boundary:
    const double forward = speeds.vx;
    const double strafeRight = -speeds.vy;
    const double turnCw = -speeds.omega;

    // Same mixing applies to X-drive and mecanum (X-drive trades ~30% top
    // speed for the diagonal geometry; the *mix* is identical).
    std::vector<double> outs = {
        forward + strafeRight + turnCw,   // front-left
        forward - strafeRight - turnCw,   // front-right
        forward - strafeRight + turnCw,   // back-left
        forward + strafeRight - turnCw,   // back-right
    };
    normalizeInPlace(outs);
    fl_->move_voltage(static_cast<int>(outs[0] * MAX_MV));
    fr_->move_voltage(static_cast<int>(outs[1] * MAX_MV));
    bl_->move_voltage(static_cast<int>(outs[2] * MAX_MV));
    br_->move_voltage(static_cast<int>(outs[3] * MAX_MV));
}

void HolonomicDrive::drive(double strafe, double forward, double turn,
                           double headingRad, bool fieldCentric) {
    double vx = forward;
    double vy = -strafe;  // stick-right = strafe right = -vy in our frame
    if (fieldCentric) {
        // Rotate the field-frame stick vector into the robot frame.
        const Point robotFrame = Point{vx, vy}.rotatedBy(-headingRad);
        vx = robotFrame.x;
        vy = robotFrame.y;
    }
    applyNormalized({vx, vy, -turn});
}

void HolonomicDrive::stop(bool hold) {
    const auto mode = hold ? pros::MotorBrake::hold : pros::MotorBrake::coast;
    for (auto& g : {fl_, fr_, bl_, br_}) {
        g->set_brake_mode_all(mode);
        g->brake();
    }
}

double HolonomicDrive::averageWheelSpeed() {
    auto avg = [](const std::vector<double>& v) {
        double s = 0;
        for (double x : v) s += std::fabs(x);
        return v.empty() ? 0 : s / v.size();
    };
    const double motorRpm = (avg(fl_->get_actual_velocity_all()) +
                             avg(fr_->get_actual_velocity_all()) +
                             avg(bl_->get_actual_velocity_all()) +
                             avg(br_->get_actual_velocity_all())) / 4.0;
    double cartridge = 200;
    switch (fl_->get_gearing()) {
        case pros::MotorGears::red:  cartridge = 100; break;
        case pros::MotorGears::green: cartridge = 200; break;
        case pros::MotorGears::blue: cartridge = 600; break;
        default: break;
    }
    const double wheelRpm = motorRpm * (cfg_.wheelRpm / cartridge);
    return wheelRpm / 60.0 * cfg_.wheelDiameter * units::PI;
}

} // namespace aklib
