#pragma once
/**
 * \file sensors/tracking_wheel.hpp
 * TrackingWheel: one odometry input — an unpowered wheel (or the drive
 * wheels themselves) whose rotation we convert to inches traveled.
 *
 * ============================================================================
 *  WHICH CONSTRUCTOR DO I USE?  (pick one per wheel)
 * ============================================================================
 *  1) V5 Rotation sensor on a dedicated tracking wheel   <- BEST accuracy
 *         TrackingWheel(rotationPort, wheelDiameter, offset)
 *  2) ADI (3-wire) optical shaft encoder on a tracking wheel
 *         TrackingWheel(adiEncoder, wheelDiameter, offset)
 *  3) The drive motors themselves (no extra hardware; slips under load)
 *         TrackingWheel(motorGroup, wheelDiameter, offset, driveRpm)
 *
 * ============================================================================
 *  OFFSET — the number everyone gets wrong (sign matters!)
 * ============================================================================
 *  Offset = signed distance (inches) from the robot's TRACKING CENTER to the
 *  wheel, measured PERPENDICULAR to the wheel's rolling direction.
 *
 *    VERTICAL wheel (rolls forward/backward):
 *        offset < 0 if the wheel is LEFT of center, > 0 if RIGHT.
 *    HORIZONTAL wheel (rolls left/right):
 *        offset < 0 if BEHIND center, > 0 if IN FRONT.
 *
 *  Sanity check after setup: spin the robot in place. The pose (x, y) should
 *  barely move. If it orbits, an offset has the wrong sign or magnitude.
 */

#include <memory>
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include "aklib/units.hpp"

namespace aklib {

class TrackingWheel {
public:
    /** 1) V5 Rotation sensor. Negate the port (e.g. -14) to reverse direction. */
    TrackingWheel(int rotationPort, double wheelDiameter, double offset)
        : rotation_(std::make_shared<pros::Rotation>(rotationPort)),
          diameter_(wheelDiameter), offset_(offset) {}

    /** 2) ADI quadrature encoder (pass your own constructed object). */
    TrackingWheel(std::shared_ptr<pros::adi::Encoder> encoder,
                  double wheelDiameter, double offset)
        : adi_(std::move(encoder)), diameter_(wheelDiameter), offset_(offset) {}

    /**
     * 3) Drive-motor "wheel" (integrated encoders).
     * \param motors        the motor group on ONE side of the drive
     * \param wheelDiameter diameter of the DRIVEN wheels, inches
     * \param offset        half the track width (negative for the left side)
     * \param wheelRpm      wheel RPM at full motor speed AFTER external
     *                      gearing (e.g. blue cartridge 600 RPM with a 36:60
     *                      gear-down = 360). Used to convert motor degrees to
     *                      wheel degrees.
     */
    TrackingWheel(std::shared_ptr<pros::MotorGroup> motors, double wheelDiameter,
                  double offset, double wheelRpm)
        : motors_(std::move(motors)), diameter_(wheelDiameter), offset_(offset),
          wheelRpm_(wheelRpm) {}

    /** Inches this wheel has rolled since program start (signed). */
    double distance() {
        const double circumference = diameter_ * units::PI;
        if (rotation_) {
            // Rotation sensor reports centidegrees of absolute position.
            return rotation_->get_position() / 36000.0 * circumference;
        }
        if (adi_) {
            // ADI encoder reports degrees.
            return adi_->get_value() / 360.0 * circumference;
        }
        if (motors_) {
            // Average all motor positions (degrees), scale motor->wheel by
            // the ratio of wheel rpm to the motor cartridge speed.
            const auto positions = motors_->get_position_all();
            if (positions.empty()) return 0;
            double sum = 0;
            for (double p : positions) sum += p;
            const double motorDeg = sum / positions.size();
            const double cartridgeRpm = cartridgeRpm_();
            return motorDeg * (wheelRpm_ / cartridgeRpm) / 360.0 * circumference;
        }
        return 0;
    }

    /** Zero the sensor. Called by Odometry::calibrate(). */
    void reset() {
        if (rotation_) rotation_->reset_position();
        if (adi_) adi_->reset();
        if (motors_) motors_->tare_position_all();
    }

    double offset() const { return offset_; }

    /** True if this "wheel" is really the drive motors (slip-prone input). */
    bool isMotorBased() const { return motors_ != nullptr; }

private:
    double cartridgeRpm_() const {
        switch (motors_->get_gearing()) {
            case pros::MotorGears::red:  return 100;
            case pros::MotorGears::green: return 200;
            case pros::MotorGears::blue: return 600;
            default: return 200;
        }
    }

    std::shared_ptr<pros::Rotation> rotation_;
    std::shared_ptr<pros::adi::Encoder> adi_;
    std::shared_ptr<pros::MotorGroup> motors_;
    double diameter_;
    double offset_;
    double wheelRpm_ = 200;
};

} // namespace aklib
