#pragma once
/**
 * \file drivetrain.hpp
 * Drivetrain: the layer that turns a ChassisSpeeds request into motor
 * voltages. Two flavors:
 *
 *   TANK       left + right motor groups. Cannot strafe (vy is ignored).
 *   HOLONOMIC  four corner motor groups (X-drive or mecanum). Full vx/vy/omega.
 *
 * Both are built from a Config struct that reads like a spec sheet — fill in
 * your robot's numbers and you're done. All motion code upstream is identical
 * for both drive types.
 */

#include <memory>
#include <vector>

#include "pros/motor_group.hpp"

#include "aklib/control/feedforward.hpp"
#include "aklib/pose.hpp"
#include "aklib/units.hpp"

namespace aklib {

/** Physical constants shared by every drivetrain type. */
struct DriveConfig {
    /** Diameter of the DRIVEN wheels, inches (measure it — labels lie). */
    double wheelDiameter = 3.25;
    /** Distance between left and right wheel contact patches, inches.
     *  (For holonomic: distance between left and right wheel centers.) */
    double trackWidth = 12.0;
    /** Wheel RPM at full motor output, AFTER external gearing.
     *  blue 600 rpm cartridge with 36:48 gearing => 600*36/48 = 450. */
    double wheelRpm = 450;
    /** Optional feedforward constants (volts). Only needed for profiled
     *  motions; leave zeroed to skip. See docs "Characterizing feedforward". */
    FeedforwardGains feedforward{};

    /** Theoretical top speed in in/s — derived, you don't set this. */
    double maxSpeed() const {
        return wheelRpm / 60.0 * wheelDiameter * units::PI;
    }
};

/**
 * Base interface. Motions only ever see this — that's what makes tank and
 * holonomic interchangeable.
 */
class Drivetrain {
public:
    virtual ~Drivetrain() = default;

    /**
     * Execute a normalized command. Each component of `speeds` is in [-1, 1]
     * (fraction of full output); the drivetrain mixes them into per-wheel
     * voltages, scaling down proportionally if any wheel would exceed 100%.
     */
    virtual void applyNormalized(const ChassisSpeeds& speeds) = 0;

    /** Stop all motors with the given brake behavior. */
    virtual void stop(bool hold = false) = 0;

    /** Average |wheel speed| in in/s — used for stall detection. */
    virtual double averageWheelSpeed() = 0;

    /** True if this drivetrain can produce sideways (vy) motion. */
    virtual bool holonomic() const = 0;

    const DriveConfig& config() const { return cfg_; }

protected:
    explicit Drivetrain(const DriveConfig& cfg) : cfg_(cfg) {}
    DriveConfig cfg_;
};

// ---------------------------------------------------------------------------
// Tank drive (a.k.a. differential / skid-steer). The default in VEX.
// ---------------------------------------------------------------------------
class TankDrive : public Drivetrain {
public:
    /**
     * \param left   left-side motors  (negate ports in the group to reverse)
     * \param right  right-side motors
     * \param cfg    physical constants (see DriveConfig)
     *
     * Example:
     *   auto left  = std::make_shared<pros::MotorGroup>(std::initializer_list<int8_t>{-1, -2, -3});
     *   auto right = std::make_shared<pros::MotorGroup>(std::initializer_list<int8_t>{4, 5, 6});
     *   aklib::TankDrive drive(left, right, {.wheelDiameter = 3.25,
     *                                        .trackWidth = 12.5,
     *                                        .wheelRpm = 450});
     */
    TankDrive(std::shared_ptr<pros::MotorGroup> left,
              std::shared_ptr<pros::MotorGroup> right,
              const DriveConfig& cfg);

    void applyNormalized(const ChassisSpeeds& speeds) override;
    void stop(bool hold = false) override;
    double averageWheelSpeed() override;
    bool holonomic() const override { return false; }

    /** Direct tank access for driver control: outputs in [-1, 1]. */
    void tank(double leftOut, double rightOut);
    /** Arcade driver control: throttle + turn in [-1, 1]. */
    void arcade(double throttle, double turn);

    std::shared_ptr<pros::MotorGroup> leftGroup() { return left_; }
    std::shared_ptr<pros::MotorGroup> rightGroup() { return right_; }

private:
    std::shared_ptr<pros::MotorGroup> left_, right_;
};

// ---------------------------------------------------------------------------
// Holonomic drive: X-drive or mecanum. Motors are the four corners.
// ---------------------------------------------------------------------------
class HolonomicDrive : public Drivetrain {
public:
    enum class Kind {
        XDrive,   ///< omni wheels at 45 degrees
        Mecanum,  ///< mecanum wheels, rollers forming an X seen from above
    };

    /**
     * \param frontLeft/.../backRight  one MotorGroup per corner (a group so
     *        double-motor corners work too; negate ports to reverse — each
     *        wheel should spin "forward" when its group is commanded +).
     *
     * Example:
     *   aklib::HolonomicDrive drive(fl, fr, bl, br,
     *       aklib::HolonomicDrive::Kind::Mecanum,
     *       {.wheelDiameter = 4.0, .trackWidth = 13.0, .wheelRpm = 200});
     */
    HolonomicDrive(std::shared_ptr<pros::MotorGroup> frontLeft,
                   std::shared_ptr<pros::MotorGroup> frontRight,
                   std::shared_ptr<pros::MotorGroup> backLeft,
                   std::shared_ptr<pros::MotorGroup> backRight,
                   Kind kind, const DriveConfig& cfg);

    void applyNormalized(const ChassisSpeeds& speeds) override;
    void stop(bool hold = false) override;
    double averageWheelSpeed() override;
    bool holonomic() const override { return true; }

    /**
     * Driver control: strafe/forward/turn in [-1, 1], robot-centric.
     * For field-centric, pass the current heading (radians) so the stick
     * vector is rotated into the robot frame first.
     */
    void drive(double strafe, double forward, double turn,
               double headingRad = 0, bool fieldCentric = false);

private:
    std::shared_ptr<pros::MotorGroup> fl_, fr_, bl_, br_;
    Kind kind_;
};

} // namespace aklib
