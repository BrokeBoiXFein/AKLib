/**
 * \file odometry.cpp
 * Arc-based odometry (Pilons 5225A method) with flexible sensor combos.
 * See odometry.hpp for the sensor-combination table and usage.
 */

#include "aklib/odometry.hpp"

#include <cmath>
#include <mutex>

#include "pros/rtos.hpp"

namespace aklib {

namespace {
constexpr int LOOP_MS = 10;   // tracking period; fixed-rate via delay_until
}

Odometry::Odometry(OdomSensors sensors) : s_(std::move(sensors)) {
    if (s_.imuPort != 0) imu_ = std::make_unique<pros::Imu>(s_.imuPort);
    if (s_.imuPort2 != 0) imu2_ = std::make_unique<pros::Imu>(s_.imuPort2);
}

Odometry::~Odometry() {
    if (task_) task_->remove();
}

bool Odometry::calibrate() {
    bool imuOk = true;

    // ---- IMU calibration: robot must be stationary. --------------------
    if (imu_) {
        imu_->reset(false);
        if (imu2_) imu2_->reset(false);
        // Wait for calibration with a hard 3.5 s cap so a dead IMU can't
        // hang initialize() forever.
        const uint32_t start = pros::millis();
        auto calibrating = [](pros::Imu& i) {
            return i.is_calibrating();
        };
        pros::delay(100);  // let calibration actually begin
        while ((calibrating(*imu_) || (imu2_ && calibrating(*imu2_)))) {
            if (pros::millis() - start > 3500) { imuOk = false; break; }
            pros::delay(20);
        }
        // A reading of exactly infinity/NaN also means a failed IMU.
        if (imuOk && !std::isfinite(imu_->get_rotation())) imuOk = false;
    }

    // ---- Zero every wheel input we were given. -------------------------
    if (s_.vertical1) s_.vertical1->reset();
    if (s_.vertical2) s_.vertical2->reset();
    if (s_.horizontal1) s_.horizontal1->reset();
    if (s_.driveLeft) s_.driveLeft->reset();
    if (s_.driveRight) s_.driveRight->reset();

    lastVert1_ = lastVert2_ = lastHorz_ = 0;
    lastDriveL_ = lastDriveR_ = 0;
    lastHeading_ = readHeadingRad_();

    // ---- Start (or restart) the background tracking task. --------------
    if (!task_) {
        task_ = std::make_unique<pros::Task>([this] { trackingLoop_(); },
                                             "aklib-odometry");
    }
    ready_ = true;
    return imuOk;
}

Pose Odometry::pose() const {
    const std::lock_guard<pros::Mutex> lock(mutex_);
    return pose_;
}

void Odometry::setPose(double x, double y, double thetaDeg) {
    setPose(Pose::fromDeg(x, y, thetaDeg));
}

void Odometry::setPose(const Pose& p) {
    const std::lock_guard<pros::Mutex> lock(mutex_);
    // Heading is tracked incrementally (deltas of the raw sensor), so
    // re-anchoring lastHeading_ to the current raw reading makes the new pose
    // the reference point from here on.
    pose_ = p;
    lastHeading_ = readHeadingRad_();
}

double Odometry::speed() const {
    const std::lock_guard<pros::Mutex> lock(mutex_);
    return speed_;
}

double Odometry::angularSpeedDeg() const {
    const std::lock_guard<pros::Mutex> lock(mutex_);
    return angSpeedDeg_;
}

double Odometry::readHeadingRad_() {
    // Priority 1: IMU(s). get_rotation() is unbounded (doesn't wrap at 360),
    // which is exactly what we want for delta math. Negated because the V5
    // IMU reports clockwise-positive but our convention is CCW-positive.
    if (imu_) {
        double deg = imu_->get_rotation();
        if (imu2_) deg = (deg + imu2_->get_rotation()) / 2.0;
        if (std::isfinite(deg)) return degToRad(-deg * s_.imuScale);
    }
    // Priority 2: two vertical wheels (Pilons-style encoder heading).
    if (s_.vertical1 && s_.vertical2) {
        const double l = s_.vertical1->distance();
        const double r = s_.vertical2->distance();
        const double width = std::fabs(s_.vertical1->offset()) +
                             std::fabs(s_.vertical2->offset());
        if (width > 0) return (l - r) / width;
    }
    // Priority 3: drive motor differential (last resort).
    if (s_.driveLeft && s_.driveRight) {
        const double l = s_.driveLeft->distance();
        const double r = s_.driveRight->distance();
        const double width = std::fabs(s_.driveLeft->offset()) +
                             std::fabs(s_.driveRight->offset());
        if (width > 0) return (l - r) / width;
    }
    return 0;
}

void Odometry::trackingLoop_() {
    uint32_t now = pros::millis();

    while (true) {
        // ---- 1. Read deltas from whichever sensors exist. ----------------
        // Forward distance: prefer a real vertical tracking wheel; fall back
        // to the average of the drive sides.
        double dForward = 0;
        double forwardOffset = 0;  // horizontal offset of the forward source
        if (s_.vertical1) {
            const double v1 = s_.vertical1->distance();
            double d = v1 - lastVert1_;
            lastVert1_ = v1;
            forwardOffset = s_.vertical1->offset();
            if (s_.vertical2) {
                const double v2 = s_.vertical2->distance();
                d = (d + (v2 - lastVert2_)) / 2.0;
                lastVert2_ = v2;
                forwardOffset = 0;  // symmetric pair tracks the center
            }
            dForward = d;
        } else if (s_.driveLeft && s_.driveRight) {
            const double l = s_.driveLeft->distance();
            const double r = s_.driveRight->distance();
            dForward = ((l - lastDriveL_) + (r - lastDriveR_)) / 2.0;
            lastDriveL_ = l;
            lastDriveR_ = r;
            forwardOffset = 0;  // averaged pair tracks the center
        }

        // Sideways distance: horizontal wheel or assumed zero.
        double dSide = 0;
        double sideOffset = 0;
        if (s_.horizontal1) {
            const double h = s_.horizontal1->distance();
            dSide = h - lastHorz_;
            lastHorz_ = h;
            sideOffset = s_.horizontal1->offset();
        }

        // Heading delta from the best available source.
        const double heading = readHeadingRad_();
        const double dTheta = heading - lastHeading_;
        lastHeading_ = heading;

        // ---- 2. Arc-model local displacement (Pilons formulas). ----------
        // If we turned, the wheel traces an arc; the chord of that arc is the
        // actual displacement. Offsets remove the component of wheel travel
        // caused purely by rotation.
        double localY;  // robot-forward displacement
        double localX;  // robot-rightward displacement
        if (std::fabs(dTheta) < 1e-9) {
            localY = dForward;
            localX = dSide;
        } else {
            const double chord = 2.0 * std::sin(dTheta / 2.0);
            localY = chord * (dForward / dTheta + forwardOffset);
            localX = chord * (dSide / dTheta + sideOffset);
        }

        // ---- 3. Rotate into the field frame at the average heading. ------
        {
            const std::lock_guard<pros::Mutex> lock(mutex_);
            const double avg = pose_.theta + dTheta / 2.0;
            const double c = std::cos(avg), s = std::sin(avg);
            // localY is along robot-forward (heading direction), localX is
            // along robot-right (heading - 90 deg).
            pose_.x += localY * c + localX * s;
            pose_.y += localY * s - localX * c;
            pose_.theta += dTheta;

            // Low-pass filtered speeds for stall detection & telemetry.
            const double dt = LOOP_MS / 1000.0;
            const double instSpeed = std::hypot(localX, localY) / dt;
            const double instAng = radToDeg(std::fabs(dTheta)) / dt;
            constexpr double a = 0.3;  // smoothing factor
            speed_ = a * instSpeed + (1 - a) * speed_;
            angSpeedDeg_ = a * instAng + (1 - a) * angSpeedDeg_;
        }

        // ---- 4. Sleep to the next fixed-rate tick (no timing drift). ------
        pros::Task::delay_until(&now, LOOP_MS);
    }
}

} // namespace aklib
