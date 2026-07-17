#pragma once
/**
 * \file units.hpp
 * AKLib unit conventions and conversion helpers.
 *
 * ============================================================================
 *  THE ONE RULE: every plain number AKLib takes or returns is IMPERIAL.
 * ============================================================================
 *    LENGTH .... inches
 *    ANGLE ..... degrees, math-standard (0 deg = +x "east", counterclockwise
 *                is positive). If you prefer compass headings (0 = up/north,
 *                clockwise positive), wrap your number in aklib::compass().
 *    TIME ...... milliseconds for timeouts, seconds inside physics math
 *    SPEED ..... inches per second (in/s) unless a parameter says "percent"
 *
 *  Metric users: you never have to convert by hand. Use the literals below —
 *  they turn any metric quantity into the inches/degrees AKLib expects:
 *
 *      chassis.driveDistance(60_cm);      // == driveDistance(23.62)
 *      chassis.moveToPoint(1.2_m, 0.6_m); // meters -> inches automatically
 *      chassis.turnToHeading(1.57_rad);   // radians -> degrees automatically
 *
 *  The literals are plain constexpr doubles — zero runtime cost.
 */

namespace aklib {

namespace units {

// ---- canonical conversion factors -----------------------------------------
inline constexpr double IN_PER_FT = 12.0;
inline constexpr double IN_PER_M  = 39.37007874015748;
inline constexpr double IN_PER_CM = IN_PER_M / 100.0;
inline constexpr double IN_PER_MM = IN_PER_M / 1000.0;
inline constexpr double PI        = 3.14159265358979323846;
inline constexpr double DEG_PER_RAD = 180.0 / PI;

// ---- to-internal (call these if you don't like literals) -------------------
constexpr double fromFeet(double ft)     { return ft * IN_PER_FT; }
constexpr double fromMeters(double m)    { return m * IN_PER_M; }
constexpr double fromCm(double cm)       { return cm * IN_PER_CM; }
constexpr double fromMm(double mm)       { return mm * IN_PER_MM; }
constexpr double fromRadians(double rad) { return rad * DEG_PER_RAD; }

// ---- from-internal (for printing/telemetry in metric) ----------------------
constexpr double toFeet(double in)     { return in / IN_PER_FT; }
constexpr double toMeters(double in)   { return in / IN_PER_M; }
constexpr double toCm(double in)       { return in / IN_PER_CM; }
constexpr double toMm(double in)       { return in / IN_PER_MM; }
constexpr double toRadians(double deg) { return deg / DEG_PER_RAD; }

} // namespace units

/**
 * Convert a compass heading (0 deg = "up"/north/+y, clockwise positive — the
 * LemLib convention) into AKLib's math-standard heading (0 deg = +x, CCW
 * positive). Use it anywhere AKLib expects an angle:
 *
 *     chassis.turnToHeading(compass(90));   // face "east" in compass terms
 */
constexpr double compass(double compassDeg) { return 90.0 - compassDeg; }

/**
 * User-defined literals. `using namespace aklib;` (or aklib::literals) makes
 * these available. Both `24_in` (integer) and `23.5_in` (floating) work.
 */
inline namespace literals {

// -- lengths: everything converts to inches ---------------------------------
constexpr double operator""_in(long double v)        { return double(v); }
constexpr double operator""_in(unsigned long long v) { return double(v); }
constexpr double operator""_ft(long double v)        { return units::fromFeet(double(v)); }
constexpr double operator""_ft(unsigned long long v) { return units::fromFeet(double(v)); }
constexpr double operator""_m(long double v)         { return units::fromMeters(double(v)); }
constexpr double operator""_m(unsigned long long v)  { return units::fromMeters(double(v)); }
constexpr double operator""_cm(long double v)        { return units::fromCm(double(v)); }
constexpr double operator""_cm(unsigned long long v) { return units::fromCm(double(v)); }
constexpr double operator""_mm(long double v)        { return units::fromMm(double(v)); }
constexpr double operator""_mm(unsigned long long v) { return units::fromMm(double(v)); }

// -- angles: everything converts to degrees ----------------------------------
constexpr double operator""_deg(long double v)        { return double(v); }
constexpr double operator""_deg(unsigned long long v) { return double(v); }
constexpr double operator""_rad(long double v)        { return units::fromRadians(double(v)); }
constexpr double operator""_rad(unsigned long long v) { return units::fromRadians(double(v)); }

// -- one tile = 24 inches: write auton distances the way you think ----------
constexpr double operator""_tiles(long double v)        { return double(v) * 24.0; }
constexpr double operator""_tiles(unsigned long long v) { return double(v) * 24.0; }

} // namespace literals

} // namespace aklib
