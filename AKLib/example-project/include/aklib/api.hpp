#pragma once
/**
 * \file api.hpp
 * One include to get all of AKLib:
 *
 *     #include "aklib/api.hpp"
 *     using namespace aklib::literals;   // enables 24_in, 90_deg, 60_cm, ...
 */

#include "aklib/units.hpp"
#include "aklib/pose.hpp"

#include "aklib/control/pid.hpp"
#include "aklib/control/exit.hpp"
#include "aklib/control/slew.hpp"
#include "aklib/control/profile.hpp"
#include "aklib/control/feedforward.hpp"

#include "aklib/sensors/tracking_wheel.hpp"
#include "aklib/odometry.hpp"
#include "aklib/drivetrain.hpp"
#include "aklib/chassis.hpp"
#include "aklib/path.hpp"
#include "aklib/autotune.hpp"
#include "aklib/selector.hpp"
