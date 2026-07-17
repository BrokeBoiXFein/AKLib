/**
 * \file selector_example.cpp
 * Wiring the auton selector into a competition program. This file shows the
 * pieces; paste them into YOUR main.cpp (this file deliberately does not
 * define initialize()/autonomous() so it can sit next to a stock main.cpp).
 */

#include <memory>

#include "aklib/api.hpp"

extern aklib::Chassis chassis;

// ── your routines ───────────────────────────────────────────────────────────
static void leftAwp() {
    chassis.setPose(36, 12, 90);
    chassis.driveDistance(24);
    // ...
}
static void rightRush() {
    chassis.setPose(108, 12, 90);
    chassis.moveToPoint(96, 48, {.minSpeed = 0.4, .earlyExitRange = 6});
    // ...
}
static void skills() {
    chassis.setPose(12, 36, 0);
    // ...
}

// ── the selector ────────────────────────────────────────────────────────────
// Global holder; constructed in initialize() when the display is ready.
std::unique_ptr<aklib::Selector> selector;

void buildSelector() {
    selector = std::make_unique<aklib::Selector>(
        std::vector<aklib::Selector::Routine>{
            // {card text, function, left-edge tag color (optional)}
            {"Left AWP", leftAwp, 0xC8485A},     // EDP identify-red
            {"Right rush", rightRush, 0x5894C8}, // EDP program-blue
            {"Skills", skills, 0xECB93C},        // EDP plan-yellow
            {"Do nothing", [] {}},
        });
}

/* In your main.cpp:

void initialize() {
    chassis.calibrate();
    buildSelector();          // build AFTER calibrate: display is ready and
}                             // the IMU wait doesn't hide the UI

void autonomous() {
    if (selector) selector->run();
}

Tap a card to pick a routine (highlight + checkmark in the header). With an
SD card in the brain, the pick is remembered across power cycles — queue for
a match, power on, and the right auton is already selected. */
