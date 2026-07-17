/**
 * \file main.cpp — AKLib template main. EVERYTHING IS ALREADY WIRED:
 *
 *   1. initialize()  calibrates the chassis, then puts the auton selector on
 *                    the brain screen, preloaded with the example routines
 *                    from autons.cpp. Tap a card to pick one (saved to SD).
 *   2. autonomous()  runs whatever the selector has picked.
 *   3. opcontrol()   arcade driver control, auto-detecting tank vs holonomic.
 *
 * Build, download, and it works. Your whole workflow after that:
 *   - robot_config.cpp  -> put in YOUR ports/dimensions/gains (the sheet)
 *   - autons.cpp        -> write your real routines (+ declare in autons.hpp)
 *   - the list below    -> swap the example cards for your routines
 *
 * NOTE: don't add pros::lcd (LLEMU) calls — the legacy LCD draws over the
 * same screen the selector uses.
 */

#include "main.h"

#include <memory>

#include "aklib/api.hpp"
#include "autons.hpp"

extern aklib::Chassis chassis;   // defined in robot_config.cpp

// Built in initialize() (after calibrate, when the display is ready).
std::unique_ptr<aklib::Selector> selector;

void initialize() {
    // Robot must sit still for ~3 s while the IMU calibrates.
    chassis.calibrate();

    selector = std::make_unique<aklib::Selector>(
        std::vector<aklib::Selector::Routine>{
            // {card label, function, left-edge tag color (optional)}
            {"Simple drive", simpleAuton, 0x5BA567},
            {"Point-based",  pointAuton,  0x5894C8},
            {"Chained",      chainedAuton, 0xE08A45},
            {"Async demo",   asyncAuton,  0xD06A99},
            {"Pure pursuit", pathAuton,   0xC8485A},
            {"Do nothing",   [] {}},
        });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    if (selector) selector->run();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    // Drive whichever drivetrain robot_config.cpp built — no edits needed
    // when you switch between tank and holonomic.
    auto tank = std::dynamic_pointer_cast<aklib::TankDrive>(chassis.drivetrain());
    auto holo = std::dynamic_pointer_cast<aklib::HolonomicDrive>(chassis.drivetrain());

    while (true) {
        const double fwd  = master.get_analog(ANALOG_LEFT_Y) / 127.0;
        const double turn = master.get_analog(ANALOG_RIGHT_X) / 127.0;

        if (tank) {
            tank->arcade(fwd, turn);
        } else if (holo) {
            const double strafe = master.get_analog(ANALOG_LEFT_X) / 127.0;
            holo->drive(strafe, fwd, turn);
            // field-centric instead? ->
            // holo->drive(strafe, fwd, turn, chassis.pose().theta, true);
        }

        pros::delay(20);
    }
}
