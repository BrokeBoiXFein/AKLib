#pragma once
/**
 * \file selector.hpp
 * Autonomous selector on the brain screen — EZ-Template's job, robodash's
 * look: a title bar, a scrollable grid of tappable routine cards (each with an
 * optional color tag on its left edge), the current pick highlighted and
 * persisted to the SD card so it survives power cycles.
 *
 * ============================================================================
 *  USAGE (full tutorial: docs > Tutorials > Auton selector)
 * ============================================================================
 *
 *   #include "aklib/api.hpp"
 *
 *   std::unique_ptr<aklib::Selector> selector;
 *
 *   void initialize() {
 *       chassis.calibrate();
 *       selector = std::make_unique<aklib::Selector>(std::vector<aklib::Selector::Routine>{
 *           {"Left AWP",   leftAwp,   0xC8485A},   // name, function, tag color
 *           {"Right rush", rightRush, 0x5894C8},
 *           {"Skills",     skills,    0xECB93C},
 *           {"Do nothing", [] {}},
 *       });
 *   }
 *
 *   void autonomous() { if (selector) selector->run(); }
 *
 * Build it in initialize() (after calibrate()) — the display is guaranteed
 * ready there. Tap a card to select; with an SD card inserted the selection
 * is restored automatically on the next boot.
 *
 * Requirements: none beyond PROS (LVGL ships with the kernel). SD card
 * optional (persistence silently skips without one).
 */

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "liblvgl/lvgl.h"

namespace aklib {

/** Cosmetic knobs. Defaults are the Affogato theme on a robodash layout. */
struct SelectorOptions {
    const char* title = "AKLib  \xC2\xB7  Autonomous";  ///< header text (UTF-8)
    std::uint32_t accent = 0xB0703C;    ///< highlight color (caramel)
    std::uint32_t background = 0x26170D;///< screen background (espresso)
    const char* savePath = "/usd/aklib_auton.txt";  ///< SD persistence file
};

class Selector {
public:
    struct Routine {
        std::string name;              ///< shown on the card (keep it short)
        std::function<void()> run;     ///< your auton function
        std::uint32_t tag = 0;         ///< optional left-edge tag color
                                       ///< (0 = no tag). e.g. 0xC8485A
    };

    /** Builds the UI immediately — call from initialize(). */
    explicit Selector(std::vector<Routine> routines, SelectorOptions opts = {});
    ~Selector();

    Selector(const Selector&) = delete;
    Selector& operator=(const Selector&) = delete;

    /** Execute the selected routine (call this in autonomous()).
     *  Does nothing if no routine is selected. */
    void run();

    /** Currently selected index, or -1. */
    int selectedIndex() const { return selected_; }

    /** Currently selected name, or "" when nothing is selected. */
    const std::string& selectedName() const;

    /** Select programmatically (same effect as tapping card i). */
    void select(int index);

    /** Show / hide the whole selector (e.g. to swap to a debug screen). */
    void setVisible(bool visible);

private:
    void build_();
    void applyStyles_();          ///< restyle every card after a selection
    void persist_() const;        ///< write selection to SD (best effort)
    void restore_();              ///< read selection from SD (best effort)
    static void cardEvent_(lv_event_t* e);

    std::vector<Routine> routines_;
    SelectorOptions opts_;
    int selected_ = -1;

    lv_obj_t* root_ = nullptr;
    lv_obj_t* grid_ = nullptr;
    lv_obj_t* statusLabel_ = nullptr;
    std::vector<lv_obj_t*> cards_;
};

} // namespace aklib
