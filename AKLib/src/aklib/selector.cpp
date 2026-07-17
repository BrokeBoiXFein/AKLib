/**
 * \file selector.cpp
 * LVGL 9 implementation of the auton selector. Layout follows robodash's
 * selector view (header bar + tappable routine cards, selection highlighted
 * and saved to the SD card); colors follow the Affogato theme.
 */

#include "aklib/selector.hpp"

#include <cstdio>

namespace aklib {

namespace {

// Affogato palette pieces used by the widget
constexpr std::uint32_t COL_HEADER = 0x2B1A10;  // espresso
constexpr std::uint32_t COL_CARD   = 0x3A2617;  // roast
constexpr std::uint32_t COL_BORDER = 0x4A2E1C;  // coffee
constexpr std::uint32_t COL_TEXT   = 0xF4E9D7;  // cream
constexpr std::uint32_t COL_MUTED  = 0xC9A47A;  // latte
constexpr std::uint32_t COL_ON_ACCENT = 0x2B1A10;

const std::string kNoneName;

} // namespace

Selector::Selector(std::vector<Routine> routines, SelectorOptions opts)
    : routines_(std::move(routines)), opts_(opts) {
    build_();
    restore_();
    applyStyles_();
}

Selector::~Selector() {
    if (root_) lv_obj_delete(root_);
}

const std::string& Selector::selectedName() const {
    if (selected_ < 0 || selected_ >= (int)routines_.size()) return kNoneName;
    return routines_[selected_].name;
}

void Selector::run() {
    if (selected_ >= 0 && selected_ < (int)routines_.size() &&
        routines_[selected_].run) {
        routines_[selected_].run();
    }
}

void Selector::select(int index) {
    if (index < 0 || index >= (int)routines_.size()) return;
    selected_ = index;
    applyStyles_();
    persist_();
}

void Selector::setVisible(bool visible) {
    if (!root_) return;
    if (visible) lv_obj_remove_flag(root_, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void Selector::build_() {
    // Full-screen root panel.
    root_ = lv_obj_create(lv_screen_active());
    lv_obj_set_size(root_, lv_pct(100), lv_pct(100));
    lv_obj_set_pos(root_, 0, 0);
    lv_obj_set_style_bg_color(root_, lv_color_hex(opts_.background), 0);
    lv_obj_set_style_border_width(root_, 0, 0);
    lv_obj_set_style_radius(root_, 0, 0);
    lv_obj_set_style_pad_all(root_, 0, 0);
    lv_obj_remove_flag(root_, LV_OBJ_FLAG_SCROLLABLE);

    // Header bar: title left, current selection right.
    lv_obj_t* header = lv_obj_create(root_);
    lv_obj_set_size(header, lv_pct(100), 38);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(COL_HEADER), 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_set_style_pad_hor(header, 12, 0);
    lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* title = lv_label_create(header);
    lv_label_set_text(title, opts_.title);
    lv_obj_set_style_text_color(title, lv_color_hex(opts_.accent), 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);

    statusLabel_ = lv_label_create(header);
    lv_label_set_text(statusLabel_, "no auton selected");
    lv_obj_set_style_text_color(statusLabel_, lv_color_hex(COL_MUTED), 0);
    lv_obj_align(statusLabel_, LV_ALIGN_RIGHT_MID, 0, 0);

    // Card grid: wraps into rows, scrolls vertically if you have many autons.
    grid_ = lv_obj_create(root_);
    lv_obj_set_size(grid_, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_pos(grid_, 0, 38);
    lv_obj_set_style_bg_opa(grid_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid_, 0, 0);
    lv_obj_set_style_pad_all(grid_, 10, 0);
    lv_obj_set_style_pad_row(grid_, 8, 0);
    lv_obj_set_style_pad_column(grid_, 8, 0);
    lv_obj_set_flex_flow(grid_, LV_FLEX_FLOW_ROW_WRAP);

    for (std::size_t i = 0; i < routines_.size(); i++) {
        lv_obj_t* card = lv_button_create(grid_);
        lv_obj_set_size(card, 222, 44);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_shadow_width(card, 0, 0);
        // Left-edge color tag (robodash-style routine color).
        if (routines_[i].tag != 0) {
            lv_obj_set_style_border_side(card, LV_BORDER_SIDE_LEFT, 0);
            lv_obj_set_style_border_width(card, 4, 0);
            lv_obj_set_style_border_color(card, lv_color_hex(routines_[i].tag), 0);
        } else {
            lv_obj_set_style_border_side(card, LV_BORDER_SIDE_FULL, 0);
            lv_obj_set_style_border_width(card, 1, 0);
            lv_obj_set_style_border_color(card, lv_color_hex(COL_BORDER), 0);
        }
        lv_obj_add_event_cb(card, cardEvent_, LV_EVENT_CLICKED, this);

        lv_obj_t* label = lv_label_create(card);
        lv_label_set_text(label, routines_[i].name.c_str());
        lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
        lv_obj_set_width(label, lv_pct(100));
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_center(label);

        cards_.push_back(card);
    }
}

void Selector::applyStyles_() {
    for (std::size_t i = 0; i < cards_.size(); i++) {
        lv_obj_t* card = cards_[i];
        lv_obj_t* label = lv_obj_get_child(card, 0);
        const bool sel = (int)i == selected_;
        lv_obj_set_style_bg_color(
            card, lv_color_hex(sel ? opts_.accent : COL_CARD), 0);
        lv_obj_set_style_text_color(
            label, lv_color_hex(sel ? COL_ON_ACCENT : COL_TEXT), 0);
    }
    if (statusLabel_) {
        if (selected_ >= 0) {
            lv_label_set_text_fmt(statusLabel_, "%s  %s", LV_SYMBOL_OK,
                                  routines_[selected_].name.c_str());
            lv_obj_set_style_text_color(statusLabel_, lv_color_hex(COL_TEXT), 0);
        } else {
            lv_label_set_text(statusLabel_, "no auton selected");
            lv_obj_set_style_text_color(statusLabel_, lv_color_hex(COL_MUTED), 0);
        }
    }
}

void Selector::cardEvent_(lv_event_t* e) {
    auto* self = static_cast<Selector*>(lv_event_get_user_data(e));
    // LVGL 9: get_target returns void*
    auto* card = static_cast<lv_obj_t*>(lv_event_get_target(e));
    // Cards are the grid's children in creation order, so the child index IS
    // the routine index.
    self->select((int)lv_obj_get_index(card));
}

// ---------------------------------------------------------------------------
// SD-card persistence (best effort: silently skipped without a card)
// ---------------------------------------------------------------------------

void Selector::persist_() const {
    if (!opts_.savePath || selected_ < 0) return;
    if (FILE* f = std::fopen(opts_.savePath, "w")) {
        std::fputs(routines_[selected_].name.c_str(), f);
        std::fclose(f);
    }
}

void Selector::restore_() {
    if (!opts_.savePath) return;
    FILE* f = std::fopen(opts_.savePath, "r");
    if (!f) return;
    char buf[128] = {};
    std::fgets(buf, sizeof buf, f);
    std::fclose(f);
    const std::string saved(buf);
    for (std::size_t i = 0; i < routines_.size(); i++) {
        if (routines_[i].name == saved) { selected_ = (int)i; return; }
    }
}

} // namespace aklib
