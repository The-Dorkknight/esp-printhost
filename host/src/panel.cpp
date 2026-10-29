// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/panel.h"

#include <cstdio>

namespace ph {

namespace {
struct ActionInfo {
    PanelAction a;
    const char *id, *label, *flash;
};
const ActionInfo ACTIONS[kPanelActions] = {
    {PanelAction::None, "none", "Nothing", ""},
    {PanelAction::Preheat, "preheat", "Preheat", "PREHEAT"},
    {PanelAction::Load, "load", "Load filament", "LOAD"},
    {PanelAction::Unload, "unload", "Unload filament", "UNLOAD"},
    {PanelAction::EStop, "estop", "Emergency stop (hold 1 s)", "E-STOP"},
    {PanelAction::Cooldown, "cooldown", "Cool down", "COOL DOWN"},
    {PanelAction::PauseResume, "pause", "Pause / resume print", "PAUSE"},
    {PanelAction::Home, "home", "Home all (G28, hold 1 s)", "HOME"},
    {PanelAction::Custom, "custom", "Custom command", "COMMAND"},
};
const ActionInfo& info(PanelAction a) {
    int i = int(a);
    return ACTIONS[i >= 0 && i < kPanelActions ? i : 0];
}
}  // namespace

const char* panel_action_id(PanelAction a) { return info(a).id; }
const char* panel_action_label(PanelAction a) { return info(a).label; }
const char* panel_flash(PanelAction a) { return info(a).flash; }

PanelAction panel_action_from(const std::string& id) {
    for (auto& x : ACTIONS)
        if (id == x.id) return x.a;
    return PanelAction::None;
}

PanelAction panel_default_action(int button) {
    static const PanelAction D[kPanelButtons] = {PanelAction::Preheat, PanelAction::Home, PanelAction::Unload,
                                                 PanelAction::EStop};
    return button >= 0 && button < kPanelButtons ? D[button] : PanelAction::None;
}

bool panel_hold_action(PanelAction a) { return a == PanelAction::EStop || a == PanelAction::Home; }

uint32_t panel_guard(uint32_t pressed) { return (pressed & (pressed - 1)) ? 0 : pressed; }

std::string panel_line(PanelAction a, const PanelPresets& p, const std::string& custom) {
    char b[120];
    switch (a) {
    case PanelAction::Preheat:
        snprintf(b, sizeof b, "SF_PREHEAT EXTRUDER=%d BED=%d", p.preheat_ext, p.preheat_bed);
        return b;
    case PanelAction::Load:
        snprintf(b, sizeof b, "SF_LOAD_FILAMENT TEMP=%d LENGTH=%d SPEED=%d", p.load_temp, p.load_len, p.load_speed);
        return b;
    case PanelAction::Unload:
        snprintf(b, sizeof b, "SF_UNLOAD_FILAMENT TEMP=%d LENGTH=%d SPEED=%d", p.load_temp, p.unload_len,
                 p.unload_speed);
        return b;
    case PanelAction::EStop: return "M112";
    case PanelAction::Cooldown: return "SF_COOLDOWN";
    case PanelAction::PauseResume: return "SF_PAUSE_RESUME";
    case PanelAction::Home: return "G28";
    case PanelAction::Custom: return custom;
    default: return "";
    }
}

bool TouchFilter::update(uint32_t raw, double now) {
    last_ = double(raw);
    if (n_ < kLearn) {   // learn the untouched level at start
        base_ = n_ == 0 ? last_ : base_ + (last_ - base_) / double(n_ + 1);
        n_++;
        return pressed_ = false;
    }
    double up = base_ > 0 ? (last_ - base_) / base_ : 0;
    if (!pressed_ && up > sens_) {
        pressed_ = true;
        since_ = now;
    } else if (pressed_ && up < sens_ * 0.6) {
        pressed_ = false;
    }
    if (pressed_ && now - since_ > 20.0) {   // stuck: this is the new normal
        base_ = last_;
        pressed_ = false;
    }
    if (!pressed_) base_ += (last_ - base_) / 256.0;   // follow slow drift (about 5 s at 50 readings/s)
    return pressed_;
}

PressTimer::Event PressTimer::update(bool pressed, double now) {
    if (t0_ < 0) t0_ = now;
    if (now - t0_ < startup_) {   // a hand near the pads at power-on, the baseline still settling
        down_ = -1;
        fired_ = counted_ = false;
        return Nothing;
    }
    if (!pressed) {
        Event e = Nothing;
        if (down_ >= 0 && hold_btn_ && !fired_ && counted_) e = TooShort;
        down_ = -1;
        fired_ = counted_ = false;
        return e;
    }
    if (down_ < 0) down_ = now;
    double held = now - down_;
    if (held >= deb_) counted_ = true;
    if (fired_) return Nothing;
    if (!hold_btn_ && held >= deb_) {
        fired_ = true;
        return Fire;
    }
    if (hold_btn_ && held >= hold_) {
        fired_ = true;
        return Fire;
    }
    return Nothing;
}

}  // namespace ph
