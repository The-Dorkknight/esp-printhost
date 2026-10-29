// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Buttons on the printer's case (2026-10-26), as in KlipperTFT-Pico: up to four inputs, each a
// capacitive touch pad (a bolt on a wire) or a switch to GND. Defaults: Preheat, Home, Unload,
// E-stop (Home and E-stop only on a 1 s hold: they move or stop the printer). The ESP32-S3 firmware reads the pins (main/panel.cpp); the
// logic is here so it is tested on the PC. Each button press becomes one console line, which
// ph::Machine runs (SF_PREHEAT, SF_LOAD_FILAMENT, ... see machine_panel.cpp).
#pragma once
#include <cstdint>
#include <string>

namespace ph {

enum class PanelAction { None = 0, Preheat, Load, Unload, EStop, Cooldown, PauseResume, Home, Custom };
constexpr int kPanelButtons = 4;
constexpr int kPanelActions = 9;
const char* panel_action_id(PanelAction a);      // "preheat", "load", ... (settings, web page)
const char* panel_action_label(PanelAction a);   // "Preheat", "Load filament", ...
PanelAction panel_action_from(const std::string& id);   // None if unknown
PanelAction panel_default_action(int button);           // Preheat, Home, Unload, EStop
// Actions that fire only after a 1 s hold (a brush against the case must not move or stop anything).
bool panel_hold_action(PanelAction a);   // EStop, Home
// Buttons that count, from the ones pressed (bit i = button i): two or more at once (wires shorted
// together, a hand across two bolts) count as none.
uint32_t panel_guard(uint32_t pressed);

// Values the buttons use (stored with the button settings on the ESP32).
struct PanelPresets {
    int preheat_ext = 200, preheat_bed = 60;   // C (PLA)
    int load_temp = 210;                        // C: the hotend is heated to at least this first
    int load_len = 40, unload_len = 45;         // mm
    int load_speed = 5, unload_speed = 20;      // mm/s
};

// The console line a button runs. Custom: the line typed for it (a [gcode_macro] name, a G-code).
std::string panel_line(PanelAction a, const PanelPresets& p, const std::string& custom = "");
// Short text for the OLED when the button fires ("PREHEAT", "LOAD", ...).
const char* panel_flash(PanelAction a);

// Capacitive touch: the ESP32-S3's touch reading goes UP when a finger is on the pad. The baseline
// is learnt at start (pads untouched) and follows slow drift (temperature, humidity) while not
// touched. Touched when the reading is `sens` (fraction) above the baseline; released below 60 %
// of that (hysteresis). A pad that reads "touched" for more than 20 s is taken as a new baseline
// (something leaning on it, or a wire moved), so it cannot hold a button down forever.
class TouchFilter {
public:
    explicit TouchFilter(double sens = 0.05) : sens_(sens) {}
    void set_sensitivity(double sens) { sens_ = sens; }
    // One reading; returns pressed().
    bool update(uint32_t raw, double now);
    bool pressed() const { return pressed_; }
    bool ready() const { return n_ >= kLearn; }
    double baseline() const { return base_; }
    double change() const { return base_ > 0 ? (last_ - base_) / base_ : 0; }   // e.g. 0.08 = 8 % up
    void reset() { *this = TouchFilter(sens_); }
private:
    static constexpr int kLearn = 16;
    double sens_, base_ = 0, last_ = 0, since_ = -1;
    int n_ = 0;
    bool pressed_ = false;
};

// Debounced press -> events. A press must last `debounce` s. Normal buttons fire once on the press;
// a hold button (E-stop) fires only after `hold` s still pressed, and a shorter tap is reported as
// TooShort (the OLED then says "HOLD 1 S"). Nothing fires during the first `startup` s.
class PressTimer {
public:
    enum Event { Nothing = 0, Fire, TooShort };
    PressTimer(bool hold_button = false, double hold = 1.0, double debounce = 0.06, double startup = 2.0)
        : hold_btn_(hold_button), hold_(hold), deb_(debounce), startup_(startup) {}
    void set_hold_button(bool h) { hold_btn_ = h; }
    Event update(bool pressed, double now);
private:
    bool hold_btn_;
    double hold_, deb_, startup_;
    double t0_ = -1, down_ = -1;
    bool fired_ = false, counted_ = false;
};

}  // namespace ph
