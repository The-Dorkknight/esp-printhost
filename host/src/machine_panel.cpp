// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// What the buttons on the case run (2026-10-26, see ph/panel.h), as console commands, so they also
// work from the console and from a [gcode_macro]:
//   SF_PREHEAT [EXTRUDER=200] [BED=60]          hotend and bed targets, no waiting
//   SF_COOLDOWN                                 both off
//   SF_LOAD_FILAMENT [TEMP=210] [LENGTH=40] [SPEED=5]     heat (at least TEMP) and wait, then push
//   SF_UNLOAD_FILAMENT [TEMP=210] [LENGTH=45] [SPEED=20]  heat and wait, push 5 mm, then pull back
//   SF_PAUSE_RESUME                             PAUSE while printing, RESUME while paused
// If printer.cfg has [gcode_macro LOAD_FILAMENT] / [gcode_macro UNLOAD_FILAMENT], that runs instead
// (the usual Klipper names). All of them except SF_PAUSE_RESUME are refused while a print, a macro,
// homing, probing or a test is running. The emergency stop is plain M112.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "ph/machine.h"
#include "ph/shaping.h"   // PH_PRO

#if PH_PRO   // CAPTAIN / SKIPPER: the DECKHAND build has no room (2026-10-26)

namespace ph {

namespace {
std::string fmt(const char* f, double a) {
    char b[64];
    snprintf(b, sizeof b, f, a);
    return b;
}
bool getnum(const std::map<std::string, std::string>& p, const char* k, double dflt, double lo, double hi, double& v,
            std::string& err) {
    auto it = p.find(k);
    if (it == p.end()) {
        v = dflt;
        return true;
    }
    char* end = nullptr;
    v = strtod(it->second.c_str(), &end);
    if (end == it->second.c_str() || *end || !std::isfinite(v) || v < lo || v > hi) {
        char b[96];
        snprintf(b, sizeof b, "error: %s must be %g to %g", k, lo, hi);
        err = b;
        return false;
    }
    return true;
}
}  // namespace

bool Machine::is_panel_command(const std::string& name) {
    return name == "SF_PREHEAT" || name == "SF_COOLDOWN" || name == "SF_LOAD_FILAMENT" ||
           name == "SF_UNLOAD_FILAMENT" || name == "SF_PAUSE_RESUME";
}

std::string Machine::panel_command(const std::string& name, const std::map<std::string, std::string>& p, double now) {
    if (name == "SF_PAUSE_RESUME") {
        if (job_.state == "printing") return command("PAUSE", now);
        if (job_.state == "paused") return command("RESUME", now);
        return "error: no print is running";
    }
    if (shutdown_) return "error: the board is shut down (FIRMWARE_RESTART first)";
    if (printing()) return "error: a print is running";
    if (mac_.active) return "error: " + mac_.name + " is running";
    if (hs_.active || pr_.active || rt_.stage || (th_ && th_->homing())) return "error: busy (homing, probing or a test)";
    for (auto& t : tests_)
        if (t.state == "running") return "error: the test '" + t.label + "' is running";
    const HeaterObj* ext = plan_.heater("extruder");
    const HeaterObj* bed = plan_.heater("heater_bed");
    std::string err;
    if (name == "SF_PREHEAT" || name == "SF_COOLDOWN") {
        double te = 0, tb = 0;
        if (name == "SF_PREHEAT" && (!getnum(p, "EXTRUDER", 200, 0, ext ? ext->max_temp : 300, te, err) ||
                                     !getnum(p, "BED", 60, 0, bed ? bed->max_temp : 130, tb, err)))
            return err;
        if (!ext && !bed) return "error: no [extruder] or [heater_bed] in printer.cfg";
        std::string r;
        if (ext) {
            r = command("M104 S" + fmt("%.0f", te), now);
            if (r.rfind("ok", 0) != 0) return r;
        }
        if (bed) {
            r = command("M140 S" + fmt("%.0f", tb), now);
            if (r.rfind("ok", 0) != 0) return r;
        }
        if (name == "SF_COOLDOWN") return "ok cooling down: heaters off";
        return "ok preheating: hotend " + fmt("%.0f", te) + (bed ? " C, bed " + fmt("%.0f", tb) : std::string()) + " C";
    }
    // load / unload
    bool load = name == "SF_LOAD_FILAMENT";
    if (const MacroObj* mo = plan_.macro(load ? "LOAD_FILAMENT" : "UNLOAD_FILAMENT"))
        return start_macro(mo->name, mo->lines, now);
    if (!ext) return "error: no [extruder] in printer.cfg";
    double t, len, speed;
    if (!getnum(p, "TEMP", 210, 0, ext->max_temp, t, err) ||
        !getnum(p, "LENGTH", load ? 40 : 45, 1, 1000, len, err) || !getnum(p, "SPEED", load ? 5 : 20, 0.5, 100, speed, err))
        return err;
    size_t hi = size_t(ext - plan_.heaters.data());
    t = std::max({t, hr_[hi].target, plan_.printer.min_extrude_temp + 5});   // never cooler than now, never too cold
    if (t > ext->max_temp) return "error: the load temperature is above the extruder's max_temp";
    std::vector<std::string> lines = {"M109 S" + fmt("%.0f", t), "M83"};
    if (load) {
        lines.push_back("G1 E" + fmt("%.1f", len) + " F" + fmt("%.0f", speed * 60));
    } else {
        lines.push_back("G1 E5 F300");   // soften the tip so it does not jam on the way out
        lines.push_back("G1 E-" + fmt("%.1f", len) + " F" + fmt("%.0f", speed * 60));
    }
    std::string r = start_macro(name, lines, now);
    if (r.rfind("ok", 0) != 0) return r;
    return std::string("ok ") + (load ? "loading" : "unloading") + ": heating to " + fmt("%.0f", t) + " C first";
}

}  // namespace ph

#endif  // PH_PRO
