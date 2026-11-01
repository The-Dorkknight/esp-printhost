// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
//
// The printer's state in Klipper's own words (2026-10-30, SkiPiPir): the status objects Moonraker
// reports (Klipper docs/Status_Reference.md), so a program written for Moonraker - Pasta Inspector,
// the spaghetti detector - can watch this printer too. Only what that needs, field by field as
// Klipper names them; a field this engine cannot know is left out rather than made up.
#include "ph/machine.h"
#include "ph/toolhead.h"

#if PH_PRO   // CAPTAIN / SKIPPER and the PC build: DECKHAND has no room (and no exclude object)

#include <cmath>
#include <cstdio>
#include <string>

namespace ph {

static std::string q(const std::string& s) {
    std::string o = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') o += '\\', o += char(c);
        else if (c < 0x20) {
            char b[8];
            snprintf(b, sizeof b, "\\u%04x", c);
            o += b;
        } else o += char(c);
    }
    return o + "\"";
}

static std::string num(double v, const char* f = "%.4f") {
    if (!std::isfinite(v)) return "null";
    char b[40];
    snprintf(b, sizeof b, f, v);
    return b;
}

std::string Machine::moonraker_status_json(double now) const {
    // print_stats.state: standby, printing, paused, complete, cancelled, error
    const std::string& js = job_.state;
    const char* st = js == "printing" ? "printing" : js == "paused" ? "paused" : js == "done" ? "complete"
                   : js == "cancelled" ? "cancelled" : js == "error" ? "error" : "standby";
    double prog = (printing() || js == "done") && io_.print_progress ? io_.print_progress() : 0;
    double used = th_ && printing() ? th_->extruded() - job_.e_start : 0;
    std::string j = "{\"print_stats\":{\"state\":\"" + std::string(st) + "\",\"filename\":" + q(job_.file) +
                    ",\"filament_used\":" + num(used < 0 ? 0 : used, "%.2f") + ",\"info\":{\"current_layer\":" +
                    (job_.layer > 0 ? std::to_string(job_.layer) : "null") + ",\"total_layer\":" +
                    (job_.layers > 0 ? std::to_string(job_.layers) : "null") + "}}";
    // toolhead: the last commanded position; motion_report: where the head is now
    bool busy = false;
    if (th_ && th_->ok()) {
        std::string h;
        for (int a = 0; a < 3; a++)
            if (th_->homed(a)) h += "xyz"[a];
        j += ",\"toolhead\":{\"position\":[" + num(th_->position(0)) + "," + num(th_->position(1)) + "," +
             num(th_->position(2)) + "," + num(th_->position(3)) + "],\"homed_axes\":\"" + h + "\"}";
        double p[4], p0[4];
        if (th_->live_position(now, p)) {
            double v = 0;
            if (th_->live_position(now - 0.1, p0)) v = (p[3] - p0[3]) / 0.1;   // filament speed, mm/s
            j += ",\"motion_report\":{\"live_position\":[" + num(p[0]) + "," + num(p[1]) + "," + num(p[2]) + "," +
                 num(p[3]) + "],\"live_extruder_velocity\":" + num(v, "%.3f") + "}";
        }
        busy = !th_->idle(now);
    }
    j += ",\"pause_resume\":{\"is_paused\":" + std::string(js == "paused" ? "true" : "false") + "}";
    j += ",\"idle_timeout\":{\"state\":\"" + std::string(js == "printing" || busy ? "Printing" : "Ready") + "\"}";
    j += ",\"virtual_sdcard\":{\"is_active\":" + std::string(js == "printing" ? "true" : "false") +
         ",\"progress\":" + num(prog) + "}";
    j += ",\"display_status\":{\"progress\":" + num(prog) + "}";
    // the hotend
    for (size_t i = 0; i < plan_.heaters.size() && i < hr_.size(); i++) {
        if (plan_.heaters[i].name != "extruder") continue;
        int s = plan_.heaters[i].sensor;
        double t = s >= 0 && size_t(s) < plan_.sensors.size() ? plan_.sensors[size_t(s)].temp_c : NAN;
        j += ",\"extruder\":{\"temperature\":" + num(t, "%.2f") + ",\"target\":" + num(hr_[i].target, "%.1f") +
             ",\"power\":" + num(hr_[i].last_pwm, "%.3f") + "}";
    }
    // exclude_object, without the object list (moonraker_objects_json)
    j += ",\"exclude_object\":{\"excluded_objects\":[";
    for (size_t i = 0; i < obj_.excluded.size(); i++) j += (i ? "," : "") + q(obj_.excluded[i]);
    j += "],\"current_object\":" + (obj_.current.empty() ? std::string("null") : q(obj_.current)) + "}}";
    return j;
}

std::string Machine::moonraker_objects_json() const {
    std::string j = "[";
    for (size_t i = 0; i < obj_.names.size(); i++) {
        j += (i ? ",{\"name\":" : "{\"name\":") + q(obj_.names[i]);
        if (i < obj_.centers.size() && std::isfinite(obj_.centers[i].first))
            j += ",\"center\":[" + num(obj_.centers[i].first, "%.3f") + "," + num(obj_.centers[i].second, "%.3f") + "]";
        if (i < obj_.polygons.size() && !obj_.polygons[i].empty()) {
            j += ",\"polygon\":[";
            for (size_t k = 0; k < obj_.polygons[i].size(); k++)
                j += (k ? ",[" : "[") + num(obj_.polygons[i][k].first, "%.3f") + "," +
                     num(obj_.polygons[i][k].second, "%.3f") + "]";
            j += "]";
        }
        if (i < obj_.heights.size()) {
            if (std::isfinite(obj_.heights[i].first)) j += ",\"PI_Z0\":" + num(obj_.heights[i].first, "%.3f");
            if (std::isfinite(obj_.heights[i].second)) j += ",\"PI_Z1\":" + num(obj_.heights[i].second, "%.3f");
        }
        j += "}";
    }
    return j + "]";
}

}  // namespace ph

#endif  // PH_PRO
