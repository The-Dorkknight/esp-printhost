// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The CAPTAIN / SKIPPER extras of ph::Machine (2026-10-24), after Klipper v0.13.0:
//   TEST_RESONANCES / SHAPER_CALIBRATE AXIS=X|Y [FREQ_START FREQ_END ACCEL_PER_HZ HZ_PER_SEC POINT=x,y,z]
//       (resonance_tester.py; the accelerometer and the analysis are the firmware's, MachineIO::accel_record)
//   BED_MESH_CALIBRATE [PROBE_COUNT=x,y] [MESH_MIN=x,y] [MESH_MAX=x,y], BED_MESH_OUTPUT, BED_MESH_CLEAR,
//   BED_MESH_PROFILE SAVE=default | LOAD=default | REMOVE=default        (bed_mesh.py)
//   SCREWS_TILT_CALCULATE [DIRECTION=CW|CCW]                              (screws_tilt_adjust.py)
//   EXCLUDE_OBJECT_DEFINE NAME= [CENTER=x,y], EXCLUDE_OBJECT_START NAME=, EXCLUDE_OBJECT_END,
//   EXCLUDE_OBJECT NAME= | CURRENT=1 | RESET=1                            (exclude_object.py)
#include "ph/config_ini.h"
#include "ph/machine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ph {

#if PH_PRO
namespace {
std::string fmtd(const char* f, double v) {
    char b[64];
    snprintf(b, sizeof b, f, v);
    return b;
}
bool pair_of(const std::map<std::string, std::string>& p, const char* k, double& a, double& b) {
    auto it = p.find(k);
    if (it == p.end()) return false;
    std::string v = it->second;
    for (auto& c : v)
        if (c == ',') c = ' ';
    return sscanf(v.c_str(), "%lf %lf", &a, &b) == 2;
}
std::string jstr(const std::string& s) {
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') o += '\\';
        if ((unsigned char)c >= 32) o += c;
    }
    return o + "\"";
}
}  // namespace
#endif

bool Machine::Objects::is_excluded(const std::string& n) const {
    return !n.empty() && std::find(excluded.begin(), excluded.end(), n) != excluded.end();
}

#if PH_PRO

// ---------------------------------------------------------------- probing: bed mesh and screws

std::string Machine::start_probe_run(int kind, const std::map<std::string, std::string>& p, double now) {
    if (!th_ || !th_->ok()) return "error: moves are not set up";
    if (!plan_.probe.present) return "error: no [probe] in printer.cfg (a sensorless Z probe works)";
    if (!th_->homed_all()) return "error: home all axes first (G28)";
    if (pr_.active || rt_.stage || mac_.active || hs_.active) return "error: something else is running";
    if (!th_->idle(now) && !printing()) return "error: moves are still running";
    ProbeRun r;
    r.kind = kind;
    if (kind == 1) {
        if (!plan_.mesh.present) return "error: no [bed_mesh] in printer.cfg (mesh_min, mesh_max, probe_count)";
        MeshConfig mc = plan_.mesh;
        double a, b;
        if (pair_of(p, "MESH_MIN", a, b)) mc.min_x = a, mc.min_y = b;
        if (pair_of(p, "MESH_MAX", a, b)) mc.max_x = a, mc.max_y = b;
        auto it = p.find("PROBE_COUNT");
        if (it != p.end()) {
            if (pair_of(p, "PROBE_COUNT", a, b)) mc.count_x = int(a), mc.count_y = int(b);
            else mc.count_x = mc.count_y = atoi(it->second.c_str());
        }
        if (mc.count_x < 3 || mc.count_y < 3 || mc.count_x > 10 || mc.count_y > 10)
            return "error: PROBE_COUNT 3 to 10 each way";
        if (mc.max_x <= mc.min_x || mc.max_y <= mc.min_y) return "error: MESH_MAX must be above MESH_MIN";
        r.mesh.cols = mc.count_x, r.mesh.rows = mc.count_y;
        r.mesh.min_x = mc.min_x, r.mesh.max_x = mc.max_x, r.mesh.min_y = mc.min_y, r.mesh.max_y = mc.max_y;
        r.mesh.z.assign(size_t(mc.count_x * mc.count_y), 0.0);
        // Klipper's order: rows from the front, every other row backwards (zig-zag)
        for (int row = 0; row < mc.count_y; row++)
            for (int k = 0; k < mc.count_x; k++) {
                int col = (row % 2) ? mc.count_x - 1 - k : k;
                r.pts.push_back({r.mesh.point_x(col) - plan_.probe.x_offset, r.mesh.point_y(row) - plan_.probe.y_offset});
                r.slot.push_back(row * mc.count_x + col);
            }
        r.hz = mc.horizontal_move_z;
        r.speed = mc.speed;
        th_->set_mesh(nullptr);   // probing measures the bare bed
    } else {
        if (!plan_.screws.present) return "error: no [screws_tilt_adjust] in printer.cfg (screw1, screw2, screw3...)";
        for (auto& s : plan_.screws.screws) r.pts.push_back({s.x, s.y}), r.slot.push_back(int(r.slot.size()));
        r.hz = plan_.screws.horizontal_move_z;
        r.speed = plan_.screws.speed;
        auto it = p.find("DIRECTION");
        if (it != p.end()) {
            r.direction = it->second;
            for (auto& c : r.direction) c = char(toupper((unsigned char)c));
            if (r.direction != "CW" && r.direction != "CCW") return "error: DIRECTION must be CW or CCW";
        }
    }
    r.z.assign(r.pts.size(), 0.0);
    r.active = true;
    pr_ = r;
    return "ok " + std::string(kind == 1 ? "bed mesh" : "screw tilt") + ": probing " + std::to_string(pr_.pts.size()) +
           " points (watch the status)";
}

void Machine::pro_tick(double now) {
    // ---- probing
    if (pr_.active && th_ && !th_->homing() && th_->idle(now)) {
        auto fail = [&](const std::string& why) {
            pr_.active = false;
            pr_.error = why;
            if (io_.log) io_.log(std::string(pr_.kind == 1 ? "bed mesh" : "screw tilt") + " stopped: " + why);
        };
        const double NANV = std::nan("");
        if (pr_.stage == 0) {   // up to the travel height
            if (pr_.idx >= pr_.pts.size()) {
                pr_.active = false;
                if (pr_.kind == 1) {
                    for (size_t i = 0; i < pr_.z.size(); i++) pr_.mesh.z[size_t(pr_.slot[i])] = pr_.z[i] - plan_.probe.z_offset;
                    th_->set_mesh(&pr_.mesh);
                    if (io_.log) io_.log(pr_.mesh.text());
                } else {
                    screws_text_ = screws_report(plan_.screws, pr_.z, pr_.direction, &screws_json_);
                    if (io_.log) io_.log(screws_text_);
                }
                return;
            }
            double xyz[3] = {NANV, NANV, pr_.hz};
            std::string r = th_->move_to(xyz, 10, now);
            if (r.rfind("ok", 0) != 0) return fail(r);
            pr_.stage = 1;
        } else if (pr_.stage == 1) {   // over the point
            double xyz[3] = {pr_.pts[pr_.idx].first, pr_.pts[pr_.idx].second, NANV};
            std::string r = th_->move_to(xyz, pr_.speed, now);
            if (r.rfind("ok", 0) != 0) return fail(r);
            pr_.stage = 2;
        } else if (pr_.stage == 2) {
            std::string r = th_->gcode("PROBE", nullptr, {}, true, now);
            if (r.rfind("ok", 0) != 0) return fail(r);
            pr_.stage = 3;
        } else {
            double z = 0;
            if (!th_->probe_result(z)) return fail(th_->last_error().empty() ? "the probe did not report" : th_->last_error());
            pr_.z[pr_.idx++] = z;
            pr_.stage = 0;
        }
    }
    // ---- resonance test
    if (rt_.stage && th_) {
        Test& t = test(rt_.axis ? "res_y" : "res_x");
        if (rt_.stage == 1 && !th_->homing() && th_->idle(now)) {
            rt_.stage = 2;
            rt_.t = now;
        } else if (rt_.stage == 2 && now - rt_.t > 0.5) {   // resonance_tester.py: dwell 0.5 s first
            std::string err;
            if (!io_.accel_record || !io_.accel_record(1, rt_.axis, &err)) {
                rt_.stage = 0;
                finish(t, "failed", "the accelerometer did not start: " + (err.empty() ? std::string("none set up") : err));
                return;
            }
            std::string r = th_->start_resonance(rt_.axis, rt_.m, now);
            if (!r.empty()) {
                io_.accel_record(-1, rt_.axis, nullptr);
                rt_.stage = 0;
                finish(t, "failed", r);
                return;
            }
            rt_.stage = 3;
            rt_.t = now + 0.25;   // the motion starts START_DELAY later
        } else if (rt_.stage == 3) {
            double el = now - rt_.t;
            if (th_->resonance_running()) {
                double f = rt_.seq.empty() ? 0 : rt_.seq.front().freq;
                for (size_t lo = 0, hi = rt_.seq.size(); lo < hi;) {
                    size_t mid = (lo + hi) / 2;
                    if (rt_.seq[mid].t < el) lo = mid + 1, f = rt_.seq[mid].freq;
                    else hi = mid;
                }
                t.detail = "testing " + fmtd("%.0f Hz", f) + " (" + fmtd("%.0f", 100 * std::min(1.0, el / rt_.m.end_t)) + " % done)";
            } else {
                rt_.stage = 0;
                std::string err;
                bool ok = io_.accel_record(0, rt_.axis, &err);
                finish(t, ok ? "passed" : "failed",
                       ok ? std::string(rt_.calibrate ? "done: the ESP32 is working out the best input shaper (Input shaping box)"
                                                      : "done: the spectrum is in the Input shaping box")
                          : "the accelerometer data could not be used: " + err);
            }
        }
    }
}

std::string Machine::start_resonance_test(const std::map<std::string, std::string>& p, bool calibrate, double now) {
    if (!th_ || !th_->ok()) return "error: moves are not set up";
    if (plan_.printer.delta) return "error: the resonance test is not supported on a delta yet";
    auto it = p.find("AXIS");
    std::string ax = it == p.end() ? "" : it->second;
    int axis = (ax == "X" || ax == "x") ? 0 : (ax == "Y" || ax == "y") ? 1 : -1;
    if (axis < 0) return "error: AXIS=X or AXIS=Y (one at a time: the accelerometer goes on the part that moves)";
    if (!io_.accel_record) return "error: no accelerometer support here";
    if (!th_->homed_all()) return "error: home all axes first (G28)";
    if (printing() || mac_.active || pr_.active || rt_.stage || hs_.active) return "error: something else is running";
    if (!th_->idle(now)) return "error: moves are still running";
    ResonanceParams rp = plan_.resonance;
    std::string err;
    auto num = [&](const char* k, double& v, double lo, double hi) {
        auto i = p.find(k);
        if (i == p.end()) return true;
        double d = atof(i->second.c_str());
        if (!(d >= lo && d <= hi)) {
            err = std::string(k) + " must be " + fmtd("%g", lo) + " to " + fmtd("%g", hi);
            return false;
        }
        v = d;
        return true;
    };
    if (!num("FREQ_START", rp.min_freq, 1, 300) || !num("FREQ_END", rp.max_freq, rp.min_freq, 300) ||
        !num("ACCEL_PER_HZ", rp.accel_per_hz, 1, 1000) || !num("HZ_PER_SEC", rp.hz_per_sec, 0.1, 2))
        return "error: " + err;
    const StepperObj& sx = *plan_.stepper("stepper_x");
    const StepperObj& sy = *plan_.stepper("stepper_y");
    const StepperObj& sz = *plan_.stepper("stepper_z");
    double pt[3] = {(sx.position_min + sx.position_max) / 2, (sy.position_min + sy.position_max) / 2,
                    std::min(std::max(th_->machine_position(2), 10.0), sz.position_max)};
    if (plan_.resonance_point_set)
        for (int i = 0; i < 3; i++) pt[i] = plan_.resonance_point[i];
    auto pi = p.find("POINT");
    if (pi != p.end()) {
        std::string v = pi->second;
        for (auto& c : v)
            if (c == ',') c = ' ';
        if (sscanf(v.c_str(), "%lf %lf %lf", &pt[0], &pt[1], &pt[2]) != 3) return "error: POINT=x,y,z";
    }
    rt_ = ResTest();
    rt_.axis = axis;
    rt_.calibrate = calibrate;
    rt_.seq = resonance_test_sequence(rp);
    rt_.m = resonance_motion(rt_.seq, plan_.printer.max_accel);
    const StepperObj& s = axis ? sy : sx;
    if (pt[axis] + rt_.m.min_x < s.position_min || pt[axis] + rt_.m.max_x > s.position_max)
        return "error: the test swings " + fmtd("%.1f", rt_.m.min_x) + " .. +" + fmtd("%.1f", rt_.m.max_x) + " mm around " +
               fmtd("%.1f", pt[axis]) + ", outside the " + (axis ? "Y" : "X") + " limits";
    for (int i = 0; i < 3; i++) rt_.pt[i] = pt[i];
    std::string r = th_->move_to(pt, 50, now);
    if (r.rfind("ok", 0) != 0) return r;
    rt_.stage = 1;
    Test& t = test(axis ? "res_y" : "res_x");
    t.state = "running";
    t.started = now;
    t.detail = "moving to " + fmtd("%.0f", pt[0]) + "," + fmtd("%.0f", pt[1]) + "," + fmtd("%.0f", pt[2]);
    return "ok resonance test " + std::string(axis ? "Y" : "X") + ": " + fmtd("%.0f", rp.min_freq) + " to " +
           fmtd("%.0f Hz", rp.max_freq) + ", about " + fmtd("%.0f s", rt_.m.end_t) + ", up to " +
           fmtd("%.0f mm/s^2", rt_.m.max_a) + " (keep clear; Abort stops it)";
}

void Machine::stop_pro(const std::string& why, double now) {
    if (pr_.active) {
        pr_.active = false;
        pr_.error = why;
        if (th_ && th_->homing()) th_->abort_homing(now);
    }
    if (rt_.stage) {
        if (th_) th_->abort_resonance(now);
        if (io_.accel_record) io_.accel_record(-1, rt_.axis, nullptr);
        Test& t = test(rt_.axis ? "res_y" : "res_x");
        if (t.state == "running") finish(t, "aborted", why + "; the axis needs homing again");
        rt_.stage = 0;
    }
}

double Machine::Tower::calc(double z) const {
    if (skip > 0) z = std::max(0.0, z - skip);
    if (step_height > 0) return start + step_delta * std::floor(z / step_height);
    if (band > 0) z = (std::floor(z / band) + 0.5) * band;
    return start + z * factor;
}

// An extrusion move at a new height: run the command with the new value (tuning_tower.py move())
void Machine::tower_after_move(const double before[4], double now) {
    if (!tt_.on || !th_) return;
    double z = th_->position(2);
    bool moved = th_->position(0) != before[0] || th_->position(1) != before[1] || z != before[2];
    if (!(th_->position(3) > before[3]) || z == tt_.last_z || !moved) return;
    if (z < tt_.last_z - 2.0) {   // CANCEL_Z_DELTA: extruding lower down, a new print
        tt_.on = false;
        if (io_.log) io_.log("tuning tower: ended");
        return;
    }
    tt_.last_z = z;
    double v = tt_.calc(z);
    if (v == tt_.last_value) return;
    tt_.last_value = v;
    std::string cmd = tt_.fmt_cmd + " " + tt_.param + "=" + fmtd("%.9f", v);
    std::string r = command(cmd, now);
    if (io_.log) io_.log("tuning tower z=" + fmtd("%.2f", z) + ": " + cmd + " -> " + r.substr(0, 60));
}

bool Machine::pro_command(const std::string& name, const std::map<std::string, std::string>& p, const GLine* g, double now,
                          std::string& reply) {
    (void)g;
    if (name == "TUNING_TOWER") {
        auto get = [&](const char* k) {
            auto it = p.find(k);
            return it == p.end() ? std::string() : it->second;
        };
        Tower t;
        t.fmt_cmd = get("COMMAND"), t.param = get("PARAMETER");
        for (auto& c : t.fmt_cmd) c = char(toupper((unsigned char)c));
        for (auto& c : t.param) c = char(toupper((unsigned char)c));
        if (t.fmt_cmd.empty() || t.param.empty()) return reply = "error: TUNING_TOWER needs COMMAND= and PARAMETER=", true;
        t.start = atof(get("START").c_str()), t.factor = atof(get("FACTOR").c_str()), t.band = atof(get("BAND").c_str());
        t.step_delta = atof(get("STEP_DELTA").c_str()), t.step_height = atof(get("STEP_HEIGHT").c_str());
        t.skip = atof(get("SKIP").c_str());
        if (t.factor != 0 && (t.step_height != 0 || t.step_delta != 0))
            return reply = "error: Cannot specify both FACTOR and STEP_DELTA/STEP_HEIGHT", true;
        if ((t.step_delta != 0) != (t.step_height != 0)) return reply = "error: Must specify both STEP_DELTA and STEP_HEIGHT", true;
        t.on = true;
        tt_ = t;
        if (t.fmt_cmd == "SET_PRESSURE_ADVANCE" && th_) {   // keep the advance machinery on from now (it can only switch on at a stop)
            ShapingConfig c = th_->shaping();
            c.pa_on = true;
            th_->set_shaping(c, now);
        }
        reply = "ok Starting tuning test (start=" + fmtd("%.6f", t.start) +
                (t.step_height ? " step_delta=" + fmtd("%.6f", t.step_delta) + " step_height=" + fmtd("%.6f", t.step_height)
                               : " factor=" + fmtd("%.6f", t.factor) + (t.band ? " band=" + fmtd("%.6f", t.band) : "")) + ")";
        return true;
    }
    if (name == "TEST_RESONANCES" || name == "SHAPER_CALIBRATE") {
        reply = start_resonance_test(p, name == "SHAPER_CALIBRATE", now);
        return true;
    }
    if (name == "BED_MESH_CALIBRATE") {
        reply = printing() ? "error: in a print, BED_MESH_CALIBRATE goes in the file" : start_probe_run(1, p, now);
        return true;
    }
    if (name == "SCREWS_TILT_CALCULATE") {
        reply = printing() ? "error: a print is running" : start_probe_run(2, p, now);
        return true;
    }
    if (name == "BED_MESH_OUTPUT") {
        reply = "ok " + (th_ ? th_->mesh().text() : std::string("no bed mesh"));
        return true;
    }
    if (name == "BED_MESH_CLEAR") {
        if (th_) th_->set_mesh(nullptr);
        reply = "ok bed mesh cleared (not used for moves until loaded or probed again)";
        return true;
    }
    if (name == "BED_MESH_PROFILE") {
        if (!th_) return reply = "error: moves are not set up", true;
        auto get = [&](const char* k) {
            auto it = p.find(k);
            return it == p.end() ? std::string() : it->second;
        };
        std::string sv = get("SAVE"), ld = get("LOAD"), rm = get("REMOVE");
        std::string prof = !sv.empty() ? sv : !ld.empty() ? ld : rm;
        if (prof.empty()) return reply = "error: BED_MESH_PROFILE SAVE=default | LOAD=default | REMOVE=default", true;
        if (prof != "default") return reply = "error: only the profile 'default' is kept (printer.cfg is 4 KB)", true;
        if (!ld.empty()) {
            if (!plan_.mesh_default.valid()) return reply = "error: no saved mesh in printer.cfg", true;
            th_->set_mesh(&plan_.mesh_default);
            return reply = "ok bed mesh 'default' loaded", true;
        }
        if (!io_.config_text || !io_.save_config) return reply = "error: saving printer.cfg is not available here", true;
        if (printing() || !th_->idle(now)) return reply = "error: wait until nothing moves (saving restarts the board)", true;
        std::string text = io_.config_text(), e2;
        if (!sv.empty()) {
            if (!th_->mesh().valid()) return reply = "error: no bed mesh to save (BED_MESH_CALIBRATE first)", true;
            for (auto& kv : th_->mesh().profile_options()) text = config_set_option(text, "bed_mesh default", kv.first, kv.second);
        } else {
            // remove the section: rebuild without it
            std::string out, line;
            bool skip = false;
            for (size_t i = 0; i <= text.size(); i++) {
                if (i == text.size() || text[i] == '\n') {
                    std::string t = line;
                    while (!t.empty() && isspace((unsigned char)t.back())) t.pop_back();
                    if (!t.empty() && t[0] == '[') skip = t == "[bed_mesh default]";
                    if (!skip) out += line + (i < text.size() ? "\n" : "");
                    line.clear();
                } else {
                    line += text[i];
                }
            }
            text = out;
        }
        if (!io_.save_config(text, &e2)) return reply = "error: could not save printer.cfg: " + e2, true;
        reply = std::string("ok bed mesh 'default' ") + (sv.empty() ? "removed from" : "saved in") +
                " printer.cfg (the board restarts with it)";
        return true;
    }
    if (name == "EXCLUDE_OBJECT_DEFINE") {
        auto it = p.find("NAME");
        if (it == p.end()) return reply = "error: EXCLUDE_OBJECT_DEFINE needs NAME=", true;
        std::string n = it->second;
        for (auto& c : n) c = char(toupper((unsigned char)c));
        double cx = NAN, cy = NAN;
        pair_of(p, "CENTER", cx, cy);
        // POLYGON=[[x,y],[x,y],...]: the outline (Klipper's exclude_object keeps it for Moonraker's clients)
        std::vector<std::pair<float, float>> poly;
        auto pg = p.find("POLYGON");
        if (pg != p.end()) {
            std::vector<double> v;
            const char* c = pg->second.c_str();
            while (*c && v.size() < 512) {
                if ((*c >= '0' && *c <= '9') || *c == '-' || *c == '.' || *c == '+') {
                    char* e = nullptr;
                    double d = strtod(c, &e);
                    if (e == c) break;
                    if (std::isfinite(d) && std::fabs(d) < 1e5) v.push_back(d);
                    c = e;
                } else {
                    c++;
                }
            }
            for (size_t i = 0; i + 1 < v.size(); i += 2) poly.push_back({float(v[i]), float(v[i + 1])});
        }
        auto num = [&](const char* k) {
            auto it2 = p.find(k);
            if (it2 == p.end()) return double(NAN);
            char* e = nullptr;
            double d = strtod(it2->second.c_str(), &e);
            return e != it2->second.c_str() && std::isfinite(d) ? d : double(NAN);
        };
        auto f = std::find(obj_.names.begin(), obj_.names.end(), n);
        if (f == obj_.names.end() && obj_.names.size() < 64) {
            obj_.names.push_back(n);
            obj_.centers.push_back({cx, cy});
            obj_.polygons.push_back(poly);
            obj_.heights.push_back({num("PI_Z0"), num("PI_Z1")});
            obj_.rev++;
        } else if (f != obj_.names.end()) {   // defined again (Klipper replaces it)
            size_t i = size_t(f - obj_.names.begin());
            obj_.centers[i] = {cx, cy};
            obj_.polygons[i] = poly;
            obj_.heights[i] = {num("PI_Z0"), num("PI_Z1")};
            obj_.rev++;
        }
        return reply = "ok", true;
    }
    if (name == "EXCLUDE_OBJECT") {
        auto it = p.find("NAME");
        if (p.count("RESET")) {
            obj_.excluded.clear();
            obj_.rev++;
            return reply = "ok no objects excluded", true;
        }
        std::string n = p.count("CURRENT") ? obj_.current : it == p.end() ? "" : it->second;
        for (auto& c : n) c = char(toupper((unsigned char)c));
        if (n.empty()) {
            std::string l;
            for (auto& x : obj_.excluded) l += " " + x;
            return reply = "ok excluded:" + (l.empty() ? std::string(" none") : l), true;
        }
        if (!obj_.is_excluded(n)) obj_.excluded.push_back(n), obj_.rev++;
        if (io_.log) io_.log("excluding object " + n);
        return reply = "ok " + n + " excluded: the rest of it is skipped", true;
    }
    return false;
}

bool Machine::pro_print_line(const std::string& name, const std::map<std::string, std::string>& p, const GLine* g,
                             double now, std::string& err) {
    if (name == "EXCLUDE_OBJECT_START") {
        auto it = p.find("NAME");
        obj_.current = it == p.end() ? "" : it->second;
        for (auto& c : obj_.current) c = char(toupper((unsigned char)c));
        return true;
    }
    if (name == "EXCLUDE_OBJECT_END") {
        obj_.current.clear();
        return true;
    }
    if (name == "EXCLUDE_OBJECT_DEFINE" || name == "EXCLUDE_OBJECT") {
        std::string r;
        pro_command(name, p, g, now, r);
        return true;
    }
    // slicer start G-code: pressure advance, input shaper, tuning tower (never SAVE from a file)
    if (name == "TUNING_TOWER" || name == "SET_PRESSURE_ADVANCE" || name == "SET_INPUT_SHAPER") {
        std::string r;
        if (name == "TUNING_TOWER") {
            pro_command(name, p, g, now, r);
        } else {
            std::map<std::string, std::string> q = p;
            q.erase("SAVE");
            r = set_shaping_cmd(name, q, now);
        }
        if (r.rfind("error", 0) == 0) err = r.substr(7);
        else if (io_.log) io_.log(name + ": " + r.substr(r.size() > 3 ? 3 : 0));
        return true;
    }
    if (obj_.is_excluded(obj_.current)) {
        if (g && (name == "G0" || name == "G1" || name == "G2" || name == "G3")) {
            th_->skip_move(*g);   // the G-code position follows; nothing moves, nothing extrudes
            return true;
        }
        if (name == "G10" || name == "G11") return true;   // its retractions too (they come in pairs)
    }
    if (name == "BED_MESH_CALIBRATE") {   // a start G-code's mesh: the print waits for it
        if (th_) th_->flush_lookahead();
        std::string r = start_probe_run(1, p, now);
        if (r.rfind("error", 0) == 0) {
            err = r.substr(7);
            return true;
        }
        job_.wait = 6;
        return true;
    }
    if (name == "BED_MESH_PROFILE" || name == "BED_MESH_CLEAR") {
        if (name == "BED_MESH_CLEAR" || (p.count("LOAD") && plan_.mesh_default.valid())) {
            std::string r;
            pro_command(name, p, g, now, r);
        }
        return true;
    }
    return false;
}

std::string Machine::pro_status_json(double now) const {
    (void)now;
    std::string j = ",\"mesh\":" + (th_ ? th_->mesh().json() : std::string("null"));
    j += ",\"probing\":{\"active\":" + std::string(pr_.active ? "true" : "false") + ",\"kind\":\"" +
         (pr_.kind == 1 ? "mesh" : pr_.kind == 2 ? "screws" : "") + "\",\"point\":" + std::to_string(pr_.idx) +
         ",\"points\":" + std::to_string(pr_.pts.size()) + ",\"error\":" + jstr(pr_.error) + "}";
    j += ",\"screws\":" + (screws_json_.empty() ? std::string("null") : screws_json_);
    j += ",\"objects\":{\"current\":" + jstr(obj_.current) + ",\"list\":[";
    for (size_t i = 0; i < obj_.names.size(); i++) {
        j += (i ? "," : "") + std::string("{\"name\":") + jstr(obj_.names[i]) + ",\"excluded\":" +
             (obj_.is_excluded(obj_.names[i]) ? "true" : "false");
        if (!std::isnan(obj_.centers[i].first)) j += ",\"x\":" + fmtd("%.1f", obj_.centers[i].first) + ",\"y\":" + fmtd("%.1f", obj_.centers[i].second);
        j += "}";
    }
    j += "]}";
    return j;
}

#else   // DECKHAND: none of the above

std::string Machine::start_probe_run(int, const std::map<std::string, std::string>&, double) { return "error: CAPTAIN+"; }
std::string Machine::start_resonance_test(const std::map<std::string, std::string>&, bool, double) { return "error: CAPTAIN+"; }
void Machine::pro_tick(double) {}
void Machine::stop_pro(const std::string&, double) {}
bool Machine::pro_command(const std::string& name, const std::map<std::string, std::string>&, const GLine*, double,
                          std::string& reply) {
    static const char* const PRO[] = {"TEST_RESONANCES", "SHAPER_CALIBRATE", "BED_MESH_CALIBRATE", "SCREWS_TILT_CALCULATE",
                                      "BED_MESH_OUTPUT", "BED_MESH_CLEAR", "BED_MESH_PROFILE", "EXCLUDE_OBJECT", "EXCLUDE_OBJECT_DEFINE",
                                      "TUNING_TOWER"};
    for (auto* n : PRO)
        if (name == n) return reply = "error: " + name + " needs the CAPTAIN or SKIPPER build (more flash)", true;
    return false;
}
bool Machine::pro_print_line(const std::string& name, const std::map<std::string, std::string>&, const GLine*, double,
                             std::string&) {
    return name == "EXCLUDE_OBJECT_START" || name == "EXCLUDE_OBJECT_END" || name == "EXCLUDE_OBJECT_DEFINE";
}
std::string Machine::pro_status_json(double) const { return ""; }
double Machine::Tower::calc(double) const { return 0; }
void Machine::tower_after_move(const double*, double) {}

#endif

}  // namespace ph
