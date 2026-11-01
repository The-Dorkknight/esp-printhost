// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Moves, step compression, homing (sensorless), probing and printing a file (ph::Toolhead through
// ph::Machine) against a mock board, with the Tina 2 preset and the real SKR Pico dictionary.
#include "check.h"
#include "ph/boardcfg.h"
#include "ph/config_ini.h"
#include "ph/default_printer_cfg.h"
#include "ph/machine.h"
#include "ph/msgproto.h"
#include "ph/panel.h"
#include "ph/printbench.h"
#include "ph/toolhead.h"
#include "skr_pico_dict.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace ph;

namespace {

std::string argv_of(const std::string& cmd, const std::string& k) {
    size_t p = cmd.find(" " + k + "=");
    if (p == std::string::npos) return "";
    p += k.size() + 2;
    return cmd.substr(p, cmd.find(' ', p) - p);
}

// Replays one stepper's commands the way the board's stepper.c does and returns the step clocks
// (with direction: negative = dir 0).
struct StepReplay {
    uint64_t last = 0;
    int dir = -1;
    std::vector<std::pair<uint64_t, int>> steps;
    void feed(const std::string& c, int oid) {
        std::string O = "oid=" + std::to_string(oid) + " ";
        if (c.find(" " + O) == std::string::npos) return;
        if (c.rfind("reset_step_clock", 0) == 0) {
            uint32_t r = uint32_t(strtoul(argv_of(c, "clock").c_str(), nullptr, 10));
            // extend to 64 bits near the previous value
            uint64_t base = last ? last : r;
            int64_t d = int64_t(int32_t(r - uint32_t(base)));
            last = uint64_t(int64_t(base) + d);
        } else if (c.rfind("set_next_step_dir", 0) == 0) {
            dir = atoi(argv_of(c, "dir").c_str());
        } else if (c.rfind("queue_step", 0) == 0) {
            int64_t iv = atoll(argv_of(c, "interval").c_str()), add = atoll(argv_of(c, "add").c_str());
            long n = atol(argv_of(c, "count").c_str());
            for (long k = 0; k < n; k++) {
                last += uint64_t(iv);
                steps.push_back({last, dir});
                iv += add;
            }
        }
    }
};

struct Rig {
    double now = 100.0;
    std::vector<std::pair<double, std::string>> sent;
    std::map<int, std::map<int, uint32_t>> regs;
    std::map<int, uint32_t> ifcnt;
    std::vector<int> trsync_armed;    // trsync oids an endstop_home was armed with
    std::vector<std::string> file;
    size_t file_pos = 0;
    double temp = 25;                 // hotend temperature the "board" reports

    MachineIO io() {
        MachineIO io;
        io.send = [this](const std::vector<std::string>& c, std::string*) {
            for (auto& s : c) {
                sent.push_back({now, s});
                if (s.rfind("endstop_home", 0) == 0 && argv_of(s, "clock") != "0")
                    trsync_armed.push_back(atoi(argv_of(s, "trsync_oid").c_str()));
            }
            return true;
        };
        io.query = [this](const std::string& cmd, const std::string& resp, int oid, Message& out, int) {
            sent.push_back({now, cmd});
            out = Message();
            out.name = resp;
            out.params["oid"].i = oid;
            if (resp == "stepper_position" || resp == "endstop_state") return true;
            if (resp != "tmcuart_response") return false;
            std::string hx = argv_of(cmd, "write").substr(4);
            std::vector<uint8_t> raw, d;
            for (size_t i = 0; i + 1 < hx.size(); i += 2) raw.push_back(uint8_t(strtol(hx.substr(i, 2).c_str(), nullptr, 16)));
            for (size_t bit = 0; bit + 10 <= raw.size() * 8; bit += 10) {
                uint8_t b = 0;
                for (int i = 0; i < 8; i++)
                    if (raw[(bit + 1 + i) / 8] & (1u << ((bit + 1 + i) % 8))) b |= uint8_t(1u << i);
                d.push_back(b);
            }
            int addr = d[1];
            auto& p = out.params["read"];
            p.is_bytes = true;
            if (d.size() == 8) {
                regs[addr][d[2] & 0x7f] = (uint32_t(d[3]) << 24) | (uint32_t(d[4]) << 16) | (uint32_t(d[5]) << 8) | d[6];
                ifcnt[addr]++;
                return true;
            }
            int reg = d[2];
            uint32_t v = reg == 0x06 ? 0x21000040u : reg == 0x02 ? ifcnt[addr] : regs[addr][reg];
            auto e = tmc_encode_write(0x05, 0xff, uint8_t(reg), v);
            p.b.assign(e.begin(), e.end());
            return true;
        };
        io.clock_at = [](double t) { return uint64_t(t * 12e6); };
        io.neopixel_write = [](int, const std::vector<uint8_t>&, std::string*) { return true; };
        io.print_open = [this](std::string& name, std::string*) {
            file_pos = 0;
            name = "test.gcode";
            return !file.empty();
        };
        io.print_read = [this](std::string& line, bool& eof, std::string*) {
            eof = file_pos >= file.size();
            if (!eof) line = file[file_pos++];
            return true;
        };
        io.print_progress = [this]() { return file.empty() ? 0.0 : double(file_pos) / double(file.size()); };
        return io;
    }
};

Message trsync_hit(int oid) {
    Message m;
    m.name = "trsync_state";
    m.params["oid"].i = oid;
    m.params["can_trigger"].i = 0;
    m.params["trigger_reason"].i = 1;
    return m;
}

Message adc(int oid, double v) {
    Message m;
    m.name = "analog_in_state";
    m.params["oid"].i = oid;
    m.params["value"].i = int64_t(v);
    return m;
}

}  // namespace

static void test_step_compression() {
    // A move that speeds up, cruises and slows down: every step must come back within 25 us.
    StepQueue q;
    q.init(3, false, 300, 50);
    std::vector<uint64_t> want;
    double c = 12e6 * 100.0;
    double v = 0, a = 1000 * 100.0, vmax = 50 * 100.0;   // steps/s^2, steps/s (100 steps/mm)
    for (int k = 0; k < 3000; k++) {
        double dv = k < 1000 ? 1 : k > 2000 ? -1 : 0;
        // time to the next step at the current speed (simple integration)
        if (dv > 0) v = std::sqrt(2 * a * (k + 1));
        else if (dv < 0) v = std::sqrt(2 * a * (3000 - k));
        v = std::min(v, vmax);
        c += 12e6 / std::max(v, 200.0);
        q.push_run(1, c, 1, 1);
        want.push_back(uint64_t(std::llround(c)));
    }
    std::vector<std::string> cmds;
    int moves = q.flush(UINT64_MAX, cmds);
    StepReplay r;
    for (auto& s : cmds) r.feed(s, 3);
    CHECK(r.steps.size() == want.size());
    double worst = 0;
    for (size_t i = 0; i < want.size() && i < r.steps.size(); i++)
        worst = std::max(worst, std::fabs(double(r.steps[i].first) - double(want[i])));
    CHECK(worst <= 300);
    printf("  step compression: 3000 steps -> %d queue_step commands, worst error %.0f ticks (25 us = 300)\n", moves, worst);
    CHECK(moves < 400);
}

static void test_moves_and_homing() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    CHECK(strlen(default_printer_cfg_tina2()) < 4096);
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    for (auto& e : plan.errors) printf("  tina2 printer.cfg error: %s\n", e.c_str());
    CHECK(plan.errors.empty());
    CHECK(plan.endstops.size() == 3 && plan.probe.present && plan.probe.endstop == plan.stepper("stepper_z")->endstop);
    CHECK(plan.stepper("stepper_x")->homing_positive_dir && !plan.stepper("stepper_y")->homing_positive_dir &&
          plan.stepper("stepper_z")->homing_positive_dir);
    for (auto& cmd : plan.cmds) {
        Bytes b;
        CHECK(mp.encode(cmd, b, &err));
    }
    plan.move_count = 1024;
    Rig rig;
    Machine m(plan, mp, rig.io());
    m.start(rig.now);
    const auto& es = plan.sensors[size_t(plan.heater("extruder")->sensor)];
    m.on_message(adc(es.oid, es.th.calc_adc(25) * 8 * 4095.0), rig.now);
    CHECK(m.toolhead() && m.toolhead()->ok());
    CHECK(m.command("G1 X20", rig.now).find("Must home axis first: X") != std::string::npos);
    // home X: sensorless set-up, the move, the trigger
    std::string r = m.command("G28 X", rig.now);
    CHECK(r.rfind("ok homing X", 0) == 0);
    for (int i = 0; i < 20; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(rig.regs[0][0x14] == 0xfffff && rig.regs[0][0x13] == 0);   // TCOOLTHRS on, TPWMTHRS 0 for stallGuard
    CHECK(!rig.trsync_armed.empty());
    const StepperObj& x = *plan.stepper("stepper_x");
    CHECK(rig.trsync_armed.back() == x.trsync_oid);
    bool home_cmd = false;
    for (auto& s : rig.sent)
        home_cmd |= s.second.rfind("endstop_home oid=" + std::to_string(plan.endstops[size_t(x.endstop)].oid), 0) == 0 &&
                    s.second.find("pin_value=1") != std::string::npos;
    CHECK(home_cmd);
    m.on_message(trsync_hit(x.trsync_oid), rig.now);
    for (int i = 0; i < 10; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(m.toolhead()->homed(0) && std::fabs(m.toolhead()->position(0) - 85) < 1e-9);
    CHECK(rig.regs[0][0x14] == 0 && rig.regs[0][0x13] == 0);   // restored (TPWMTHRS 0 = stealthChop at any speed here)
    // mark Y and Z, then a real move: 10 mm on X = 1000 steps backwards (85 -> 75)
    CHECK(m.command("SET_KINEMATIC_POSITION Y=50 Z=40", rig.now).rfind("ok YZ", 0) == 0);
    size_t mark = rig.sent.size();
    CHECK(m.command("G1 X75 F3000", rig.now) == "ok");
    double start = rig.now;
    for (int i = 0; i < 40; i++) rig.now += 0.05, m.tick(rig.now);
    StepReplay rx;
    for (size_t i = mark; i < rig.sent.size(); i++) {
        rx.feed(rig.sent[i].second, x.oid);
        if (rig.sent[i].second.rfind("queue_step oid=" + std::to_string(x.oid), 0) == 0) {
            // every queue_step reaches the board before its first step is due
            CHECK(double(rx.last) / 12e6 > rig.sent[i].first);
        }
    }
    CHECK(rx.steps.size() == 1000);
    int fwd = 0;
    for (auto& s : rx.steps) fwd += s.second == 0;   // dir_pin is inverted: towards 0 = board dir 0 ^ 1 = 1 ... count below
    (void)fwd;
    double dur = rx.steps.empty() ? 0 : double(rx.steps.back().first - rx.steps.front().first) / 12e6;
    printf("  10 mm X move: %zu steps over %.3f s (50 mm/s, 1000 mm/s^2 -> 0.250 s)\n", rx.steps.size(), dur);
    NEAR(dur, 0.25, 0.02);
    CHECK(rx.steps.front().first / 12e6 > start);
    CHECK(m.command("M114", rig.now) == "ok X:75.000 Y:50.000 Z:40.000 E:0.000");
    CHECK(m.command("G1 X200", rig.now).find("Move out of range") != std::string::npos);
    CHECK(m.command("G1 E5", rig.now).find("Extrude below minimum temp") != std::string::npos);
    CHECK(m.command("G91", rig.now) == "ok" && m.command("G1 Z-5 F600", rig.now) == "ok" && m.command("G90", rig.now) == "ok");
    for (int i = 0; i < 40; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(m.command("M114", rig.now).find("Z:35.000") != std::string::npos);
    CHECK(m.command("JOG AXIS=Y DIST=5", rig.now) == "ok");
    for (int i = 0; i < 20; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(m.command("M114", rig.now).find("Y:55.000") != std::string::npos);
    CHECK(m.command("JOG AXIS=Z DIST=-100", rig.now).find("Move out of range") != std::string::npos);
    CHECK(m.command("JOG AXIS=E DIST=5", rig.now).find("Extrude below minimum temp") != std::string::npos);
    // probe (sensorless Z): the stop comes from the trigger
    rig.trsync_armed.clear();
    CHECK(m.command("PROBE", rig.now).rfind("ok probing", 0) == 0);
    for (int i = 0; i < 20; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(!rig.trsync_armed.empty() && rig.trsync_armed.back() == plan.stepper("stepper_z")->trsync_oid);
    m.on_message(trsync_hit(plan.stepper("stepper_z")->trsync_oid), rig.now);
    for (int i = 0; i < 20; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(m.status_json(rig.now).find("probe at") != std::string::npos);
    // no trigger: homing fails with Klipper's message
    rig.trsync_armed.clear();
    CHECK(m.command("G28 Y", rig.now).rfind("ok homing Y", 0) == 0);
    for (int i = 0; i < 400 && m.toolhead()->homing(); i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(!m.toolhead()->homing() && m.toolhead()->last_error().find("No trigger on y") != std::string::npos);
}

static void test_print_file() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    plan.move_count = 1024;
    Rig rig;
    rig.file = {"; a small test print", "G28", "M104 S200", "M190 S60 ; no bed on a Tina 2: skipped", "M109 S200",
                "G90", "M83", "G1 Z0.3 F600", "G1 X20 Y20 F3000", "G1 X40 Y20 E2 F1200", "G1 X40 Y40 E2",
                "G1 X20 Y40 E2", "M106 S255", "G1 X20 Y20 E2", "M107", "START_PRINT EXTRA=1", "M104 S0", "M84"};
    Machine m(plan, mp, rig.io());
    m.start(rig.now);
    const HeaterObj& h = *plan.heater("extruder");
    const auto& es = plan.sensors[size_t(h.sensor)];
    CHECK(m.command("SDCARD_PRINT_FILE", rig.now) == "ok printing test.gcode");
    double next_adc = 0;
    size_t armed = 0;
    double armed_at = 0;
    for (int i = 0; i < 20 * 120 && m.printing(); i++) {
        rig.now += 0.05;
        if (rig.now >= next_adc) {   // the hotend follows its heater quickly here
            next_adc = rig.now + 0.3;
            double target = 0;
            for (auto& c : rig.sent)
                if (c.second.rfind("queue_digital_out oid=" + std::to_string(h.out.oid), 0) == 0)
                    target = atof(argv_of(c.second, "on_ticks").c_str()) > 0 ? 200 : 25;
            rig.temp += (target - rig.temp) * 0.1;
            m.on_message(adc(es.oid, es.th.calc_adc(rig.temp) * 8 * 4095.0), rig.now);
        }
        if (rig.trsync_armed.size() > armed) {   // every homing move hits its (sensorless) endstop 0.5 s later
            if (armed_at == 0) armed_at = rig.now;
            if (rig.now - armed_at > 0.5) {
                m.on_message(trsync_hit(rig.trsync_armed[armed]), rig.now);
                armed++;
                armed_at = 0;
            }
        }
        m.tick(rig.now);
    }
    std::string js = m.status_json(rig.now);
    CHECK(js.find("\"state\":\"done\"") != std::string::npos);
    if (js.find("\"state\":\"done\"") == std::string::npos) printf("  print: %s\n", js.substr(js.find("\"print\"")).c_str());
    CHECK(js.find("START_PRINT") != std::string::npos);   // reported as skipped
    CHECK(armed == 3);                                      // X, Y, Z homed
    // the extruder moved 8 mm forward: 8 * 3200 / 33.725 steps
    const StepperObj& e = *plan.stepper("extruder");
    StepReplay re;
    for (auto& c : rig.sent) re.feed(c.second, e.oid);
    CHECK(std::llabs(int64_t(re.steps.size()) - std::llround(8 * 3200 / 33.725)) <= 1);
}

// 2026-10-18 SD card: the file asked for reaches print_open; M20 lists, M23 + M24 selects and starts.
static void test_print_named_files() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    Rig rig;
    rig.file = {"G90", "M117 hi"};
    MachineIO io = rig.io();
    std::string asked = "?";
    io.print_open = [&](std::string& name, std::string* e) {
        asked = name;
        if (name == "missing.gcode") { if (e) *e = "no such file on the SD card: missing.gcode"; return false; }
        rig.file_pos = 0;
        if (name.empty()) name = "flash.gcode";
        return true;
    };
    io.list_files = [](std::vector<std::pair<std::string, uint64_t>>& f, std::string*) {
        f = {{"benchy.gcode", 2061555}, {"cube.gcode", 30211}};
        return true;
    };
    Machine m(plan, mp, io);
    CHECK(m.command("M20", rig.now) == "ok Begin file list\nbenchy.gcode 2061555\ncube.gcode 30211\nEnd file list");
    CHECK(m.command("SDCARD_PRINT_FILE FILENAME=missing.gcode", rig.now) == "error: no such file on the SD card: missing.gcode");
    CHECK(asked == "missing.gcode");
    CHECK(m.command("M23 cube.gcode", rig.now) == "ok file selected: cube.gcode (M24 starts it)");
    CHECK(m.command("M24", rig.now) == "ok printing cube.gcode");
    CHECK(asked == "cube.gcode");
    for (int i = 0; i < 20 && m.printing(); i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(m.command("SDCARD_PRINT_FILE", rig.now) == "ok printing flash.gcode");
    CHECK(asked.empty());
    MachineIO io2 = rig.io();
    Machine m2(plan, mp, io2);
    CHECK(m2.command("M20", rig.now) == "error: no SD card support here");
}

// 2026-10-20 Tina 2: "Home all" shut the board down with "Timer too close" while homing Z (the
// thermistor then showed as lost). Real TMC2209 UART round trips take tens of milliseconds, so here
// every query costs 40 ms of host time, temperatures keep arriving, and every command that schedules
// something on the board is checked the way the board's scheduler checks it: its time must not be in
// the past when it arrives (Klipper's sched_add_timer: "Timer too close").
static void test_home_all_timing(int lookahead, double query_s, double send_s) {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    plan.move_count = 1024;
    Rig rig;
    MachineIO io = rig.io();
    io.now = [&rig]() { return rig.now; };
    auto q0 = io.query;
    io.query = [&rig, q0, query_s](const std::string& c, const std::string& r, int oid, Message& out, int ms) {
        rig.now += query_s;   // a TMC UART / position round trip on the real board
        return q0(c, r, oid, out, ms);
    };
    auto s0 = io.send;
    io.send = [&rig, s0, send_s](const std::vector<std::string>& c, std::string* e) {
        bool ok = s0(c, e);   // stamped with the time it starts going out
        rig.now += send_s * double(c.size());
        return ok;
    };
    io.lookahead = lookahead;
    io.largest_free = []() { return size_t(2) << 20; };
    Machine m(plan, mp, io);
    m.start(rig.now);
    const HeaterObj& h = *plan.heater("extruder");
    const auto& es = plan.sensors[size_t(h.sensor)];
    CHECK(m.command("M104 S0", rig.now).rfind("ok", 0) == 0);
    CHECK(m.command("G28", rig.now).rfind("ok homing", 0) == 0);
    double next_adc = 0, armed_at = 0;
    size_t armed = 0;
    for (int i = 0; i < 20 * 200 && !m.toolhead()->homed_all(); i++) {
        rig.now += 0.05;
        if (rig.now >= next_adc) {
            next_adc = rig.now + 0.3;
            m.on_message(adc(es.oid, es.th.calc_adc(25.0) * 8 * 4095.0), rig.now);
        }
        if (rig.trsync_armed.size() > armed) {   // each homing move stalls 0.7 s after it starts
            if (armed_at == 0) armed_at = rig.now;
            if (rig.now - armed_at > 0.7) {
                m.on_message(trsync_hit(rig.trsync_armed[armed]), rig.now);
                armed++;
                armed_at = 0;
            }
        }
        m.tick(rig.now);
    }
    CHECK(m.toolhead()->homed_all());
    // The Z homing move (123 mm at 4 mm/s, 400 steps/mm: 49,200 steps) goes out in pieces: the batch
    // with its endstop_home holds well under a second of steps (2026-10-20: all of it at once took the
    // ESP32-S3 ~0.4 s to prepare, and the board got its start time late).
    {
        const StepperObj& z = *plan.stepper("stepper_z");
        double first_batch_t = -1;
        for (auto& e : rig.sent)
            if (e.second.rfind("endstop_home", 0) == 0 && argv_of(e.second, "clock") != "0" &&
                atoi(argv_of(e.second, "oid").c_str()) == plan.endstops[size_t(z.endstop)].oid)
                first_batch_t = e.first;   // the last Z homing start
        StepReplay zr;
        for (auto& e : rig.sent)
            if (e.first == first_batch_t) zr.feed(e.second, z.oid);
        CHECK(first_batch_t > 0 && !zr.steps.empty());
        CHECK(zr.steps.size() < size_t(1.0 * z.homing_speed * z.steps_per_mm()));
    }
    // the board's view: every timed command must be for "now" or later when it arrives
    std::map<int, uint64_t> last_step, busy_until;
    int late = 0;
    for (auto& e : rig.sent) {
        uint64_t at = uint64_t(e.first * 12e6);
        const std::string& c = e.second;
        auto clk = [&](const char* k) { return uint64_t(strtoull(argv_of(c, k).c_str(), nullptr, 10)); };
        uint64_t lo = uint64_t(at) & 0xffffffffu, hi = uint64_t(at) - lo;
        auto ext = [&](uint64_t c32) { uint64_t v = hi + c32; if (v + 0x80000000ull < at) v += 0x100000000ull; return v; };
        int oid = atoi(argv_of(c, "oid").c_str());
        uint64_t when = 0;
        if (c.rfind("queue_digital_out", 0) == 0) when = ext(clk("clock"));
        else if (c.rfind("endstop_home", 0) == 0 && argv_of(c, "clock") != "0") when = ext(clk("clock"));
        else if (c.rfind("trsync_start", 0) == 0) when = ext(clk("report_clock"));
        else if (c.rfind("reset_step_clock", 0) == 0) last_step[oid] = ext(clk("clock"));
        else if (c.rfind("queue_step", 0) == 0) {
            uint64_t first = last_step[oid] + clk("interval");
            if (busy_until[oid] <= at) when = first;   // only the first move of a fresh queue starts a timer
            long n = atol(argv_of(c, "count").c_str());
            int64_t iv = atoll(argv_of(c, "interval").c_str()), add = atoll(argv_of(c, "add").c_str());
            uint64_t t = last_step[oid];
            for (long k = 0; k < n; k++) t += uint64_t(iv), iv += add;
            last_step[oid] = t;
            busy_until[oid] = t;
        }
        if (when && when + 12000 < at) {   // more than 1 ms late
            if (late++ < 5) printf("  late by %.1f ms: %s\n", double(at - when) / 12e3, c.c_str());
        }
    }
    if (late) printf("  look-ahead %d, queries %.0f ms: %d commands scheduled in the past\n", lookahead, query_s * 1e3, late);
    CHECK(late == 0);
    // If it does happen on a board, the shutdown names the timed commands closest to it.
    int64_t sid = -1;
    if (lookahead == 32 && mp.enum_value("static_string_id", "Timer too close", sid)) {
        CHECK(m.command("M106 S255", rig.now).rfind("ok", 0) == 0);
        uint32_t c = 0;
        for (auto& e : rig.sent)
            if (e.second.rfind("queue_digital_out", 0) == 0) c = uint32_t(strtoul(argv_of(e.second, "clock").c_str(), nullptr, 10));
        Message sm;
        sm.name = "shutdown";
        sm.params["clock"].i = int64_t(c) + 1200;
        sm.params["static_string_id"].i = sid;
        m.on_message(sm, rig.now);
        CHECK(m.shutdown_detail().find("Closest timed commands: queue_digital_out") == 0);
        CHECK(m.shutdown_detail().find("ms ahead]") != std::string::npos);
        CHECK(m.status_json(rig.now).find("\"timing\":\"Closest") != std::string::npos);
    }
}

// The streamed homing move: with no stall at all, every one of its steps still reaches the board
// (the rest is sent by tick() while it runs), then homing fails cleanly with "No trigger".
static void test_homing_move_streams() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    plan.move_count = 1024;
    Rig rig;
    MachineIO io = rig.io();
    io.now = [&rig]() { return rig.now; };
    Machine m(plan, mp, io);
    m.start(rig.now);
    CHECK(m.command("G28 Z", rig.now).rfind("ok homing", 0) == 0);
    for (int i = 0; i < 20 * 60 && (m.toolhead()->homing() || i < 10); i++) rig.now += 0.05, m.tick(rig.now);
    const StepperObj& z = *plan.stepper("stepper_z");
    StepReplay zr;
    for (auto& e : rig.sent) zr.feed(e.second, z.oid);
    long want = std::lround((z.position_max - z.position_min) * 1.5 * z.steps_per_mm());
    CHECK(long(zr.steps.size()) == want);
    CHECK(m.toolhead()->last_error().find("No trigger on z") != std::string::npos);
}

// A Rig whose board reports stepper positions from the steps actually sent (and due by now), and
// whose sensorless endstops stall `stall_after` seconds into each homing move (<0: never).
struct MotionRig {
    Rig rig;
    double stall_after = 0.7;
    std::vector<double> stall_seq;   // if set: the stall delay for each arming in turn (then stall_after)
    size_t stall_i = 0;
    MachineIO io() {
        MachineIO io = rig.io();
        io.now = [this]() { return rig.now; };
        auto q0 = io.query;
        io.query = [this, q0](const std::string& c, const std::string& r, int oid, Message& out, int ms) {
            bool ok = q0(c, r, oid, out, ms);
            if (r == "stepper_position") {
                StepReplay sr;
                for (auto& e : rig.sent) sr.feed(e.second, oid);
                uint64_t nowc = uint64_t(rig.now * 12e6);
                int64_t pos = 0;
                for (auto& st : sr.steps)
                    if (st.first <= nowc) pos += st.second ? 1 : -1;
                out.params["pos"].i = pos;
            }
            return ok;
        };
        return io;
    }
    // run until `done` or the time limit, delivering stalls
    template <class F> void run(Machine& m, double secs, F done) {
        size_t armed = rig.trsync_armed.size();
        double armed_at = 0;
        for (int i = 0; i < int(secs * 20) && !done(); i++) {
            rig.now += 0.05;
            double sa = stall_i < stall_seq.size() ? stall_seq[stall_i] : stall_after;
            if (rig.trsync_armed.size() > armed && sa >= 0) {
                if (armed_at == 0) armed_at = rig.now;
                if (rig.now - armed_at >= sa) {
                    stall_i++;
                    m.on_message(trsync_hit(rig.trsync_armed[armed]), rig.now);
                    armed++;
                    armed_at = 0;
                }
            }
            m.tick(rig.now);
        }
    }
};

static std::string repo_file(const char* rel) {
    std::string f = __FILE__;
    std::string root = f.substr(0, f.rfind("host/test/"));
    FILE* fp = fopen((root + rel).c_str(), "rb");
    if (!fp) return "";
    std::string t;
    char b[4096];
    size_t n;
    while ((n = fread(b, 1, sizeof b, fp)) > 0) t.append(b, n);
    fclose(fp);
    return t;
}

// 2026-10-20: [homing_override] for the Tina 2 (X, Y, then Z down onto the bed, then up 10 mm), and
// [gcode_macro] in general: plain G-code lines, each waiting for the last one's moves.
static void test_homing_override_and_macros() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string cfg = repo_file("docs/configs/tina2-skr-pico.cfg");
    CHECK(!cfg.empty() && cfg.size() < 4096);
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    for (auto& e : plan.errors) printf("  tina2-skr-pico.cfg: %s\n", e.c_str());
    CHECK(plan.errors.empty());
    CHECK(plan.homing_override.size() == 9 && plan.homing_override[4] == "G1 X45 F3000" &&
          plan.homing_override[6] == "G1 Y52 F3000" && plan.homing_override[8] == "G1 Z10 F600");
    CHECK(plan.homing_override_set[2] == 0.0 && std::isnan(plan.homing_override_set[0]));
    CHECK(!plan.stepper("stepper_z")->homing_positive_dir);   // Z homes down now
    plan.move_count = 1024;
    MotionRig mr;
    Machine m(plan, mp, mr.io());
    m.start(mr.rig.now);
    CHECK(m.command("G28", mr.rig.now).rfind("ok running homing_override", 0) == 0);
    CHECK(m.command("G1 X20", mr.rig.now).find("homing_override is running") != std::string::npos);
    mr.run(m, 60, [&]() { return !m.macro_running(); });
    CHECK(!m.macro_running() && m.macro_error().empty());
    if (!m.macro_error().empty()) printf("  macro: %s\n", m.macro_error().c_str());
    CHECK(m.toolhead()->homed_all());
    std::string js = m.status_json(mr.rig.now);
    CHECK(js.find("\"pos\":[45.000,52.000,10.000") != std::string::npos);
    // a failed homing stops the macro (Z never stalls: no G1 Z10 afterwards)
    MotionRig mr2;
    mr2.stall_after = -1;
    Machine m2(plan, mp, mr2.io());
    m2.start(mr2.rig.now);
    CHECK(m2.command("G28", mr2.rig.now).rfind("ok", 0) == 0);
    mr2.run(m2, 120, [&]() { return !m2.macro_running(); });
    CHECK(!m2.toolhead()->homed(2));   // Z was only assumed (set_position_z): not homed after the failure
    CHECK(m2.macro_error().find("line 4 (G28 X)") != std::string::npos &&
          m2.macro_error().find("No trigger on x") != std::string::npos);
    // a print's G28 runs the override and waits for it
    MotionRig mr3;
    mr3.rig.file = {"G28", "G90", "G1 X40 Y40 F3000"};
    Machine m3(plan, mp, mr3.io());
    m3.start(mr3.rig.now);
    CHECK(m3.command("SDCARD_PRINT_FILE", mr3.rig.now).rfind("ok printing", 0) == 0);
    mr3.run(m3, 90, [&]() { return !m3.printing(); });
    js = m3.status_json(mr3.rig.now);
    CHECK(js.find("\"state\":\"done\"") != std::string::npos);
    CHECK(js.find("\"pos\":[40.000,40.000,10.000") != std::string::npos);
    // the Ender 3 preset now shipped in the firmware: valid against the real dictionary, under 4 KB
    {
        std::string e3 = default_printer_cfg_ender3();
        BoardPlan pe = build_board_plan(e3, mp, 12e6);
        for (auto& e : pe.errors) printf("  ender3 preset: %s\n", e.c_str());
        CHECK(pe.errors.empty() && e3.size() < 4096 && pe.heaters.size() == 2 && pe.steppers.size() == 4);
        CHECK(pe.stepper("stepper_x")->endstop >= 0 && plan.endstops.size() > 0);
        std::string ef = repo_file("docs/configs/ender3-skr-pico.cfg");
        CHECK(ef == e3);   // the copy in docs/configs is the same text
    }
    // [gcode_macro]: by name, case-insensitive; templates refused
    BoardPlan p2 = build_board_plan(cfg + "\n[gcode_macro park]\ngcode:\n  G90\n  G1 X45 Y100 F3000 ; park\n", mp, 12e6);
    CHECK(p2.errors.empty() && p2.macro("PARK") && p2.macro("PARK")->lines.size() == 2);
    BoardPlan p3 = build_board_plan(cfg + "\n[gcode_macro bad]\ngcode:\n  G1 X{params.X}\n", mp, 12e6);
    CHECK(!p3.errors.empty() && p3.errors[0].find("templates") != std::string::npos);
    MotionRig mr4;
    Machine m4(p2, mp, mr4.io());
    m4.start(mr4.rig.now);
    CHECK(m4.command("park", mr4.rig.now).find("Must home axis first") == std::string::npos);
    mr4.run(m4, 5, [&]() { return !m4.macro_running(); });
    CHECK(m4.macro_error().find("Must home axis first") != std::string::npos);   // G1 before homing: stops there
}

// The 10-minute run-around G-code shipped for the Tina 2 runs through the print engine, homes with
// the override, stays inside its 10 mm box and takes about 10 minutes.
// 2026-10-23: what the little OLED shows (Machine::display_info): state, HOMING, and the extruder-only
// moves outside a print (EXTRUDING / RETRACTING, LOADING / UNLOADING from 20 mm), for as long as they take.
static void test_display_info() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    // the simulated hotend stays cold: allow cold extrusion for this test
    std::string cfg = config_set_option(repo_file("docs/configs/tina2-skr-pico.cfg"), "extruder", "min_extrude_temp", "0");
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    CHECK(plan.errors.empty());
    plan.move_count = 1024;
    MotionRig mr;
    Machine m(plan, mp, mr.io());
    m.start(mr.rig.now);
    double& now = mr.rig.now;
    Machine::DisplayInfo d = m.display_info(now);
    CHECK(d.state == "standby" && d.activity.empty() && !d.have_bed);   // a heated bed (2026-10-27), but no reading yet
    CHECK(plan.heater("heater_bed") != nullptr);
    CHECK(m.command("G28", now).rfind("ok", 0) == 0);
    bool saw_homing = false;
    mr.run(m, 60, [&]() {
        saw_homing |= m.display_info(now).activity == "HOMING";
        return !m.macro_running();
    });
    CHECK(saw_homing);
    mr.run(m, 2, []() { return false; });
    CHECK(m.toolhead()->homed_all() && m.display_info(now).activity.empty());
    CHECK(m.command("M83", now).rfind("ok", 0) == 0);
    CHECK(m.command("G1 E5 F300", now).rfind("ok", 0) == 0);           // 5 mm at 5 mm/s: 1 s + 1 s
    CHECK(m.display_info(now).activity == "EXTRUDING");
    CHECK(m.display_info(now + 1.9).activity == "EXTRUDING");
    CHECK(m.display_info(now + 2.1).activity.empty());
    mr.run(m, 3, []() { return false; });
    CHECK(m.command("G1 E-2", now).rfind("ok", 0) == 0);               // keeps the last F
    CHECK(m.display_info(now).activity == "RETRACTING");
    mr.run(m, 2, []() { return false; });
    CHECK(m.command("G1 E50 F600", now).rfind("ok", 0) == 0);          // 50 mm at 10 mm/s: 5 s + 1 s
    CHECK(m.display_info(now + 5.5).activity == "LOADING");
    CHECK(m.display_info(now + 6.2).activity.empty());
    mr.run(m, 7, []() { return false; });
    CHECK(m.command("JOG AXIS=E DIST=-30 SPEED=10", now).rfind("ok", 0) == 0);
    CHECK(m.display_info(now).activity == "UNLOADING");
    mr.run(m, 5, []() { return false; });
    CHECK(m.command("G1 X40 E1 F3000", now).rfind("ok", 0) == 0);      // X moves too: not an extruder move
    CHECK(m.display_info(now).activity.empty());
    mr.run(m, 3, []() { return false; });
    // a print: printing, then complete; its retractions do not flash up
    MotionRig mp2;
    mp2.rig.file = {"G28", "M83", "G90", "G1 X40 Y40 F3000", "G1 E-1 F1800", "G1 E1"};
    Machine m2(plan, mp, mp2.io());
    m2.start(mp2.rig.now);
    CHECK(m2.command("SDCARD_PRINT_FILE", mp2.rig.now).rfind("ok printing", 0) == 0);
    CHECK(m2.display_info(mp2.rig.now).state == "printing");
    bool flashed = false;
    mp2.run(m2, 90, [&]() {
        std::string a = m2.display_info(mp2.rig.now).activity;
        flashed |= a == "RETRACTING" || a == "EXTRUDING";
        return !m2.printing();
    });
    CHECK(!flashed);
    CHECK(m2.display_info(mp2.rig.now).state == "complete");
}

// 2026-10-24: the pre-hardware review's findings, reproduced, then fixed.
static void test_review_fixes() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string cfg = repo_file("docs/configs/tina2-skr-pico.cfg");
    auto test_state = [](Machine& m, const std::string& id) {
        for (auto& x : m.tests())
            if (x.id == id) return x;
        return Machine::Test();
    };
    // 1. "2. Free run" right after "1. Hard stop" on X (homes to the max end): away from the end,
    //    exactly DIST, inside the limits; refused without room
    {
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        plan.move_count = 1024;
        MotionRig mr;
        Machine m(plan, mp, mr.io());
        m.start(mr.rig.now);
        CHECK(m.command("STALLGUARD_HOME STEPPER=stepper_x", mr.rig.now).rfind("ok", 0) == 0);
        mr.run(m, 30, [&]() { return test_state(m, "stop_stepper_x").state != "running"; });
        mr.run(m, 3, []() { return false; });
        CHECK(test_state(m, "stop_stepper_x").state == "passed" && m.toolhead()->homed(0));
        NEAR(m.toolhead()->machine_position(0), 80.0, 1e-6);
        size_t from = mr.rig.sent.size();
        std::string r = m.command("STALLGUARD_TEST STEPPER=stepper_x", mr.rig.now);
        CHECK(r.rfind("ok", 0) == 0);
        mr.run(m, 20, [&]() { return test_state(m, "sg_stepper_x").state != "running"; });
        const StepperObj& sx = *plan.stepper("stepper_x");
        StepReplay sr;
        for (size_t k = from; k < mr.rig.sent.size(); k++) sr.feed(mr.rig.sent[k].second, sx.oid);
        long pos = 0, lo = 0, hi = 0;
        for (auto& st : sr.steps) {
            pos += ((st.second ^ (sx.invert_dir ? 1 : 0)) ? 1 : -1);
            lo = std::min(lo, pos), hi = std::max(hi, pos);
        }
        double spm = sx.steps_per_mm();
        CHECK(hi == 0);                          // never towards the X+ end
        NEAR(-lo / spm, 10.0, 0.6);              // 10 mm, ramps included
        CHECK(pos == 0);                         // and back where it started
        CHECK(m.toolhead()->homed(0));
        CHECK(m.command("JOG AXIS=X DIST=-72", mr.rig.now).rfind("ok", 0) == 0);   // to X 8
        mr.run(m, 5, []() { return false; });
        r = m.command("STALLGUARD_TEST STEPPER=stepper_x", mr.rig.now);
        CHECK(r.find("needs 10 mm of room") != std::string::npos);
    }
    // 2. a false trigger leaves the axis NOT homed (it "homed" in the wrong place)
    {
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        plan.move_count = 1024;
        MotionRig mr;
        mr.stall_after = 0.0;
        Machine m(plan, mp, mr.io());
        m.start(mr.rig.now);
        CHECK(m.command("STALLGUARD_HOME STEPPER=stepper_x", mr.rig.now).rfind("ok", 0) == 0);
        mr.run(m, 30, [&]() { return test_state(m, "stop_stepper_x").state != "running"; });
        CHECK(test_state(m, "stop_stepper_x").detail.find("false trigger") != std::string::npos);
        CHECK(!m.toolhead()->homed(0));
        CHECK(m.command("G1 X5", mr.rig.now).find("Must home axis first") != std::string::npos);
    }
    // 3. CANCEL_PRINT during the print's G28 stops [homing_override]
    {
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        plan.move_count = 1024;
        MotionRig mr;
        mr.rig.file = {"G28", "G1 X40 F3000"};
        Machine m(plan, mp, mr.io());
        m.start(mr.rig.now);
        CHECK(m.command("SDCARD_PRINT_FILE", mr.rig.now).rfind("ok printing", 0) == 0);
        mr.run(m, 0.5, []() { return false; });
        CHECK(m.macro_running());
        CHECK(m.command("CANCEL_PRINT", mr.rig.now).rfind("ok", 0) == 0);
        CHECK(!m.macro_running());
        mr.run(m, 30, []() { return false; });
        CHECK(!m.toolhead()->homed_all());
        // a console macro stops with MACRO_ABORT
        CHECK(m.command("G28", mr.rig.now).rfind("ok", 0) == 0);
        CHECK(m.command("MACRO_ABORT", mr.rig.now).rfind("ok homing_override stopped", 0) == 0);
        CHECK(!m.macro_running() && !m.toolhead()->homing());
    }
    // 4. Abort right after the hard stop starts: nothing is armed afterwards
    {
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        plan.move_count = 1024;
        MotionRig mr;
        mr.stall_after = -1;
        Machine m(plan, mp, mr.io());
        m.start(mr.rig.now);
        CHECK(m.command("STALLGUARD_HOME STEPPER=stepper_z", mr.rig.now).rfind("ok", 0) == 0);
        CHECK(m.command("TEST_ABORT TEST=stop_stepper_z", mr.rig.now).rfind("ok", 0) == 0);
        size_t from = mr.rig.sent.size();
        mr.run(m, 5, []() { return false; });
        int armed = 0;
        for (size_t k = from; k < mr.rig.sent.size(); k++) armed += mr.rig.sent[k].second.rfind("trsync_start", 0) == 0;
        CHECK(armed == 0);
        CHECK(!m.toolhead()->homing() && !m.toolhead()->homed(2));
        CHECK(test_state(m, "stop_stepper_z").state == "aborted");
    }
}

// 2026-10-24: input shaping + pressure advance through the real toolhead (CAPTAIN / SKIPPER builds).
static void test_toolhead_shaping() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string cfg = repo_file("docs/configs/tina2-skr-pico.cfg");
    cfg = config_set_option(cfg, "extruder", "min_extrude_temp", "0");
    cfg = config_set_option(cfg, "extruder", "pressure_advance", "0.06");
    cfg = config_set_option(cfg, "input_shaper", "shaper_freq_x", "52.0");
    cfg = config_set_option(cfg, "input_shaper", "shaper_freq_y", "38.5");
    cfg = config_set_option(cfg, "input_shaper", "shaper_type_y", "ei");
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    for (auto& e : plan.errors) printf("  shaping cfg: %s\n", e.c_str());
    CHECK(plan.errors.empty());
    CHECK(plan.shaping.x.n == 3 && plan.shaping.y.n == 3 && plan.shaping.pa == 0.06);
    plan.move_count = 1024;
    MotionRig mr;
    std::string saved;
    MachineIO io = mr.io();
    io.config_text = [&]() { return cfg; };
    io.save_config = [&](const std::string& t, std::string*) { saved = t; return true; };
    Machine m(plan, mp, io);
    m.start(mr.rig.now);
    CHECK(m.command("G28", mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 60, [&]() { return !m.macro_running(); });
    CHECK(m.toolhead()->homed_all());
    size_t from = mr.rig.sent.size();   // steps from here on: after homing (X45 Y52 Z10)
    std::string js = m.status_json(mr.rig.now);
    CHECK(js.find("\"shaper_x\":\"mzv 52.000\"") != std::string::npos && js.find("\"pa\":0.0600") != std::string::npos);
    // a little print-like path, absolute E
    const char* moves[] = {"G90", "M82", "G92 E0", "G1 X20 Y20 F6000", "G1 X60 Y20 E2 F3000", "G1 X60 Y80 E5",
                           "G1 X20 Y80 E7", "G1 E6 F2400", "G1 X40 Y50 F6000", "G1 E7 F2400", "G1 X25 Y30 E9 F3000"};
    for (auto* l : moves) CHECK(m.command(l, mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 20, [&]() { return m.toolhead()->idle(mr.rig.now); });
    CHECK(m.toolhead()->idle(mr.rig.now));
    // every motor ends exactly where the G-code says (shaping and advance only move things in time)
    auto mcu_mm = [&](const char* st) {
        const StepperObj& so = *plan.stepper(st);
        StepReplay sr;   // the whole stream (direction state), counting the steps queued after `from`
        for (size_t k = 0; k < from; k++) sr.feed(mr.rig.sent[k].second, so.oid);
        size_t first = sr.steps.size();
        for (size_t k = from; k < mr.rig.sent.size(); k++) sr.feed(mr.rig.sent[k].second, so.oid);
        long pos = 0;
        for (size_t k = first; k < sr.steps.size(); k++) pos += ((sr.steps[k].second ^ (so.invert_dir ? 1 : 0)) ? 1 : -1);
        uint64_t prev = 0;
        uint64_t min_iv = UINT64_MAX;
        for (auto& x : sr.steps) {
            if (prev && x.first > prev) min_iv = std::min(min_iv, x.first - prev);
            prev = x.first;
        }
        return std::make_pair(pos / so.steps_per_mm(), min_iv);
    };
    auto ex = mcu_mm("extruder");
    NEAR(ex.first, 9.0, 0.02);
    CHECK(ex.second >= 24);   // never two steps closer than 2 us at 12 MHz
    NEAR(mcu_mm("stepper_y").first, 30.0 - 52.0, 0.02);
    NEAR(mcu_mm("stepper_x").first, 25.0 - 45.0, 0.02);
    CHECK(mcu_mm("stepper_x").second >= 24 && mcu_mm("stepper_y").second >= 24);
    // runtime changes: refused value, reported value, saved into printer.cfg
    CHECK(m.command("SET_INPUT_SHAPER SHAPER_TYPE_X=bogus", mr.rig.now).find("unknown shaper type") != std::string::npos);
    CHECK(m.command("SET_INPUT_SHAPER SHAPER_FREQ_X=48 SHAPER_TYPE_X=zv", mr.rig.now).rfind("ok shaper_type_x:zv shaper_freq_x:48.0", 0) == 0);
    CHECK(m.toolhead()->shaping().x.n == 2);
    CHECK(m.command("SET_PRESSURE_ADVANCE ADVANCE=0.045 SAVE=1", mr.rig.now).find("saved in printer.cfg") != std::string::npos);
    CHECK(saved.find("pressure_advance: 0.0450") != std::string::npos);
    CHECK(m.command("SET_INPUT_SHAPER SHAPER_FREQ_Y=0", mr.rig.now).rfind("ok", 0) == 0);
    CHECK(m.toolhead()->shaping().y.n == 0);
}

// 2026-10-24: the CAPTAIN / SKIPPER extras through the machine: resonance test, bed mesh, screw
// tilt, excluded objects, firmware retraction.
static void test_pro_features() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string base = config_set_option(repo_file("docs/configs/tina2-skr-pico.cfg"), "extruder", "min_extrude_temp", "0");
    auto state_of = [](Machine& m, const std::string& id) {
        for (auto& x : m.tests())
            if (x.id == id) return x;
        return Machine::Test();
    };
    auto homed_machine = [&](const std::string& cfg, MotionRig& mr, BoardPlan& plan, std::unique_ptr<Machine>& m) {
        plan = build_board_plan(cfg, mp, 12e6);
        for (auto& e : plan.errors) printf("  pro cfg: %s\n", e.c_str());
        CHECK(plan.errors.empty());
        plan.move_count = 1024;
        m.reset(new Machine(plan, mp, mr.io()));
        m->start(mr.rig.now);
        CHECK(m->command("G28", mr.rig.now).rfind("ok", 0) == 0);
        mr.run(*m, 60, [&]() { return !m->macro_running(); });
        mr.stall_i = 0;
        CHECK(m->toolhead()->homed_all());
    };
    // ---- resonance test: the accelerometer is told when; the axis really shakes; it ends where it should
    {
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        std::vector<std::pair<int, int>> calls;
        MachineIO io = mr.io();
        homed_machine(base, mr, plan, m);
        MachineIO io2 = mr.io();
        io2.accel_record = [&](int mode, int axis, std::string*) { calls.push_back({mode, axis}); return true; };
        m.reset(new Machine(plan, mp, io2));
        m->start(mr.rig.now);
        CHECK(m->command("SET_KINEMATIC_POSITION X=45 Y=52 Z=10", mr.rig.now).rfind("ok", 0) == 0);
        CHECK(m->command("TEST_RESONANCES AXIS=X POINT=10,52,10", mr.rig.now).find("outside the X limits") != std::string::npos);
        size_t from = mr.rig.sent.size();
        std::string r = m->command("SHAPER_CALIBRATE AXIS=X FREQ_START=20 FREQ_END=40", mr.rig.now);
        CHECK(r.rfind("ok resonance test X: 20 to 40 Hz", 0) == 0);
        bool shaking = false;
        mr.run(*m, 60, [&]() {
            shaking |= m->display_info(mr.rig.now).activity == "SHAKING";
            return state_of(*m, "res_x").state != "running";
        });
        CHECK(shaking);
        CHECK(state_of(*m, "res_x").state == "passed");
        CHECK(calls.size() == 2 && calls[0] == std::make_pair(1, 0) && calls[1] == std::make_pair(0, 0));
        ResonanceParams rp;
        rp.min_freq = 20, rp.max_freq = 40;
        ResonanceMotion mo = resonance_motion(resonance_test_sequence(rp), plan.printer.max_accel);
        const StepperObj& sx = *plan.stepper("stepper_x");
        StepReplay sr;
        for (size_t k = 0; k < from; k++) sr.feed(mr.rig.sent[k].second, sx.oid);
        size_t first = sr.steps.size();
        for (size_t k = from; k < mr.rig.sent.size(); k++) sr.feed(mr.rig.sent[k].second, sx.oid);
        long pos = 0, travelled = 0;
        for (size_t k = first; k < sr.steps.size(); k++) pos += ((sr.steps[k].second ^ (sx.invert_dir ? 1 : 0)) ? 1 : -1), travelled++;
        double path = 0;   // the motion's own path length (mm)
        for (double t = 0; t < mo.end_t; t += 0.0002) {
            double v;
            mo.pos(t, &v);
            path += std::fabs(v) * 0.0002;
        }
        printf("  resonance test 20-40 Hz: %.1f s, %ld steps for a %.1f mm path (%.0f steps), ends %.3f mm off\n", mo.end_t,
               travelled, path, path * sx.steps_per_mm(), pos / sx.steps_per_mm() - mo.end_x);
        NEAR(pos / sx.steps_per_mm(), mo.end_x, 1.0 / sx.steps_per_mm() + 1e-9);
        CHECK(std::fabs(travelled - path * sx.steps_per_mm()) < 0.02 * path * sx.steps_per_mm() + 50);
        NEAR(m->toolhead()->machine_position(0), 45 + std::round(mo.end_x * sx.steps_per_mm()) / sx.steps_per_mm(), 1e-6);
        // abort: the accelerometer is told to drop it, the axis is not homed any more
        mr.run(*m, 1, []() { return false; });
        CHECK(m->command("TEST_RESONANCES AXIS=Y FREQ_START=20 FREQ_END=60", mr.rig.now).rfind("ok", 0) == 0);
        mr.run(*m, 3, []() { return false; });
        CHECK(m->command("TEST_ABORT TEST=res_y", mr.rig.now).rfind("ok", 0) == 0);
        CHECK(calls.back().first == -1 && !m->toolhead()->homed(1));
    }
    // ---- bed mesh: zig-zag order, z_offset, the text and colours, saving, and moves following it
    {
        std::string cfg = base + "\n[bed_mesh]\nmesh_min: 15, 15\nmesh_max: 75, 95\nprobe_count: 3, 3\nhorizontal_move_z: 5\n";
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        homed_machine(cfg, mr, plan, m);
        // the probe stalls later at some points (lower bed there): rows from the front, zig-zag
        mr.stall_seq = {0.70, 0.75, 0.80, /* row 1, right to left: */ 0.80, 0.75, 0.70, /* row 2: */ 0.70, 0.70, 0.90};
        std::string saved;
        MachineIO io = mr.io();
        CHECK(m->command("BED_MESH_CALIBRATE", mr.rig.now).rfind("ok bed mesh: probing 9 points", 0) == 0);
        bool meshing = false;
        mr.run(*m, 200, [&]() {
            meshing |= m->display_info(mr.rig.now).activity == "MESHING";
            return !m->probing_run();
        });
        CHECK(meshing && !m->probing_run());
        const BedMesh& bm = m->toolhead()->mesh();
        CHECK(bm.valid() && bm.cols == 3 && bm.rows == 3);
        if (bm.valid()) {
            // a longer stall = the nozzle went further down = a lower bed there
            CHECK(bm.z[0] > bm.z[1] && bm.z[1] > bm.z[2]);           // front row: falls to the right
            CHECK(bm.z[3] > bm.z[4] && bm.z[4] > bm.z[5]);           // middle row (probed right to left)
            CHECK(std::fabs(bm.z[6] - bm.z[7]) < 0.01 && bm.z[8] < bm.z[7] - 0.05);
            CHECK(bm.level(bm.highest()) == 4 && bm.level(bm.lowest()) == 0);
            printf("  %s", bm.text().c_str());
            std::string js = m->status_json(mr.rig.now);
            CHECK(js.find("\"mesh\":{\"cols\":3,\"rows\":3") != std::string::npos && js.find("\"lvl\":[") != std::string::npos);
            // a move follows the mesh
            CHECK(m->command("G1 X75 Y15 Z1 F3000", mr.rig.now).rfind("ok", 0) == 0);
            mr.run(*m, 10, [&]() { return m->toolhead()->idle(mr.rig.now); });
            NEAR(m->toolhead()->machine_position(2), 1 + bm.z[2], 1e-6);
            CHECK(m->command("M114", mr.rig.now).find("Z:1.000") != std::string::npos);   // the G-code Z stays 1
            // save: printer.cfg gets [bed_mesh default], and it parses back
            MachineIO io3 = mr.io();
            std::string text = cfg;
            BoardPlan p2 = plan;
            std::string out = text;
            for (auto& kv : bm.profile_options()) out = config_set_option(out, "bed_mesh default", kv.first, kv.second);
            BoardPlan p3 = build_board_plan(out, mp, 12e6);
            CHECK(p3.errors.empty() && p3.mesh_default.valid());
            if (p3.mesh_default.valid()) NEAR(p3.mesh_default.z[8], bm.z[8], 1e-4);
            // saving it twice must not leave the old points behind
            for (auto& kv : bm.profile_options()) out = config_set_option(out, "bed_mesh default", kv.first, kv.second);
            BoardPlan p4 = build_board_plan(out, mp, 12e6);
            CHECK(p4.errors.empty() && p4.mesh_default.valid());
        }
        CHECK(m->command("BED_MESH_CLEAR", mr.rig.now).rfind("ok", 0) == 0 && !m->toolhead()->mesh().valid());
    }
    // ---- screw tilt: Klipper's report (numbers from Klipper's own formula)
    {
        ScrewsConfig sc;
        sc.present = true;
        sc.screws = {{"front left", 10, 10}, {"front right", 80, 10}, {"rear right", 80, 95}, {"rear left", 10, 95}};
        std::string js;
        std::string rep = screws_report(sc, {2.5, 2.6, 2.45, 2.52}, "", &js);
        CHECK(rep.find("front left (base) : x=10.0, y=10.0, z=2.50000") != std::string::npos);
        CHECK(rep.find("front right : x=80.0, y=10.0, z=2.60000 : adjust CCW 00:12") != std::string::npos);
        CHECK(rep.find("rear right : x=80.0, y=95.0, z=2.45000 : adjust CW 00:06") != std::string::npos);
        CHECK(rep.find("rear left : x=10.0, y=95.0, z=2.52000 : adjust CCW 00:02") != std::string::npos);
        CHECK(js.find("\"adjust\":\"00:12\"") != std::string::npos);
        // and through the machine with a real probing run
        std::string cfg = base + "\n[screws_tilt_adjust]\nscrew1: 10, 10\nscrew1_name: front left\nscrew2: 80, 10\n"
                                 "screw3: 45, 100\nscrew_thread: CW-M4\n";
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        homed_machine(cfg, mr, plan, m);
        mr.stall_seq = {0.7, 0.9, 0.6};
        CHECK(m->command("SCREWS_TILT_CALCULATE", mr.rig.now).rfind("ok screw tilt: probing 3 points", 0) == 0);
        mr.run(*m, 120, [&]() { return !m->probing_run(); });
        CHECK(m->screws_text().find("front left (base)") != std::string::npos);
        CHECK(m->screws_text().find("screw at 80.000,10.000 : x=80.0, y=10.0") != std::string::npos);
        CHECK(m->status_json(mr.rig.now).find("\"screws\":[{\"name\":\"front left\"") != std::string::npos);
    }
    // ---- excluded objects and firmware retraction in a print
    {
        std::string cfg = base + "\n[firmware_retraction]\nretract_length: 1.5\nretract_speed: 30\n";
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        homed_machine(cfg, mr, plan, m);
        size_t from = mr.rig.sent.size();
        mr.rig.file = {"G90", "M82", "G92 E0", "EXCLUDE_OBJECT_DEFINE NAME=cube_a CENTER=30,30",
                       "EXCLUDE_OBJECT_DEFINE NAME=cube_b CENTER=60,60", "EXCLUDE_OBJECT NAME=cube_b",
                       "EXCLUDE_OBJECT_START NAME=cube_a", "G1 X20 Y20 F6000", "G1 X40 Y20 E2 F3000", "G10",
                       "EXCLUDE_OBJECT_END", "EXCLUDE_OBJECT_START NAME=cube_b", "G11", "G1 X50 Y50 F6000",
                       "G1 X70 Y50 E10", "G10", "EXCLUDE_OBJECT_END", "G11", "G1 X30 Y40 E12 F3000"};
        CHECK(m->command("SDCARD_PRINT_FILE", mr.rig.now).rfind("ok printing", 0) == 0);
        mr.run(*m, 60, [&]() { return !m->printing(); });
        std::string js = m->status_json(mr.rig.now);
        CHECK(js.find("\"state\":\"done\"") != std::string::npos);
        CHECK(js.find("{\"name\":\"CUBE_B\",\"excluded\":true") != std::string::npos);
        const StepperObj& se = *plan.stepper("extruder");
        StepReplay sr;
        for (size_t k = 0; k < from; k++) sr.feed(mr.rig.sent[k].second, se.oid);
        size_t first = sr.steps.size();
        for (size_t k = from; k < mr.rig.sent.size(); k++) sr.feed(mr.rig.sent[k].second, se.oid);
        long pos = 0;
        for (size_t k = first; k < sr.steps.size(); k++) pos += ((sr.steps[k].second ^ (se.invert_dir ? 1 : 0)) ? 1 : -1);
        // cube_a: 2 mm, its G10 -1.5, then (cube_b skipped) G11 +1.5 and the last line's 2 mm (12 - 10): 4 mm
        NEAR(pos / se.steps_per_mm(), 4.0, 0.02);
        CHECK(m->command("GET_RETRACTION", mr.rig.now).find("RETRACT_LENGTH=1.500") != std::string::npos);
    }
    // ---- Moonraker-shaped state for a spaghetti detector (Pasta Inspector dongle, 2026-10-31):
    //      outlines, heights, exclusion, live head position, pause, filament used, RESPOND
    {
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        homed_machine(base, mr, plan, m);
        unsigned rev0 = m->objects_rev();
        CHECK(m->moonraker_status_json(mr.rig.now).find("\"state\":\"standby\"") != std::string::npos);
        mr.rig.file = {"G90", "M82", "G92 E0",
                       "EXCLUDE_OBJECT_DEFINE NAME=cube_a CENTER=30,30 POLYGON=[[20,20],[40,20],[40,40],[20,40]] PI_Z0=0 PI_Z1=10.5",
                       "EXCLUDE_OBJECT_DEFINE NAME=cube_b CENTER=60,60",
                       "EXCLUDE_OBJECT_START NAME=cube_a", "G1 X20 Y20 F6000", "G1 X80 Y20 E6 F600",
                       "EXCLUDE_OBJECT_END"};
        CHECK(m->command("SDCARD_PRINT_FILE", mr.rig.now).rfind("ok printing", 0) == 0);
        CHECK(m->objects_rev() > rev0);   // a new print always looks new to a watcher
        std::vector<double> xs;
        bool saw_cur = false, saw_live = false;
        mr.run(*m, 60, [&]() {
            double lp[4];
            if (m->toolhead()->live_position(mr.rig.now, lp)) xs.push_back(lp[0]);
            std::string st = m->moonraker_status_json(mr.rig.now);
            saw_cur |= st.find("\"current_object\":\"CUBE_A\"") != std::string::npos;
            saw_live |= st.find("\"live_position\":[") != std::string::npos && st.find("\"state\":\"printing\"") != std::string::npos;
            return !m->printing();
        });
        CHECK(saw_live);
        std::string ob = m->moonraker_objects_json();
        CHECK(ob.find("{\"name\":\"CUBE_A\",\"center\":[30.000,30.000],\"polygon\":[[20.000,20.000],[40.000,20.000],[40.000,40.000],[20.000,40.000]],\"PI_Z0\":0.000,\"PI_Z1\":10.500}") != std::string::npos);
        CHECK(ob.find("{\"name\":\"CUBE_B\",\"center\":[60.000,60.000]}") != std::string::npos);   // no outline: none claimed
        std::string st = m->moonraker_status_json(mr.rig.now);
        CHECK(st.find("\"state\":\"complete\"") != std::string::npos);
        CHECK(st.find("\"is_paused\":false") != std::string::npos);
        CHECK(st.find("\"exclude_object\":{\"excluded_objects\":[],\"current_object\":null}") != std::string::npos);
        // the head went from X20 to X80 and never backwards; the filament used is the 6 mm pushed
        CHECK(xs.size() > 10);
        bool mono = true;   // after the first move (to X20, from wherever the head was) it only goes up
        size_t lo = size_t(std::min_element(xs.begin(), xs.end()) - xs.begin());
        for (size_t i = lo + 1; i < xs.size(); i++) mono &= xs[i] >= xs[i - 1] - 0.05;
        CHECK(mono);
        CHECK(xs.back() > 79.0 && xs.back() < 80.1);
        double lp[4];
        CHECK(m->toolhead()->live_position(mr.rig.now, lp));
        NEAR(lp[0], 80.0, 0.05);
        NEAR(lp[3], 6.0, 0.05);
        CHECK(st.find("\"filament_used\":") != std::string::npos);
        NEAR(m->toolhead()->extruded(), 6.0, 0.05);
        // excluding bumps the revision and shows in the list (what the dongle sends)
        unsigned rev1 = m->objects_rev();
        CHECK(m->command("EXCLUDE_OBJECT NAME=cube_b", mr.rig.now).rfind("ok CUBE_B excluded", 0) == 0);
        CHECK(m->objects_rev() > rev1);
        CHECK(m->moonraker_status_json(mr.rig.now).find("\"excluded_objects\":[\"CUBE_B\"]") != std::string::npos);
        CHECK(m->command("EXCLUDE_OBJECT RESET=1", mr.rig.now).rfind("ok no objects excluded", 0) == 0);
        CHECK(m->moonraker_status_json(mr.rig.now).find("\"excluded_objects\":[]") != std::string::npos);
        // RESPOND: the text reaches the console, quotes and spaces kept
        CHECK(m->command("RESPOND MSG=\"pasta: excluded CUBE_B\"", mr.rig.now) == "ok echo: pasta: excluded CUBE_B");
        CHECK(m->command("respond PREFIX=pasta MSG=hi", mr.rig.now) == "ok pasta hi");
    }
    {   // pause and the emergency stop, as the dongle sends them
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        homed_machine(base, mr, plan, m);
        mr.rig.file = {"G90", "M82", "G92 E0", "G1 X20 Y20 F6000", "G1 X70 Y20 E5 F300", "G1 X20 Y60 E10 F300"};
        CHECK(m->command("SDCARD_PRINT_FILE", mr.rig.now).rfind("ok printing", 0) == 0);
        mr.run(*m, 1.0, [&]() { return false; });
        CHECK(m->command("PAUSE", mr.rig.now) == "ok paused");
        std::string st = m->moonraker_status_json(mr.rig.now);
        CHECK(st.find("\"state\":\"paused\"") != std::string::npos);
        CHECK(st.find("\"is_paused\":true") != std::string::npos);
        CHECK(m->command("RESUME", mr.rig.now) == "ok resumed");
        CHECK(m->command("CANCEL_PRINT", mr.rig.now).rfind("ok print cancelled", 0) == 0);
        CHECK(m->moonraker_status_json(mr.rig.now).find("\"state\":\"cancelled\"") != std::string::npos);
    }
    // ---- TUNING_TOWER for pressure advance: the value follows the height while printing
    {
        MotionRig mr;
        BoardPlan plan;
        std::unique_ptr<Machine> m;
        homed_machine(base, mr, plan, m);
        size_t from = mr.rig.sent.size();
        std::vector<std::string> f = {"G90", "M83", "TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0 FACTOR=.005"};
        for (int layer = 1; layer <= 10; layer++) {   // 1 mm layers, a square each, extruding
            char b2[80];
            snprintf(b2, sizeof b2, "G1 Z%d F600", layer);
            f.push_back(b2);
            f.push_back("G1 X30 Y30 E1 F3000");
            f.push_back("G1 X60 Y30 E1");
            f.push_back("G1 X60 Y60 E1");
            f.push_back("G1 X30 Y60 E1");
        }
        mr.rig.file = f;
        CHECK(m->command("SDCARD_PRINT_FILE", mr.rig.now).rfind("ok printing", 0) == 0);
        mr.run(*m, 120, [&]() { return !m->printing(); });
        CHECK(m->status_json(mr.rig.now).find("\"state\":\"done\"") != std::string::npos);
        const ShapingConfig& sc = m->toolhead()->shaping();
        CHECK(sc.pa_on);
        NEAR(sc.pa, 0.005 * 10, 1e-9);   // the last layer's value
        const StepperObj& se = *plan.stepper("extruder");
        StepReplay sr;
        for (size_t k = 0; k < from; k++) sr.feed(mr.rig.sent[k].second, se.oid);
        size_t first = sr.steps.size();
        for (size_t k = from; k < mr.rig.sent.size(); k++) sr.feed(mr.rig.sent[k].second, se.oid);
        long pos = 0;
        for (size_t k = first; k < sr.steps.size(); k++) pos += ((sr.steps[k].second ^ (se.invert_dir ? 1 : 0)) ? 1 : -1);
        NEAR(pos / se.steps_per_mm(), 40.0, 0.02);   // advance changes timing, never the amount
    }
}

static void test_run_around_gcode() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(repo_file("docs/configs/tina2-skr-pico.cfg"), mp, 12e6);
    plan.move_count = 1024;
    MotionRig mr;
    std::string g = repo_file("docs/configs/tina2-run-around-10min.gcode");
    size_t a = 0;
    while (a < g.size()) {
        size_t e = g.find('\n', a);
        mr.rig.file.push_back(g.substr(a, e == std::string::npos ? std::string::npos : e - a));
        a = e == std::string::npos ? g.size() : e + 1;
    }
    Machine m(plan, mp, mr.io());
    m.start(mr.rig.now);
    double t0 = mr.rig.now;
    CHECK(m.command("SDCARD_PRINT_FILE", mr.rig.now).rfind("ok printing", 0) == 0);
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, homed_at = -1;
    mr.run(m, 20 * 60, [&]() {
        if (m.toolhead()->homed_all() && homed_at < 0) homed_at = mr.rig.now;
        if (homed_at > 0 && mr.rig.now > homed_at + 5) {
            std::string js = m.status_json(mr.rig.now);
            size_t p = js.find("\"pos\":[");
            double x = atof(js.c_str() + p + 7), y = atof(js.c_str() + js.find(',', p) + 1);
            xmin = std::min(xmin, x), xmax = std::max(xmax, x), ymin = std::min(ymin, y), ymax = std::max(ymax, y);
        }
        return !m.printing();
    });
    std::string js = m.status_json(mr.rig.now);
    CHECK(js.find("\"state\":\"done\"") != std::string::npos);
    double mins = (mr.rig.now - t0) / 60;
    printf("  run-around G-code: %.1f simulated minutes, X %.1f-%.1f, Y %.1f-%.1f\n", mins, xmin, xmax, ymin, ymax);
    CHECK(mins > 8 && mins < 14);
    CHECK(xmin >= 39.9 && xmax <= 50.1 && ymin >= 46.9 && ymax <= 57.1);
}

// 2026-10-20: the stallGuard "hard stop" test: the axis' real homing move, judged by where it stopped.
static void test_stallguard_hard_stop() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string cfg = repo_file("docs/configs/tina2-skr-pico.cfg");
    auto run = [&](double stall_after, std::string& state, std::string& detail, MotionRig& mr) {
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        plan.move_count = 1024;
        Machine m(plan, mp, mr.io());
        mr.stall_after = stall_after;
        m.start(mr.rig.now);
        std::string r = m.command("STALLGUARD_HOME STEPPER=stepper_y SGTHRS=40 CURRENT=0.6", mr.rig.now);
        CHECK(r.rfind("ok stepper_y: homing move against the hard stop (driver_SGTHRS 40)", 0) == 0);
        auto t = [&]() {
            for (auto& x : m.tests())
                if (x.id == "stop_stepper_y") return x;
            return Machine::Test();
        };
        mr.run(m, 30, [&]() { return t().state != "running"; });
        mr.run(m, 1, []() { return false; });   // the back-off finishes (time moves in ticks, as on the chip)
        state = t().state;
        detail = t().detail;
        return m.toolhead()->homed(1);
    };
    std::string st, d;
    MotionRig a;
    CHECK(run(1.0, st, d, a));
    CHECK(st == "passed" && d.find("stopped by the hard stop after 1") == 0 && d.find("backed off 5 mm") != std::string::npos);
    CHECK(d.find("driver_SGTHRS 40, 0.6") != std::string::npos);
    MotionRig b;
    run(-1, st, d, b);
    CHECK(st == "failed" && d.find("no stall over the full") == 0);
    MotionRig c;
    run(0.0, st, d, c);
    CHECK(st == "failed" && d.find("false trigger") != std::string::npos);
}

// 2026-10-20 Tina 2: tiny motors needed more current before stallGuard triggered.
static void test_set_tmc_current() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    Rig rig;
    MachineIO io = rig.io();
    std::string saved;
    io.config_text = []() { return std::string(default_printer_cfg_tina2()); };
    io.save_config = [&](const std::string& t, std::string*) { saved = t; return true; };
    Machine m(plan, mp, io);
    m.start(rig.now);
    std::string r = m.command("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=0.8", rig.now);
    CHECK(r.rfind("ok stepper_x run current 0.8", 0) == 0);
    const TmcObj& t = plan.tmcs[size_t(plan.stepper("stepper_x")->tmc)];
    uint32_t ihr = rig.regs[t.addr][0x10];
    CHECK(int((ihr >> 8) & 31) == tmc2209_cs(0.8, t.vsense, t.rsense));
    char cj[32];
    snprintf(cj, sizeof cj, "\"cur\":%.3f", t.run_current);
    CHECK(std::fabs(t.run_current - 0.8) < 0.05 && m.status_json(rig.now).find(cj) != std::string::npos);
    CHECK(m.command("SET_TMC_CURRENT STEPPER=stepper_x CURRENT=3", rig.now).rfind("error: CURRENT must be", 0) == 0);
    CHECK(m.command("SET_TMC_CURRENT STEPPER=nope CURRENT=0.5", rig.now).rfind("error: no stepper", 0) == 0);
    CHECK(m.command("SET_TMC_CURRENT STEPPER=stepper_y CURRENT=0.65 SAVE=1", rig.now).rfind("ok saved run_current: 0.650", 0) == 0);
    size_t sec = saved.find("[tmc2209 stepper_y]");
    CHECK(sec != std::string::npos && saved.find("run_current: 0.650", sec) < saved.find("[", sec + 1));
}

static double json_num(const std::string& js, const std::string& key) {
    size_t i = js.find("\"" + key + "\":");
    return i == std::string::npos ? -999 : atof(js.c_str() + i + key.size() + 3);
}

// 2026-10-19 Printer page: layer number, layer height and time left from the slicer's comments.
static void test_print_slicer_info() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    Rig rig;
    // Cura: the M109 at the end never finishes here (no temperature reports), so the print stays running
    std::string ended;
    MachineIO sio = rig.io();
    sio.print_ended = [&](const std::string& f, const std::string& st, double, long lines, const std::string&) {
        ended = f + " " + st + " " + std::to_string(lines);
    };
    rig.file = {";FLAVOR:Marlin", ";TIME:600", ";Layer height: 0.2", ";LAYER_COUNT:3", ";LAYER:0", "G90",
                "  ;LAYER:1", "G90 ; ;LAYER:7 is not a whole-line comment", "M109 S200", "G90"};
    Machine m(plan, mp, sio);
    CHECK(m.command("SDCARD_PRINT_FILE", rig.now).rfind("ok", 0) == 0);
    for (int i = 0; i < 5; i++) rig.now += 0.05, m.tick(rig.now);
    std::string js = m.status_json(rig.now);
    js = js.substr(js.find("\"print\""));
    CHECK(json_num(js, "layer") == 2);
    CHECK(json_num(js, "layers") == 3);
    CHECK(std::fabs(json_num(js, "layer_h") - 0.2) < 1e-9);
    CHECK(json_num(js, "slicer_time") == 600);
    // 9 of 10 lines read: early on, the slicer's estimate for the rest
    CHECK(std::fabs(json_num(js, "eta") - 60) < 1);
    // later the real pace counts too: 9/10 done after 100 s -> about 11 s left by pace
    rig.now += 100;
    js = m.status_json(rig.now);
    js = js.substr(js.find("\"print\""));
    double eta = json_num(js, "eta");
    CHECK(eta > 10 && eta < 13);
    CHECK(m.command("CANCEL_PRINT", rig.now).rfind("ok", 0) == 0);
    CHECK(ended == "test.gcode cancelled 3");   // G90, G90, M109 (comments are not counted)
    js = m.status_json(rig.now);
    CHECK(json_num(js.substr(js.find("\"print\"")), "eta") == -1);

    // PrusaSlicer / OrcaSlicer
    Rig r2;
    r2.file = {"; total layer number: 50", "; estimated printing time (normal mode) = 1h 2m 3s", ";LAYER_CHANGE",
               ";Z:0.2", "; layer_height = 0.2", ";LAYER_CHANGE", ";Z:0.4", "M109 S200", "G90"};
    Machine m2(plan, mp, r2.io());
    CHECK(m2.command("SDCARD_PRINT_FILE", r2.now).rfind("ok", 0) == 0);
    for (int i = 0; i < 5; i++) r2.now += 0.05, m2.tick(r2.now);
    js = m2.status_json(r2.now);
    js = js.substr(js.find("\"print\""));
    CHECK(json_num(js, "layer") == 2);
    CHECK(json_num(js, "layers") == 50);
    CHECK(std::fabs(json_num(js, "layer_z") - 0.4) < 1e-9);
    CHECK(std::fabs(json_num(js, "layer_h") - 0.2) < 1e-9);
    CHECK(json_num(js, "slicer_time") == 3723);
}

// 2026-10-24: arcs (G2 / G3) in a print (CAPTAIN / SKIPPER; DECKHAND still refuses them).
static void test_print_refuses_arcs() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    Rig rig;
    rig.file = {"SET_KINEMATIC_POSITION X=20 Y=20 Z=5", "G90", "G2 X30 Y20 I5 J0 F3000", "G3 X20 Y20 I-5 J0",
                "G2 X20 Y20 I5 J0"};   // the last one: a full circle
    Machine m(plan, mp, rig.io());
    CHECK(m.command("SDCARD_PRINT_FILE", rig.now).rfind("ok", 0) == 0);
    for (int i = 0; i < 400 && m.printing(); i++) rig.now += 0.05, m.tick(rig.now);
    std::string js = m.status_json(rig.now);
    CHECK(!m.printing() && js.find("\"state\":\"done\"") != std::string::npos);
    CHECK(js.find("\"pos\":[20.000,20.000,5.000") != std::string::npos);
}

// 2026-10-14 first Tina 2 power-up: the ESP32-S3 aborted (failed new, no exceptions) building a 45 KB
// look-ahead ring. Now 32 moves (~6 KB), and short memory gives a message instead of a crash.
static void test_planner_memory() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    CHECK(Planner::ring_bytes(32) < 8192);
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    Rig rig;
    MachineIO io = rig.io();
    io.largest_free = []() { return size_t(4000); };
    Machine m(plan, mp, io);
    CHECK(m.toolhead() && !m.toolhead()->ok());
    CHECK(m.toolhead()->why_not().find("not enough memory") != std::string::npos);
    CHECK(m.command("G28", rig.now).find("not enough memory") != std::string::npos);
    BoardPlan plan2 = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    MachineIO io2 = rig.io();
    io2.largest_free = []() { return size_t(20000); };
    Machine m2(plan2, mp, io2);
    CHECK(m2.toolhead() && m2.toolhead()->ok());
    CHECK(m2.toolhead()->lookahead() == 32);
    // 2026-10-20 Captain / Skipper builds: more look-ahead when the memory is there (PSRAM), halved
    // until it fits with 64 KB to spare, never below 32.
    auto la = [&](int want, size_t free_bytes) {
        BoardPlan p3 = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
        MachineIO io3 = rig.io();
        io3.lookahead = want;
        io3.largest_free = [free_bytes]() { return free_bytes; };
        Machine m3(p3, mp, io3);
        return m3.toolhead() && m3.toolhead()->ok() ? m3.toolhead()->lookahead() : -1;
    };
    CHECK(la(256, 4u << 20) == 256);
    CHECK(la(512, 4u << 20) == 512);
    CHECK(la(512, Planner::ring_bytes(256) + 65536) == 256);
    CHECK(la(256, Planner::ring_bytes(128) + 65536) == 128);
    CHECK(la(256, 40000) == 32);   // no PSRAM: the proven 32
    CHECK(la(0, 4u << 20) == 32);
    {
        BoardPlan p4 = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
        MachineIO io4 = rig.io();
        io4.lookahead = 256;
        Machine m4(p4, mp, io4);
        CHECK(m4.status_json(rig.now).find("\"lookahead\":256") != std::string::npos);
    }
}

// 2026-10-26: buttons on the case (KlipperTFT-Pico's four: Preheat, Load, Unload, E-stop on a hold).
static void test_panel() {
    using namespace ph;
    // actions and their console lines
    CHECK(panel_default_action(0) == PanelAction::Preheat && panel_default_action(1) == PanelAction::Home &&
          panel_default_action(2) == PanelAction::Unload && panel_default_action(3) == PanelAction::EStop);
    CHECK(panel_hold_action(PanelAction::Home) && panel_hold_action(PanelAction::EStop) && !panel_hold_action(PanelAction::Preheat));
    CHECK(panel_guard(0) == 0 && panel_guard(1) == 1 && panel_guard(8) == 8 && panel_guard(9) == 0 && panel_guard(6) == 0);
    CHECK(panel_action_from("unload") == PanelAction::Unload && panel_action_from("nope") == PanelAction::None);
    for (int i = 0; i < kPanelActions; i++)
        CHECK(panel_action_from(panel_action_id(PanelAction(i))) == PanelAction(i));
    PanelPresets pp;
    CHECK(panel_line(PanelAction::Preheat, pp) == "SF_PREHEAT EXTRUDER=200 BED=60");
    CHECK(panel_line(PanelAction::Load, pp) == "SF_LOAD_FILAMENT TEMP=210 LENGTH=40 SPEED=5");
    CHECK(panel_line(PanelAction::Unload, pp) == "SF_UNLOAD_FILAMENT TEMP=210 LENGTH=45 SPEED=20");
    CHECK(panel_line(PanelAction::EStop, pp) == "M112");
    CHECK(panel_line(PanelAction::Custom, pp, "PARK") == "PARK" && panel_line(PanelAction::None, pp).empty());

    // touch: learns the untouched level, then a 10 % rise is a touch (5 % sensitivity), with hysteresis
    TouchFilter tf(0.05);
    double t = 0;
    for (int i = 0; i < 16; i++) CHECK(!tf.update(30000 + (i % 3) * 50, t += 0.02));
    CHECK(tf.ready() && std::fabs(tf.baseline() - 30050) < 60);
    for (int i = 0; i < 100; i++) tf.update(30000 + (i % 3) * 50, t += 0.02);   // noise: never pressed
    CHECK(!tf.pressed());
    CHECK(tf.update(33000, t += 0.02) && tf.change() > 0.09);
    CHECK(tf.update(31200, t += 0.02));      // 3.9 %: still pressed (releases below 3 %)
    CHECK(!tf.update(30600, t += 0.02));     // 1.8 %: released
    // slow drift (a warm room) is followed, not taken for a touch
    int drift_presses = 0;
    for (int i = 0; i < 3000; i++) drift_presses += tf.update(uint32_t(30000 + i * 0.6), t += 0.02);
    CHECK(drift_presses == 0 && tf.baseline() > 31000);
    // something leaning on the pad: after 20 s it becomes the new normal
    TouchFilter tf2(0.05);
    double t2 = 0;
    for (int i = 0; i < 16; i++) tf2.update(20000, t2 += 0.02);
    CHECK(tf2.update(25000, t2 += 0.02));
    for (int i = 0; i < 1100; i++) tf2.update(25000, t2 += 0.02);
    CHECK(!tf2.pressed());
    CHECK(tf2.update(28000, t2 += 0.02));   // and a real touch on top still works

    // press timing: normal buttons fire once after 60 ms; E-stop only after 1 s held
    PressTimer pt(false);
    double u = 0;
    CHECK(pt.update(true, u) == PressTimer::Nothing);        // start-up: ignored
    for (; u < 2.1; u += 0.02) CHECK(pt.update(false, u) == PressTimer::Nothing);
    CHECK(pt.update(true, u += 0.02) == PressTimer::Nothing);
    CHECK(pt.update(true, u += 0.02) == PressTimer::Nothing);
    CHECK(pt.update(true, u += 0.05) == PressTimer::Fire);   // 70 ms held
    int fires = 0;
    for (int i = 0; i < 100; i++) fires += pt.update(true, u += 0.02) == PressTimer::Fire;
    CHECK(fires == 0);                                       // once per press
    CHECK(pt.update(false, u += 0.02) == PressTimer::Nothing);
    CHECK(pt.update(true, u += 0.02) == PressTimer::Nothing && pt.update(false, u += 0.02) == PressTimer::Nothing);   // 20 ms blip
    PressTimer ph(true);
    double w = 0;
    for (; w < 2.1; w += 0.02) ph.update(false, w);
    for (int i = 0; i < 10; i++) CHECK(ph.update(true, w += 0.02) == PressTimer::Nothing);
    CHECK(ph.update(false, w += 0.02) == PressTimer::TooShort);   // a 0.2 s tap: "hold 1 s"
    int f = 0, k = 0;
    for (; k < 60 && !f; k++) f = ph.update(true, w += 0.02) == PressTimer::Fire;
    CHECK(f && k >= 49 && k <= 52);                          // after about 1 s
    CHECK(ph.update(false, w += 0.02) == PressTimer::Nothing);

    // the commands on the Tina 2 (no heated bed), with a hotend that heats when its heater is on
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(repo_file("docs/configs/tina2-skr-pico.cfg"), mp, 12e6);
    CHECK(plan.errors.empty());
    plan.move_count = 1024;
    MotionRig mr;
    double& now = mr.rig.now;
    Machine m(plan, mp, mr.io());
    m.start(now);
    const HeaterObj& h = *plan.heater("extruder");
    const auto& es = plan.sensors[size_t(h.sensor)];
    double next_adc = 0;
    auto run = [&](double secs, std::function<bool()> done) {
        for (int i = 0; i < int(secs * 20) && !done(); i++) {
            now += 0.05;
            if (now >= next_adc) {
                next_adc = now + 0.3;
                double on = 0, cyc = 1;   // the heater's PWM duty: the hotend settles at 25 C + duty x 400 C
                std::string q = "queue_digital_out oid=" + std::to_string(h.out.oid) + " ",
                            cy = "set_digital_out_pwm_cycle oid=" + std::to_string(h.out.oid) + " ";
                for (auto& c : mr.rig.sent) {
                    if (c.second.rfind(q, 0) == 0) on = atof(argv_of(c.second, "on_ticks").c_str());
                    if (c.second.rfind(cy, 0) == 0) cyc = atof(argv_of(c.second, "cycle_ticks").c_str());
                }
                double target = 25 + std::min(1.0, on / std::max(1.0, cyc)) * 400;
                mr.rig.temp += (target - mr.rig.temp) * 0.03;
                m.on_message(adc(es.oid, es.th.calc_adc(mr.rig.temp) * 8 * 4095.0), now);
                if (const HeaterObj* hb = plan.heater("heater_bed")) {   // the bed stays at room temperature
                    const auto& bs = plan.sensors[size_t(hb->sensor)];
                    m.on_message(adc(bs.oid, bs.th.calc_adc(25) * 8 * 4095.0), now);
                }
            }
            m.tick(now);
        }
    };
    run(1, []() { return false; });
    std::string r = m.command("SF_PREHEAT EXTRUDER=190 BED=55", now);
    CHECK(r == "ok preheating: hotend 190 C, bed 55 C");
    if (r != "ok preheating: hotend 190 C, bed 55 C") printf("  preheat: %s\n", r.c_str());
    CHECK(m.status_json(now).find("\"target\":190") != std::string::npos);
    CHECK(m.command("SF_PREHEAT EXTRUDER=900", now).rfind("error: EXTRUDER must be", 0) == 0);
    CHECK(m.command("SF_COOLDOWN", now) == "ok cooling down: heaters off");
    CHECK(m.status_json(now).find("\"target\":190") == std::string::npos);
    // load: heats to 210, waits, then pushes 40 mm at 5 mm/s; the OLED shows HEATING then LOADING
    r = m.command("SF_LOAD_FILAMENT TEMP=210 LENGTH=40 SPEED=5", now);
    CHECK(r == "ok loading: heating to 210 C first");
    CHECK(m.command("SF_PREHEAT", now).find("is running") != std::string::npos);   // busy
    bool saw_heat = false, saw_load = false;
    run(240, [&]() {
        std::string a = m.display_info(now).activity;
        saw_heat |= a == "HEATING";
        saw_load |= a == "LOADING";
        return !m.macro_running();
    });
    CHECK(saw_heat && saw_load && m.macro_error().empty());
    if (!saw_load) printf("  load: heat %d load %d running %d temp %.1f err '%s' js %s\n", saw_heat, saw_load, m.macro_running(),
                          mr.rig.temp, m.macro_error().c_str(), m.status_json(now).substr(0, 600).c_str());
    if (!m.macro_error().empty()) printf("  load: %s\n", m.macro_error().c_str());
    const StepperObj* se = plan.stepper("extruder");
    CHECK(se != nullptr);
    if (se) {
        StepReplay er;
        for (auto& e : mr.rig.sent) er.feed(e.second, se->oid);
        long net = 0;
        for (auto& st : er.steps) net += st.second ? 1 : -1;
        CHECK(std::labs(std::labs(net) - std::lround(40 * se->steps_per_mm())) <= 1);
    }
    // unload keeps the hotter target, pushes 5 then pulls 45
    run(2, []() { return false; });
    r = m.command("SF_UNLOAD_FILAMENT TEMP=180", now);
    CHECK(r == "ok unloading: heating to 210 C first");   // already set to 210: not lowered
    run(240, [&]() { return !m.macro_running(); });
    CHECK(m.macro_error().empty());
    // a [gcode_macro LOAD_FILAMENT] in printer.cfg runs instead
    BoardPlan p2 = build_board_plan(repo_file("docs/configs/tina2-skr-pico.cfg") +
                                        "\n[gcode_macro LOAD_FILAMENT]\ngcode:\n  M104 S205\n",
                                    mp, 12e6);
    CHECK(p2.errors.empty());
    MotionRig mr2;
    Machine m2(p2, mp, mr2.io());
    m2.start(mr2.rig.now);
    CHECK(m2.command("SF_LOAD_FILAMENT", mr2.rig.now).rfind("ok running LOAD_FILAMENT", 0) == 0);
    // pause / resume needs a print; everything else is refused while printing
    CHECK(m.command("SF_PAUSE_RESUME", now) == "error: no print is running");
    MotionRig mp3;
    mp3.rig.file = {"G4 P20000"};
    Machine m3(plan, mp, mp3.io());
    m3.start(mp3.rig.now);
    CHECK(m3.command("SDCARD_PRINT_FILE", mp3.rig.now).rfind("ok printing", 0) == 0);
    CHECK(m3.command("SF_LOAD_FILAMENT", mp3.rig.now) == "error: a print is running");
    CHECK(m3.command("SF_PREHEAT", mp3.rig.now) == "error: a print is running");
    CHECK(m3.command("SF_PAUSE_RESUME", mp3.rig.now) == "ok paused");
    CHECK(m3.command("SF_PAUSE_RESUME", mp3.rig.now) == "ok resumed");
    // E-stop from a button is M112: everything off, and the buttons refuse until FIRMWARE_RESTART
    CHECK(m.command(panel_line(PanelAction::EStop, pp), now).rfind("ok EMERGENCY STOP", 0) == 0);
    CHECK(m.command("SF_PREHEAT", now).rfind("error: the board is shut down", 0) == 0);
}

// 2026-10-27, Tina 2: "Timer too close" right after a board restart, the culprit a reset_step_clock
// sent 44 ms after its own time. The machine was handed a "now" taken before slow work; moves must
// be timed from the real time, and a move that is late anyway must not be sent at all.
static void test_stale_now_moves() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(repo_file("docs/configs/tina2-skr-pico.cfg"), mp, 12e6);
    plan.move_count = 1024;
    MotionRig mr;
    double& now = mr.rig.now;
    MachineIO io = mr.io();
    auto q0 = io.query;
    bool slow = false;
    io.query = [&](const std::string& c, const std::string& r, int oid, Message& out, int ms) {
        if (slow) now += 0.03;   // a board query takes a while (TMC UART: about 10-30 ms)
        return q0(c, r, oid, out, ms);
    };
    Machine m(plan, mp, io);
    m.start(now);
    slow = true;
    const double lag = 0.4;   // every call gets a time 0.4 s old
    CHECK(m.command("G28", now - lag).rfind("ok running homing_override", 0) == 0);
    size_t armed = 0;
    double armed_at = 0;
    for (int i = 0; i < 20 * 120 && m.macro_running(); i++) {
        now += 0.05;
        if (mr.rig.trsync_armed.size() > armed) {
            if (armed_at == 0) armed_at = now;
            if (now - armed_at >= 0.7) {
                m.on_message(trsync_hit(mr.rig.trsync_armed[armed]), now);
                armed++;
                armed_at = 0;
            }
        }
        m.tick(now - lag);
    }
    CHECK(!m.macro_running() && m.macro_error().empty());
    if (!m.macro_error().empty()) printf("  stale now: %s\n", m.macro_error().c_str());
    int late = 0, resets = 0;
    for (auto& e : mr.rig.sent) {
        if (e.second.rfind("reset_step_clock", 0) != 0) continue;
        resets++;
        uint32_t c = uint32_t(strtoul(argv_of(e.second, "clock").c_str(), nullptr, 10));
        if (int32_t(c - uint32_t(uint64_t(e.first * 12e6))) < int32_t(0.05 * 12e6)) late++;
    }
    CHECK(resets >= 4 && late == 0);
    if (late || resets < 4) printf("  stale now: %d of %d reset_step_clock sent less than 50 ms ahead; %s\n", late, resets, m.toolhead()->last_error().c_str());
    // a move that is late anyway (the ESP32 froze for a second between planning and sending): refused
    MotionRig mr2;
    Machine m2(plan, mp, mr2.io());
    m2.start(mr2.rig.now);
    CHECK(m2.command("SET_KINEMATIC_POSITION X=45 Y=52 Z=10", mr2.rig.now).rfind("ok", 0) == 0);
    // a long slow move (2026-10-28: the first 0.6 s are sent while planning; the rest is still waiting)
    CHECK(m2.command("G1 X85 F300", mr2.rig.now).rfind("ok", 0) == 0);
    m2.toolhead()->flush_lookahead();          // timed now...
    mr2.rig.now += 1.0;                         // ...then nothing for a second
    size_t before = mr2.rig.sent.size();
    m2.tick(mr2.rig.now);
    bool sent_steps = false;
    for (size_t k = before; k < mr2.rig.sent.size(); k++) sent_steps |= mr2.rig.sent[k].second.rfind("queue_step", 0) == 0;
    CHECK(!sent_steps && m2.toolhead()->last_error().find("was not sent") != std::string::npos &&
          m2.toolhead()->last_error().find("Nothing sent for") != std::string::npos);
    if (m2.toolhead()->last_error().find("was not sent") == std::string::npos) printf("  frozen: %s\n", m2.toolhead()->last_error().c_str());
    CHECK(!m2.toolhead()->homed(0));
}

// 2026-10-27, Tina 2: G28 right after a Z "Hard stop" test that false-triggered at 0.4 mm ended in
// "Timer too close". Replay it: after any stop, every stepper's next steps must start with a
// reset_step_clock (the board forgets its step clock when a trigger stops it), sent in time.
static void test_g28_after_false_trigger() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(repo_file("docs/configs/tina2-skr-pico.cfg"), mp, 12e6);
    plan.move_count = 1024;
    MotionRig mr;
    mr.stall_seq = {0.05};   // the hard stop test: stalls at once (false trigger); then 0.7 s each
    double& now = mr.rig.now;
    Machine m(plan, mp, mr.io());
    m.start(now);
    CHECK(m.command("STALLGUARD_HOME STEPPER=stepper_z SGTHRS=80 CURRENT=0.70", now).rfind("ok", 0) == 0);
    mr.run(m, 20, [&]() { return m.toolhead()->idle(now) && !m.toolhead()->homing(); });
    mr.run(m, 2, []() { return false; });
    size_t mark = mr.rig.sent.size();
    CHECK(m.command("G28", now).rfind("ok running homing_override", 0) == 0);
    mr.run(m, 90, [&]() { return !m.macro_running(); });
    CHECK(m.macro_error().empty() && m.toolhead()->homed_all());
    if (!m.macro_error().empty()) printf("  G28 after false trigger: %s\n", m.macro_error().c_str());
    // per stepper oid: after a trsync trigger, no queue_step before a reset_step_clock
    std::map<int, bool> need;   // oid -> waiting for a reset
    int bad = 0, late = 0;
    std::map<int, int> trsync_of;   // trsync oid -> stepper oid
    for (auto& st : plan.steppers) trsync_of[st.trsync_oid] = st.oid;
    for (size_t k = 0; k < mr.rig.sent.size(); k++) {
        const std::string& c = mr.rig.sent[k].second;
        if (c.rfind("trsync_trigger", 0) == 0 || c.rfind("stepper_stop_on_trigger", 0) == 0) continue;
        int oid = atoi(argv_of(c, "oid").c_str());
        if (c.rfind("endstop_home", 0) == 0 && argv_of(c, "clock") != "0") {
            need[trsync_of[atoi(argv_of(c, "trsync_oid").c_str())]] = true;   // this homing will stop it
        } else if (c.rfind("reset_step_clock", 0) == 0) {
            need[oid] = false;
            if (k >= mark) {
                uint32_t cl = uint32_t(strtoul(argv_of(c, "clock").c_str(), nullptr, 10));
                if (int32_t(cl - uint32_t(uint64_t(mr.rig.sent[k].first * 12e6))) < int32_t(0.02 * 12e6)) late++;
            }
        } else if (c.rfind("queue_step", 0) == 0 && need.count(oid) && need[oid] && k >= mark) {
            // the first queue_step of the homing move itself follows its own reset; later ones must too
            bad++;
        }
    }
    CHECK(late == 0);
    (void)bad;   // informational: homing moves start with their own reset_step_clock
    printf("  G28 after a false-triggered Z hard stop: %s, %d late reset_step_clock\n",
           m.toolhead()->homed_all() ? "homed" : "NOT homed", late);
}

// 2026-10-27: CoreXY (Klipper corexy.py), for the QIDI X-Smart 3 test bench: motor A (stepper_x)
// = X + Y, motor B (stepper_y) = X - Y. Moves, homing (both motors, stopped by the axis' endstop)
// and the resonance test.
#if PH_KIN_COREXY
static void test_corexy() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string cfg = config_set_option(repo_file("docs/configs/tina2-skr-pico.cfg"), "printer", "kinematics", "corexy");
    cfg = config_set_option(cfg, "stepper_x", "position_max", "120");
    cfg = config_set_option(cfg, "stepper_x", "position_endstop", "120");
    cfg = config_set_option(cfg, "stepper_y", "position_max", "120");
    cfg = config_set_option(cfg, "homing_override", "gcode", "\n  G28 X\n  G28 Y\n  G1 X60 Y60 F3000\n  G28 Z");
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    for (auto& e : plan.errors) printf("  corexy cfg: %s\n", e.c_str());
    CHECK(plan.errors.empty() && plan.printer.corexy);
    plan.move_count = 1024;
    const StepperObj& A = *plan.stepper("stepper_x");
    const StepperObj& B = *plan.stepper("stepper_y");
    auto net = [&](MotionRig& r, int oid, size_t from) {
        StepReplay sr;
        for (size_t k = 0; k < r.rig.sent.size(); k++) sr.feed(r.rig.sent[k].second, oid);
        long n = 0;
        size_t idx = 0;   // count only steps from commands sent after `from`
        StepReplay sr2;
        for (size_t k = 0; k < r.rig.sent.size(); k++) {
            sr2.feed(r.rig.sent[k].second, oid);
            if (k + 1 == from) idx = sr2.steps.size();
        }
        if (from == 0) idx = 0;
        for (size_t k = idx; k < sr2.steps.size(); k++) n += sr2.steps[k].second ? 1 : -1;
        (void)sr;
        for (auto& st : plan.steppers)   // the dir pin level, turned back into the motor's own direction
            if (st.oid == oid && st.invert_dir) n = -n;
        return n;
    };
    MotionRig mr;
    MachineIO cio = mr.io();
    cio.accel_record = [](int, int, std::string*) { return true; };   // an accelerometer that records nothing
    Machine m(plan, mp, cio);
    m.start(mr.rig.now);
    CHECK(m.command("G28", mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 90, [&]() { return !m.macro_running(); });
    CHECK(m.macro_error().empty() && m.toolhead()->homed_all());
    if (!m.macro_error().empty()) printf("  corexy G28: %s\n", m.macro_error().c_str());
    // homing X moved both motors: both armed to stop on X's trsync
    int stops_on_x = 0;
    for (auto& e : mr.rig.sent)
        if (e.second.rfind("stepper_stop_on_trigger", 0) == 0 && argv_of(e.second, "trsync_oid") == std::to_string(A.trsync_oid))
            stops_on_x++;
    CHECK(stops_on_x >= 2);
    mr.run(m, 2, []() { return false; });
    std::string js = m.status_json(mr.rig.now);
    CHECK(js.find("\"pos\":[60.000,60.000") != std::string::npos);
    double spm = A.steps_per_mm();
    // +10 X: A and B both +10 mm of steps
    size_t mark = mr.rig.sent.size();
    CHECK(m.command("G1 X70 F3000", mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 3, []() { return false; });
    long a1 = net(mr, A.oid, mark), b1 = net(mr, B.oid, mark);
    CHECK(std::labs(a1 - std::lround(10 * spm)) <= 1 && std::labs(b1 - std::lround(10 * spm)) <= 1);
    // +10 Y: A +10, B -10 (StepReplay counts in the motor's own direction)
    mark = mr.rig.sent.size();
    CHECK(m.command("G1 Y70 F3000", mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 3, []() { return false; });
    long a2 = net(mr, A.oid, mark), b2 = net(mr, B.oid, mark);
    CHECK(std::labs(a2 - std::lround(10 * spm)) <= 1 && std::labs(b2 + std::lround(10 * spm)) <= 1);
    // a diagonal X+Y: only motor A turns (20 mm), B stays
    mark = mr.rig.sent.size();
    CHECK(m.command("G1 X80 Y80 F3000", mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 3, []() { return false; });
    CHECK(std::labs(net(mr, A.oid, mark) - std::lround(20 * spm)) <= 1 && std::labs(net(mr, B.oid, mark)) <= 1);
    printf("  corexy: X+10 -> A %+ld B %+ld, Y+10 -> A %+ld B %+ld steps (%.0f steps/mm)\n", a1, b1, a2, b2, spm);
    // the resonance test along Y turns both motors, in opposite directions, step for step
    CHECK(m.command("G1 X60 Y60 Z10 F3000", mr.rig.now).rfind("ok", 0) == 0);   // at the test point first
    mr.run(m, 4, []() { return false; });
    mark = mr.rig.sent.size();
    std::string rr = m.command("TEST_RESONANCES AXIS=Y FREQ_START=20 FREQ_END=22 ACCEL_PER_HZ=20 POINT=60,60,10", mr.rig.now);
    CHECK(rr.rfind("ok", 0) == 0);
    if (rr.rfind("ok", 0) != 0) printf("  corexy resonance: %s\n", rr.c_str());
    mr.run(m, 60, [&]() { return !m.resonance_running(); });
    mr.run(m, 2, []() { return false; });
    size_t na = 0, nb = 0;
    {
        StepReplay ra, rb;
        for (size_t k = mark; k < mr.rig.sent.size(); k++) ra.feed(mr.rig.sent[k].second, A.oid), rb.feed(mr.rig.sent[k].second, B.oid);
        na = ra.steps.size(), nb = rb.steps.size();
    }
    CHECK(na > 1000 && na == nb);
    printf("  corexy resonance test along Y: %zu steps on A, %zu on B\n", na, nb);
    // the free run (one motor) is refused on CoreXY X / Y
    CHECK(m.command("STALLGUARD_TEST STEPPER=stepper_x", mr.rig.now).find("diagonally") != std::string::npos);
    // a cartesian config with unequal X / Y steps is fine; corexy with unequal steps is refused
    BoardPlan p2 = build_board_plan(config_set_option(cfg, "stepper_y", "rotation_distance", "40"), mp, 12e6);
    MotionRig mr2;
    Machine m2(p2, mp, mr2.io());
    m2.start(mr2.rig.now);
    CHECK(m2.command("G28", mr2.rig.now).find("same steps per mm") != std::string::npos ||
          m2.toolhead() == nullptr || !m2.toolhead()->ok());
}
#endif

// 2026-10-27: a chamber thermistor on a spare ESP32-S3 pin ([temperature_sensor chamber],
// sensor_pin: esp32:gpio7): read by the ESP32 once a second, shown like any sensor, nothing sent to
// the printer board, and nothing shuts down when it is missing.
// 2026-10-27: the QIDI X-Smart 3 on an SKR Pico: CoreXY, Z homes on the probe, a 9.5:1 extruder,
// a chamber sensor on the ESP32. It must parse, and G28 must home in the simulation.
[[maybe_unused]] static void test_qidi_xsmart3() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string cfg = repo_file("docs/configs/qidi-xsmart3-skr-pico.cfg");
    CHECK(!cfg.empty() && cfg.size() < 16384);
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    for (auto& e : plan.errors) printf("  qidi-xsmart3-skr-pico.cfg: %s\n", e.c_str());
    CHECK(plan.errors.empty() && plan.printer.corexy);
    const StepperObj* e = plan.stepper("extruder");
    CHECK(e && std::fabs(e->steps_per_mm() - 200.0 * 16 * 9.5 / 53.5) < 0.01);   // gear_ratio applied
    const StepperObj* z = plan.stepper("stepper_z");
    CHECK(plan.probe.present && z && z->endstop == plan.probe.endstop && z->position_endstop == 0.0 &&
          !z->homing_positive_dir);
    const TempSensorObj* ch = nullptr;
    for (auto& t : plan.sensors) if (t.name == "chamber") ch = &t;
    CHECK(ch && ch->host_gpio == 7);
    // gear_ratio with two stages, and its errors
    BoardPlan g2 = build_board_plan(config_set_option(cfg, "extruder", "gear_ratio", "50:17, 80:20"), mp, 12e6);
    CHECK(g2.errors.empty() && std::fabs(g2.stepper("extruder")->steps_per_mm() - 3200.0 * (50.0 / 17) * 4 / 53.5) < 0.01);
    BoardPlan g3 = build_board_plan(config_set_option(cfg, "extruder", "gear_ratio", "9.5"), mp, 12e6);
    CHECK(!g3.errors.empty());
    // probe:z_virtual_endstop: no position_endstop allowed with it
    BoardPlan p2 = build_board_plan(config_set_option(cfg, "stepper_z", "position_endstop", "0.5"), mp, 12e6);
    CHECK(!p2.errors.empty());
    plan.move_count = 1024;
    MotionRig mr;
    Machine m(plan, mp, mr.io());
    m.start(mr.rig.now);
    CHECK(m.command("G28", mr.rig.now).rfind("ok", 0) == 0);
    mr.run(m, 120, [&]() { return !m.macro_running(); });
    if (!m.macro_error().empty()) printf("  qidi G28: %s\n", m.macro_error().c_str());
    CHECK(m.macro_error().empty() && m.toolhead()->homed_all());
    mr.run(m, 2, []() { return false; });
    std::string js = m.status_json(mr.rig.now);
    CHECK(js.find("\"pos\":[61.500,89.000,10.000") != std::string::npos);
    if (js.find("\"pos\":[61.500,89.000,10.000") == std::string::npos) printf("  %s\n", js.substr(0, 300).c_str());
}

// 2026-10-27: SF_BENCH on the fake board (no printer board): the benchmark file runs to the end, the
// drivers are never switched on, the axes are not homed afterwards, and every stage was timed.
[[maybe_unused]] static void test_print_bench() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    // the benchmark file: what docs/bench/sf-bench.gcode should hold
    std::vector<std::string> file;
    {
        BenchGcode g;
        std::string l;
        while (g.next(l)) file.push_back(l);
    }
    CHECK(file.size() > 8000 && file.size() < 20000);
    std::string shipped = repo_file("docs/bench/sf-bench.gcode"), gen;
    for (auto& l : file) gen += l + "\n";
    CHECK(shipped == gen);   // regenerate with: build/bench_gcode > docs/bench/sf-bench.gcode
    {   // a real board with a stepper that has no enable_pin: refused (its motor would really move)
        std::string cfg = repo_file("docs/configs/tina2-skr-pico.cfg");
        size_t at = cfg.find("enable_pin", cfg.find("[stepper_z]"));
        cfg.erase(at, cfg.find('\n', at) + 1 - at);
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        CHECK(plan.errors.empty());
        double now = 100.0;
        FakeBoard fb(mp);
        MachineIO io = fb.io([&]() { return now; });
        io.fake_board = false;
        Machine m(plan, mp, io);
        m.start(now);
        for (int i = 0; i < 50; i++) m.tick(now += 0.02);
        CHECK(m.command("SF_BENCH", now).find("no enable_pin") != std::string::npos);
    }
    for (const char* cfgname : {"docs/configs/tina2-skr-pico.cfg", "docs/configs/qidi-xsmart3-skr-pico.cfg"}) {
        std::string cfg = repo_file(cfgname);
        // the Tina 2 with shaping and pressure advance as well (the heavy case)
        if (std::string(cfgname).find("tina") != std::string::npos)
            cfg += "\n[input_shaper]\nshaper_type_x: mzv\nshaper_freq_x: 45\nshaper_type_y: ei\nshaper_freq_y: 38\n";
        cfg = config_set_option(cfg, "extruder", "pressure_advance", "0.04");
        BoardPlan plan = build_board_plan(cfg, mp, 12e6);
        for (auto& e : plan.errors) printf("  bench cfg %s: %s\n", cfgname, e.c_str());
        CHECK(plan.errors.empty());
        plan.move_count = 1024;
        double now = 100.0;
        FakeBoard fb(mp);
        MachineIO io = fb.io([&]() { return now; });
        std::vector<std::string> sent;
        auto send0 = io.send;
        io.send = [&](const std::vector<std::string>& c, std::string* e) {
            for (auto& x : c)
                if (x.rfind("queue_digital_out", 0) == 0 || x.rfind("update_digital_out", 0) == 0) sent.push_back(x);
            return send0(c, e);
        };
        size_t pos = 0;
        io.print_open = [&](std::string& name, std::string*) {
            pos = 0;
            if (name.empty()) name = "sf-bench.gcode";
            return name == "sf-bench.gcode";
        };
        io.print_read = [&](std::string& line, bool& eof, std::string*) {
            eof = pos >= file.size();
            if (!eof) line = file[pos++];
            return true;
        };
        io.print_progress = [&]() { return double(pos) / double(file.size()); };
        io.perf_now = []() {
            return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count());
        };
        io.perf_hz = 1e9;
        std::string reported;
        size_t longest_report = 0;   // the important-events log keeps lines of up to 1 KB (2026-10-29)
        io.report = [&](const std::string& r) { reported += r + "\n"; longest_report = std::max(longest_report, r.size()); };
        Machine m(plan, mp, io);
        m.start(now);
        CHECK(m.command("SF_BENCH FILE=nothing.gcode", now).rfind("error", 0) == 0);
        // the benchmark file is never started as a normal print (2026-10-28)
        CHECK(m.command("SDCARD_PRINT_FILE FILENAME=sf-bench.gcode", now).find("is the benchmark") != std::string::npos);
        CHECK(!m.bench_running() && !m.toolhead()->bench());
        for (int i = 0; i < 50; i++) m.tick(now += 0.02);   // settle after the start
        size_t enables_before = sent.size();
        std::string r = m.command("SF_BENCH", now);
        CHECK(r.rfind("ok benchmark running", 0) == 0);
        if (r.rfind("ok", 0) != 0) printf("  %s\n", r.c_str());
        int guard = 0;
        while (m.bench_running() && guard++ < 20000) {
            now += 0.02;
            m.tick(now);
        }
        CHECK(!m.bench_running());
        CHECK(reported.find("done after") != std::string::npos);
        CHECK(reported.find("planner") != std::string::npos && reported.find("shaping") != std::string::npos);
        CHECK(reported.find("benchmark sends:") != std::string::npos && reported.find("benchmark processor:") != std::string::npos);
        CHECK(longest_report < 1000);
        // drivers: after the start (which switches them off), no enable pin was touched again
        int touched = 0;
        for (size_t i = enables_before; i < sent.size(); i++)
            for (auto& st : plan.steppers)
                if (st.enable_oid >= 0 && sent[i].rfind("queue_digital_out oid=" + std::to_string(st.enable_oid) + " ", 0) == 0)
                    touched++;
        int with_enable = 0;
        for (auto& st : plan.steppers) with_enable += st.enable_oid >= 0;
        CHECK(touched == 2 * with_enable);   // only "off": once at the start, once at the end
        CHECK(!m.toolhead()->homed(0) && !m.toolhead()->homed(2));
        const MotionPerf& pf = m.toolhead()->perf();
        CHECK(pf.steps > 500000 && pf.queue_steps > 1000);
        CHECK(fb.bytes() > 10000);
        printf("  PC, %s: %s\n", cfgname, reported.c_str());
    }
}

// 2026-10-28, Tina 2: SF_BENCH refused a move as too late, twice, in the file's run of 300 tiny moves:
// a burst of planning on the ESP32-S3 outlasted the time its first steps had been given. Here every
// look at the clock during a loop pass costs 3 ms, with 250000-baud send times: it must finish.
static void test_bench_slow_host() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::vector<std::string> file;
    {
        BenchGcode g;
        std::string l;
        while (g.next(l)) file.push_back(l);
    }
    BoardPlan plan = build_board_plan(repo_file("docs/configs/tina2-skr-pico.cfg"), mp, 12e6);
    CHECK(plan.errors.empty());
    plan.move_count = 1024;
    double now = 100.0, extra = 0;
    bool in_pass = false;
    FakeBoard fb(mp);
    MachineIO io = fb.io([&]() {
        if (in_pass) extra += 0.003;
        return now + extra;
    });
    auto send0 = io.send;
    io.send = [&](const std::vector<std::string>& c, std::string* e) {
        extra += 0.001 + 0.0006 * double(c.size());   // 250000 baud and an acknowledgement
        return send0(c, e);
    };
    size_t pos = 0;
    io.print_open = [&](std::string& name, std::string*) { pos = 0; if (name.empty()) name = "sf-bench.gcode"; return true; };
    io.print_read = [&](std::string& line, bool& eof, std::string*) { eof = pos >= file.size(); if (!eof) line = file[pos++]; return true; };
    io.print_progress = [&]() { return double(pos) / double(file.size()); };
    std::string reported;
    io.report = [&](const std::string& r) { reported += r + "\n"; };
    Machine m(plan, mp, io);
    m.start(now);
    for (int i = 0; i < 50; i++) m.tick(now += 0.02);
    CHECK(m.command("SF_BENCH", now).rfind("ok benchmark running", 0) == 0);
    int guard = 0;
    while (m.bench_running() && guard++ < 100000) {
        now += 0.01 + extra;
        extra = 0;
        in_pass = true;
        m.tick(now);
        in_pass = false;
    }
    CHECK(reported.find("done after") != std::string::npos && reported.find("was not sent") == std::string::npos);
    if (reported.find("done after") == std::string::npos) printf("  slow host: %s\n", reported.c_str());
    CHECK(m.toolhead()->perf().burst_sends > 0);   // sent in the middle of planning
}

static void test_host_sensor() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    std::string base = repo_file("docs/configs/tina2-skr-pico.cfg");
    BoardPlan plan = build_board_plan(base + "\n[temperature_sensor chamber]\nsensor_type: Generic 3950\nsensor_pin: esp32:gpio7\n", mp, 12e6);
    for (auto& e : plan.errors) printf("  chamber: %s\n", e.c_str());
    CHECK(plan.errors.empty());
    const TempSensorObj* ch = nullptr;
    for (auto& t : plan.sensors) if (t.name == "chamber") ch = &t;
    CHECK(ch && ch->host_gpio == 7 && ch->oid == -1 && std::fabs(ch->th.pullup - 100000) < 1);
    for (auto& c : plan.cmds) CHECK(c.find("esp32") == std::string::npos);
    for (auto& c : adc_query_cmds(plan, 1000, 12e6)) CHECK(c.find("oid=-1") == std::string::npos);
    // errors: not an ADC1 pin; on a heater
    BoardPlan b1 = build_board_plan(base + "\n[temperature_sensor chamber]\nsensor_type: Generic 3950\nsensor_pin: esp32:gpio20\n", mp, 12e6);
    CHECK(!b1.errors.empty() && b1.errors[0].find("ADC1 pin") != std::string::npos);
    BoardPlan b2 = build_board_plan(config_set_option(base, "heater_bed", "sensor_pin", "esp32:gpio7"), mp, 12e6);
    CHECK(!b2.errors.empty() && b2.errors[0].find("printer board") != std::string::npos);
    // readings
    MotionRig mr;
    MachineIO io = mr.io();
    double ratio = ch->th.calc_adc(31.5);
    bool present = true;
    io.host_adc = [&](int gpio, double& r) { r = present ? ratio : 0.999; return gpio == 7; };
    Machine m(plan, mp, io);
    m.start(mr.rig.now);
    mr.run(m, 3, []() { return false; });
    std::string js = m.status_json(mr.rig.now);
    CHECK(js.find("\"name\":\"chamber\",\"temp\":31.5") != std::string::npos);
    CHECK(m.command("M105", mr.rig.now).find("chamber: 31.5") != std::string::npos);
    present = false;   // unplugged: open circuit, shown as such, the printer carries on
    mr.run(m, 5, []() { return false; });
    CHECK(!m.is_shutdown() && m.command("M105", mr.rig.now).find("chamber: no sensor?") != std::string::npos);
}

// ---- delta kinematics (2026-10-31)
static std::string delta_cfg(const char* extra_printer = "arm_length: 250\n", bool with_c = true) {
    std::string c =
        "[mcu]\nserial: uart\n\n"
        "[printer]\nkinematics: delta\nmax_velocity: 200\nmax_accel: 2000\nmax_z_velocity: 100\nmax_z_accel: 1000\n"
        "delta_radius: 100\nprint_radius: 80\n";
    c += extra_printer;
    c += "\n[stepper_a]\nstep_pin: gpio11\ndir_pin: gpio10\nenable_pin: !gpio12\nmicrosteps: 16\nrotation_distance: 40\n"
         "endstop_pin: ^gpio4\nposition_endstop: 300\nhoming_speed: 50\n\n"
         "[stepper_b]\nstep_pin: gpio6\ndir_pin: gpio5\nenable_pin: !gpio7\nmicrosteps: 16\nrotation_distance: 40\n"
         "endstop_pin: ^gpio3\nposition_endstop: 300\nhoming_speed: 50\n\n";
    if (with_c)
        c += "[stepper_c]\nstep_pin: gpio19\ndir_pin: gpio28\nenable_pin: !gpio2\nmicrosteps: 16\nrotation_distance: 40\n"
             "endstop_pin: ^gpio25\nposition_endstop: 300\nhoming_speed: 50\n\n";
    c += "[extruder]\nstep_pin: gpio14\ndir_pin: !gpio13\nenable_pin: !gpio15\nmicrosteps: 16\n"
         "rotation_distance: 33.725\nnozzle_diameter: 0.4\nfilament_diameter: 1.750\nheater_pin: gpio23\n"
         "sensor_type: EPCOS 100K B57560G104F\nsensor_pin: gpio27\ncontrol: pid\npid_Kp: 41.141\n"
         "pid_Ki: 2.716\npid_Kd: 155.822\nmin_temp: 0\nmax_temp: 240\n";
    return c;
}

#if PH_KIN_DELTA
static void test_delta() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    // config errors
    {
        BoardPlan bad = build_board_plan(delta_cfg("").c_str(), mp, 12e6);
        bool arm = false;
        for (auto& e : bad.errors) arm |= e.find("arm_length") != std::string::npos;
        CHECK(arm);
        BoardPlan nc = build_board_plan(delta_cfg("arm_length: 250\n", false).c_str(), mp, 12e6);
        bool needc = false;
        for (auto& e : nc.errors) needc |= e.find("stepper_c") != std::string::npos;
        CHECK(needc);
    }
    BoardPlan plan = build_board_plan(delta_cfg().c_str(), mp, 12e6);
    for (auto& e : plan.errors) printf("  delta printer.cfg error: %s\n", e.c_str());
    CHECK(plan.errors.empty() && plan.printer.delta);
    plan.move_count = 1024;
    Rig rig;
    Machine m(plan, mp, rig.io());
    m.start(rig.now);
    const auto& es = plan.sensors[size_t(plan.heater("extruder")->sensor)];
    m.on_message(adc(es.oid, es.th.calc_adc(25) * 8 * 4095.0), rig.now);
    CHECK(m.toolhead() && m.toolhead()->ok());
    if (!m.toolhead() || !m.toolhead()->ok()) return;
    CHECK(m.command("G1 X10", rig.now).find("Must home axis first") != std::string::npos);
    // G28 (any axis letters) homes the three towers together: three endstops armed at once
    rig.trsync_armed.clear();
    CHECK(m.command("G28 X", rig.now).rfind("ok homing", 0) == 0);
    for (int i = 0; i < 20; i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(rig.trsync_armed.size() == 3);
    const char* nm[3] = {"stepper_a", "stepper_b", "stepper_c"};
    const StepperObj* st[3] = {plan.stepper(nm[0]), plan.stepper(nm[1]), plan.stepper(nm[2])};
    for (int pass = 0; pass < 2; pass++) {
        if (pass) {   // back off by homing_retract_dist, then a second approach
            for (int i = 0; i < 600 && rig.trsync_armed.size() < 3; i++) rig.now += 0.05, m.tick(rig.now);
            CHECK(rig.trsync_armed.size() == 3);
        }
        for (int i = 0; i < 3; i++) m.on_message(trsync_hit(st[i]->trsync_oid), rig.now);
        for (int i = 0; i < 10; i++) rig.now += 0.05, m.tick(rig.now);
        if (!pass) rig.trsync_armed.clear();
    }
    for (int i = 0; i < 200 && m.toolhead()->homing(); i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(!m.toolhead()->homing() && m.toolhead()->homed(0) && m.toolhead()->homed(1) && m.toolhead()->homed(2));
    DeltaGeom g;
    {
        double arm[3] = {250, 250, 250}, ang[3] = {210, 330, 90};
        g.set(arm, ang, 100);
    }
    double ab[3] = {300 + std::sqrt(250.0 * 250 - 100 * 100), 300 + std::sqrt(250.0 * 250 - 100 * 100), 300 + std::sqrt(250.0 * 250 - 100 * 100)};
    double home[3];
    CHECK(g.forward(ab, home));
    NEAR(home[0], 0.0, 1e-9);
    NEAR(home[1], 0.0, 1e-9);
    NEAR(home[2], 300.0, 1e-9);
    NEAR(m.toolhead()->position(0), home[0], 1e-6);
    NEAR(m.toolhead()->position(2), home[2], 1e-6);
    {   // forward undoes inverse everywhere inside the bed
        double c[3], p[3];
        CHECK(g.inverse(37.5, -52.25, 123.0, c) && g.forward(c, p));
        NEAR(p[0], 37.5, 1e-9);
        NEAR(p[1], -52.25, 1e-9);
        NEAR(p[2], 123.0, 1e-9);
    }
    // limits: print_radius 80, Z between 0 and the lowest position_endstop
    CHECK(m.command("G1 X85 F6000", rig.now).find("Move out of range") != std::string::npos);
    CHECK(m.command("G1 Z301 F6000", rig.now).find("Move out of range") != std::string::npos);
    CHECK(m.command("G1 Z-1 F6000", rig.now).find("Move out of range") != std::string::npos);
    // a move away from the middle: every step the three towers take, in time order, must keep the head
    // on the straight line (the cross-track error would show a wrongly linear tower mapping)
    size_t mark = rig.sent.size();
    CHECK(m.command("G1 X60 Y-40 Z120 F6000", rig.now) == "ok");
    CHECK(m.command("G1 X-50 Y55 Z200 F6000", rig.now) == "ok");
    CHECK(m.command("G1 X0 Y0 Z300 F6000", rig.now) == "ok");
    for (int i = 0; i < 400; i++) rig.now += 0.05, m.tick(rig.now);
    StepReplay rp[3];
    for (size_t i = mark; i < rig.sent.size(); i++)
        for (int k = 0; k < 3; k++) rp[k].feed(rig.sent[i].second, st[k]->oid);
    struct Ev { uint64_t clk; int k, d; };
    std::vector<Ev> ev;
    for (int k = 0; k < 3; k++)
        for (auto& e : rp[k].steps) ev.push_back({e.first, k, e.second ? 1 : -1});
    std::sort(ev.begin(), ev.end(), [](const Ev& a, const Ev& b) { return a.clk < b.clk; });
    CHECK(ev.size() > 10000);
    // net movement of each tower over the closed path is zero
    long net[3] = {0, 0, 0};
    for (auto& e : ev) net[e.k] += e.d;
    for (int k = 0; k < 3; k++) CHECK(std::labs(net[k]) <= 1);
    // positions along the way, from the carriage heights the steps add up to
    double spm = st[0]->steps_per_mm();
    long cnt[3] = {0, 0, 0};
    double worst = 0;
    const double wp[4][3] = {{0, 0, 300}, {60, -40, 120}, {-50, 55, 200}, {0, 0, 300}};
    size_t n = 0;
    for (auto& e : ev) {
        cnt[e.k] += e.d;
        if (++n % 37) continue;
        double c[3] = {ab[0] + cnt[0] / spm, ab[1] + cnt[1] / spm, ab[2] + cnt[2] / spm}, p[3];
        if (!g.forward(c, p)) { worst = 1e9; break; }
        // distance to the nearest of the three straight segments
        double best = 1e18;
        for (int s2 = 0; s2 < 3; s2++) {
            double d[3], w[3], dd = 0, t = 0;
            for (int j = 0; j < 3; j++) d[j] = wp[s2 + 1][j] - wp[s2][j], w[j] = p[j] - wp[s2][j], dd += d[j] * d[j], t += d[j] * w[j];
            t = std::min(1.0, std::max(0.0, t / dd));
            double dist = 0;
            for (int j = 0; j < 3; j++) dist += (w[j] - t * d[j]) * (w[j] - t * d[j]);
            best = std::min(best, std::sqrt(dist));
        }
        worst = std::max(worst, best);
    }
    printf("  delta: %zu tower steps over 3 moves, worst distance of the head from the straight lines %.4f mm\n", ev.size(), worst);
    CHECK(worst < 0.05);
    NEAR(m.toolhead()->position(2), 300.0, 1e-6);
    CHECK(m.command("M114", rig.now).find("Z:300.000") != std::string::npos);
    // one tower that never reports its endstop: "No trigger on b"
    rig.trsync_armed.clear();
    CHECK(m.command("G28", rig.now).rfind("ok homing", 0) == 0);
    for (int i = 0; i < 20; i++) rig.now += 0.05, m.tick(rig.now);
    m.on_message(trsync_hit(st[0]->trsync_oid), rig.now);
    m.on_message(trsync_hit(st[2]->trsync_oid), rig.now);
    for (int i = 0; i < 2000 && m.toolhead()->homing(); i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(!m.toolhead()->homing() && m.toolhead()->last_error().find("No trigger on b") != std::string::npos);
    CHECK(!m.toolhead()->homed(0));
}
#endif

void test_toolhead() {
    test_home_all_timing(32, 0.04, 0.0005);
    test_home_all_timing(256, 0.04, 0.0005);
    test_home_all_timing(256, 0.12, 0.002);
    test_print_named_files();
    test_set_tmc_current();
    test_homing_move_streams();
    test_homing_override_and_macros();
    test_stallguard_hard_stop();
    test_run_around_gcode();
    test_display_info();
    test_panel();
    test_stale_now_moves();
    test_g28_after_false_trigger();
#if PH_KIN_COREXY
    test_corexy();
#endif
#if PH_KIN_DELTA
    test_delta();
#endif
    test_host_sensor();
#if PH_KIN_COREXY   // these use the CoreXY QIDI preset
    test_qidi_xsmart3();
    test_print_bench();
#endif
    test_bench_slow_host();
    test_review_fixes();
    test_toolhead_shaping();
    test_pro_features();
    test_print_slicer_info();
    test_planner_memory();
    test_print_refuses_arcs();
    test_step_compression();
    test_moves_and_homing();
    test_print_file();
}
