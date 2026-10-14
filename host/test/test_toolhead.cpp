// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Moves, step compression, homing (sensorless), probing and printing a file (ph::Toolhead through
// ph::Machine) against a mock board, with the Tina 2 preset and the real SKR Pico dictionary.
#include "check.h"
#include "ph/boardcfg.h"
#include "ph/default_printer_cfg.h"
#include "ph/machine.h"
#include "ph/msgproto.h"
#include "ph/toolhead.h"
#include "skr_pico_dict.h"

#include <cmath>
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

static void test_print_refuses_arcs() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    Rig rig;
    rig.file = {"G90", "G2 X10 Y10 I5 J0", "G1 X1"};
    Machine m(plan, mp, rig.io());
    CHECK(m.command("SDCARD_PRINT_FILE", rig.now).rfind("ok", 0) == 0);
    for (int i = 0; i < 20 && m.printing(); i++) rig.now += 0.05, m.tick(rig.now);
    CHECK(!m.printing() && m.status_json(rig.now).find("arc fitting") != std::string::npos);
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
}

void test_toolhead() {
    test_planner_memory();
    test_print_refuses_arcs();
    test_step_compression();
    test_moves_and_homing();
    test_print_file();
}
