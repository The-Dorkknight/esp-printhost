// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Heaters, fans, TMC2209, motor tests and the console (ph::Machine) against a mock board, using the
// shipped SKR Pico firmware's real data dictionary and the shipped default printer.cfg.
// Reference values marked "Klipper" come from host/tools/klipper_tmc_reference.py, which runs
// Klipper v0.13.0's own TMC code.
#include "check.h"
#include "ph/boardcfg.h"
#include "ph/config_ini.h"
#include "ph/default_printer_cfg.h"
#include "ph/machine.h"
#include "ph/json_mini.h"
#include "ph/msgproto.h"
#include "skr_pico_dict.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace ph;

static std::string hex(const std::vector<uint8_t>& v) {
    static const char* h = "0123456789abcdef";
    std::string s;
    for (uint8_t b : v) s += h[b >> 4], s += h[b & 15];
    return s;
}

static std::vector<uint8_t> unhex(const std::string& s) {
    std::vector<uint8_t> v;
    for (size_t i = 0; i + 1 < s.size(); i += 2) v.push_back(uint8_t(strtol(s.substr(i, 2).c_str(), nullptr, 16)));
    return v;
}

static std::string arg(const std::string& cmd, const std::string& k) {
    size_t p = cmd.find(" " + k + "=");
    if (p == std::string::npos) return "";
    p += k.size() + 2;
    return cmd.substr(p, cmd.find(' ', p) - p);
}

// A pretend SKR Pico: records commands, answers TMC UART and stepper position queries.
struct MockBoard {
    double now = 100.0;
    std::vector<std::pair<double, std::string>> sent;
    std::map<int, std::map<int, uint32_t>> regs;   // TMC address -> register -> value
    std::map<int, uint32_t> ifcnt;
    std::set<int> dead;                             // TMC addresses that do not answer
    uint32_t drv_status = 0;
    std::vector<std::vector<uint8_t>> leds;

    MachineIO io() {
        MachineIO io;
        io.send = [this](const std::vector<std::string>& c, std::string*) {
            for (auto& s : c) sent.push_back({now, s});
            return true;
        };
        io.query = [this](const std::string& cmd, const std::string& resp, int oid, Message& out, int) {
            sent.push_back({now, cmd});
            out = Message();
            out.name = resp;
            out.params["oid"].i = oid;
            if (resp == "stepper_position") {
                out.params["pos"].i = 0;
                return true;
            }
            if (resp != "tmcuart_response") return false;
            // undo the serial bits to get the datagram
            std::vector<uint8_t> raw = unhex(arg(cmd, "write").substr(4)), d;
            for (size_t bit = 0; bit + 10 <= raw.size() * 8; bit += 10) {
                uint8_t b = 0;
                for (int i = 0; i < 8; i++)
                    if (raw[(bit + 1 + i) / 8] & (1u << ((bit + 1 + i) % 8))) b |= uint8_t(1u << i);
                d.push_back(b);
            }
            int addr = d.size() > 1 ? d[1] : -1;
            if (dead.count(addr)) return false;
            auto& p = out.params["read"];
            p.is_bytes = true;
            if (d.size() == 8) {   // write
                regs[addr][d[2] & 0x7f] = (uint32_t(d[3]) << 24) | (uint32_t(d[4]) << 16) | (uint32_t(d[5]) << 8) | d[6];
                ifcnt[addr]++;
                return true;
            }
            int reg = d[2];
            uint32_t v = reg == 0x06 ? 0x21000040u : reg == 0x02 ? ifcnt[addr] : reg == 0x6f ? drv_status : regs[addr][reg];
            auto e = tmc_encode_write(0x05, 0xff, uint8_t(reg), v);
            p.b.assign(e.begin(), e.end());
            return true;
        };
        io.clock_at = [](double t) { return uint64_t(t * 12e6); };
        io.neopixel_write = [this](int, const std::vector<uint8_t>& d, std::string*) {
            leds.push_back(d);
            return true;
        };
        return io;
    }
    std::vector<std::string> since(size_t from) const {
        std::vector<std::string> v;
        for (size_t i = from; i < sent.size(); i++) v.push_back(sent[i].second);
        return v;
    }
    bool has(const std::string& prefix, size_t from = 0) const {
        for (size_t i = from; i < sent.size(); i++)
            if (sent[i].second.rfind(prefix, 0) == 0) return true;
        return false;
    }
};

static Message adc_msg(int oid, double value) {
    Message m;
    m.name = "analog_in_state";
    m.params["oid"].i = oid;
    m.params["next_clock"].i = 0;
    m.params["value"].i = int64_t(value);
    return m;
}

static void test_tmc_wire() {
    // Klipper tmc_uart.py: _encode_read(0xf5, 0, IOIN) and _encode_write(0xf5, 3, 0x80, 0x1c0)
    CHECK(hex(tmc_encode_read(0xf5, 0, 0x06)) == "ea03c82094");
    CHECK(hex(tmc_encode_write(0xf5, 3, 0x80, 0x1c0)) == "ea1b083080000a08b8ca");
    uint32_t v = 0;
    std::vector<uint8_t> reply = unhex("0afacfa090000208a8a7");   // Klipper: reply IOIN 0x21000040
    CHECK(tmc_decode_read(0x06, reply, v) && v == 0x21000040u);
    reply[3] ^= 0x10;   // a damaged bit is rejected
    CHECK(!tmc_decode_read(0x06, reply, v));
    CHECK(!tmc_decode_read(0x06, std::vector<uint8_t>(9, 0), v));
}

static void test_default_cfg_plan() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    CHECK(strlen(default_printer_cfg()) < 4096);   // fits the NVS limit
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    for (auto& e : plan.errors) printf("  default printer.cfg error: %s\n", e.c_str());
    CHECK(plan.errors.empty());
    CHECK(plan.heaters.size() == 2 && plan.steppers.size() == 4 && plan.tmcs.size() == 4 && plan.fans.size() == 3);
    CHECK(plan.sensors.size() == 2 && plan.neopixels.size() == 1);
    // the heater outputs, as Klipper's mcu.py builds them: soft PWM, 0.1 s cycle, 5 s max_duration
    const HeaterObj* e = plan.heater("extruder");
    CHECK(e && e->out.pin == "gpio23" && e->out.cycle_ticks == 1200000);
    bool cfg_heater = false, cycle = false, stepper = false, trsync = false, enable = false, uart = false, fan = false;
    for (auto& c : plan.cmds) {
        cfg_heater |= c == "config_digital_out oid=" + std::to_string(e->out.oid) + " pin=gpio23 value=0 default_value=0 max_duration=60000000";
        cycle |= c == "set_digital_out_pwm_cycle oid=" + std::to_string(e->out.oid) + " cycle_ticks=1200000";
        stepper |= c.rfind("config_stepper oid=", 0) == 0 && c.find("step_pin=gpio11 dir_pin=gpio10 invert_step=0 step_pulse_ticks=24") != std::string::npos;
        trsync |= c.rfind("config_trsync oid=", 0) == 0;
        enable |= c.find("pin=gpio12 value=1 default_value=1 max_duration=0") != std::string::npos;   // !gpio12: off = high
        uart |= c.find("rx_pin=gpio9 pull_up=0 tx_pin=gpio8 bit_time=300") != std::string::npos;
        fan |= c.find("pin=gpio17 value=0 default_value=0 max_duration=0") != std::string::npos;
    }
    CHECK(cfg_heater && cycle && stepper && trsync && enable && uart && fan);
    int uarts = 0;
    for (auto& c : plan.cmds) uarts += c.rfind("config_tmcuart", 0) == 0;
    CHECK(uarts == 1);   // four drivers share one UART
    // every command must encode with the real firmware's formats
    for (auto& c : plan.cmds) {
        Bytes b;
        bool ok = mp.encode(c, b, &err);
        if (!ok) printf("  cannot encode: %s (%s)\n", c.c_str(), err.c_str());
        CHECK(ok);
    }
    // heater ADC range check from min_temp 0 / max_temp 250 (Klipper: int(min*max_adc), ceil(max*max_adc))
    const TempSensorObj& s = plan.sensors[size_t(e->sensor)];
    double full = 8 * 4095.0;
    CHECK(s.min_value == uint16_t(s.th.calc_adc(250) * full));
    CHECK(s.max_value == uint16_t(std::ceil(s.th.calc_adc(0) * full)));
    auto q = adc_query_cmds(plan, 1000, 12e6);
    CHECK(q.size() == 2 && q[0].find("min_value=" + std::to_string(s.min_value) + " max_value=" + std::to_string(s.max_value)) != std::string::npos);
    // TMC2209 registers: identical to Klipper's tmc2209.py for the same settings
    const TmcObj& x = plan.tmcs[size_t(plan.stepper("stepper_x")->tmc)];
    std::vector<std::pair<std::string, uint32_t>> want = {
        {"GCONF", 0x1c0}, {"SLAVECONF", 0x200}, {"CHOPCONF", 0x14030053}, {"IHOLD_IRUN", 0x81212}, {"TPWMTHRS", 0},
        {"TCOOLTHRS", 0}, {"COOLCONF", 0}, {"PWMCONF", 0xc80d0e24}, {"TPOWERDOWN", 0x14}, {"SGTHRS", 0}};
    CHECK(x.init_regs.size() == want.size());
    for (size_t i = 0; i < want.size() && i < x.init_regs.size(); i++)
        CHECK(x.init_regs[i].name == want[i].first && x.init_regs[i].value == want[i].second);
    // extruder at 0.65 A without stealthchop_threshold (Klipper: IHOLD_IRUN 0x81414, GCONF 0x1c4, TPWMTHRS 0xfffff)
    TmcObj t;
    t.run_current = 0.65, t.hold_current = 2.0;
    CHECK(tmc2209_registers(t, 16, 33.5, 200, 0.110, false, 0, true, &err));
    CHECK(t.init_regs[0].value == 0x1c4 && t.init_regs[3].value == 0x81414 && t.init_regs[4].value == 0xfffff);
    // mistakes are caught with line numbers
    BoardPlan bad = build_board_plan(
        "[extruder]\nheater_pin: gpio23\nsensor_type: Generic 3950\nsensor_pin: gpio23\ncontrol: pid\npid_Kp: 1\n"
        "pid_Ki: 1\npid_Kd: 1\nmin_temp: 0\nmax_temp: 250\nmax_temperature: 3\n"
        "[stepper_x]\nstep_pin: gpio11\ndir_pin: gpio10\nmicrosteps: 12\nrotation_distance: 40\nhoming_speed: 5\n"
        "[tmc2209 stepper_y]\nuart_pin: gpio9\nrun_current: 0.5\n",
        mp, 12e6);
    bool twice = false, typo = false, ms = false, nosec = false, homing = false;
    for (auto& er : bad.errors) {
        twice |= er.find("pin gpio23 is used twice") != std::string::npos;
        typo |= er.find("unknown option [extruder] max_temperature (line 11)") != std::string::npos;
        ms |= er.find("microsteps must be") != std::string::npos;
        nosec |= er.find("needs a [stepper_y] section") != std::string::npos;
        homing |= er.find("homing_speed") != std::string::npos;
    }
    CHECK(twice && typo && ms && nosec && !homing);   // homing_speed is a known Klipper option, not a typo
}

// Hotend model: full power raises it ~4 C/s, losses pull it back to 25 C.
struct Plant {
    double temp = 25, gain = 4.0, loss = 0.02, power = 0;
    void step(double dt) { temp += (power * gain - (temp - 25) * loss) * dt; }
};

static double last_power(const MockBoard& b, int oid, uint32_t cycle) {
    std::string key = "queue_digital_out oid=" + std::to_string(oid) + " ";
    for (size_t i = b.sent.size(); i-- > 0;)
        if (b.sent[i].second.rfind(key, 0) == 0) return double(strtoul(arg(b.sent[i].second, "on_ticks").c_str(), nullptr, 10)) / cycle;
    return 0;
}

static void test_heater_control() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    MockBoard b;
    Machine m(plan, mp, b.io());
    std::string r = m.start(b.now);
    CHECK(r.find("TMC2209: 4 ok") != std::string::npos);
    CHECK(b.regs[0][0x6c] == 0x14030053 && b.regs[3][0x10] == 0x81414);   // registers really written
    const HeaterObj& h = *plan.heater("extruder");
    const TempSensorObj& s = plan.sensors[size_t(h.sensor)];
    const HeaterObj& bed = *plan.heater("heater_bed");
    const TempSensorObj& bs = plan.sensors[size_t(bed.sensor)];
    double full = 8 * 4095.0;
    Plant p;
    // run the heat test: 75 C, hold 60 s
    auto run = [&](double secs) {
        double next_adc = b.now;
        for (double t = 0; t < secs; t += 0.05) {
            b.now += 0.05;
            if (b.now >= next_adc) {
                next_adc = b.now + 0.3;
                m.on_message(adc_msg(s.oid, s.th.calc_adc(p.temp) * full), b.now);
                m.on_message(adc_msg(bs.oid, bs.th.calc_adc(22) * full), b.now);
            }
            m.tick(b.now);
            p.power = last_power(b, h.out.oid, h.out.cycle_ticks);
            p.step(0.05);
        }
    };
    run(2);
    CHECK(m.command("TEST_HEATER HEATER=extruder TARGET=75 HOLD=60", b.now).rfind("ok", 0) == 0);
    size_t mark = b.sent.size();
    run(200);
    const Machine::Test* ht = nullptr;
    for (auto& t : m.tests()) if (t.id == "heat_extruder") ht = &t;
    CHECK(ht && ht->state == "passed");
    if (ht && ht->state != "passed") printf("  heat test: %s %s\n", ht->state.c_str(), ht->detail.c_str());
    CHECK(!m.is_shutdown());
    // the heater was refreshed well inside the board's 5 s watchdog while on
    double last = -1, worst = 0;
    std::string key = "queue_digital_out oid=" + std::to_string(h.out.oid) + " ";
    for (size_t i = mark; i < b.sent.size(); i++)
        if (b.sent[i].second.rfind(key, 0) == 0) {
            if (last >= 0 && b.sent[i].first - last > worst) worst = b.sent[i].first - last;
            last = b.sent[i].first;
            if (arg(b.sent[i].second, "on_ticks") == "0") break;   // heater switched off at the end
        }
    CHECK(worst > 0 && worst <= 0.75 * 5.0 + 0.5);
    CHECK(last_power(b, h.out.oid, h.out.cycle_ticks) == 0);   // off after the test
    // the heatbreak fan came on with the hot end (heater_fan above 50 C)
    bool fan_on = false;
    std::string fkey = "queue_digital_out oid=" + std::to_string(plan.fan("heatbreak_cooling_fan")->out.oid) + " ";
    for (auto& c : b.sent) fan_on |= c.second.rfind(fkey, 0) == 0 && arg(c.second, "on_ticks") != "0";
    CHECK(fan_on);
    // status JSON names the states the page colours by
    std::string js = m.status_json(b.now);
    CHECK(js.find("\"name\":\"extruder\"") != std::string::npos && js.find("\"state\":\"passed\"") != std::string::npos);

    // verify_heater: a heater that does not warm up shuts everything down (emergency_stop)
    MockBoard b2;
    Machine m2(plan, mp, b2.io());
    m2.start(b2.now);
    CHECK(m2.command("M104 S200", b2.now) == "ok extruder target 200");
    for (int i = 0; i < 400 && !m2.is_shutdown(); i++) {
        b2.now += 0.1;
        if (i % 3 == 0) m2.on_message(adc_msg(s.oid, s.th.calc_adc(25) * full), b2.now);
        m2.tick(b2.now);
    }
    CHECK(m2.is_shutdown() && m2.shutdown_reason().find("not heating at expected rate") != std::string::npos);
    CHECK(b2.has("emergency_stop"));
    CHECK(m2.command("M104 S50", b2.now).find("shut down") != std::string::npos);
    CHECK(m2.command("FIRMWARE_RESTART", b2.now).rfind("ok", 0) == 0 && m2.restart_requested());

    // above max_temp: shut down
    MockBoard b3;
    Machine m3(plan, mp, b3.io());
    m3.on_message(adc_msg(s.oid, s.th.calc_adc(260) * full), b3.now);
    m3.tick(b3.now + 1.1);
    CHECK(m3.is_shutdown() && m3.shutdown_reason().find("above max_temp") != std::string::npos);

    // the board's own shutdown message is understood
    MockBoard b4;
    Machine m4(plan, mp, b4.io());
    Message sd;
    sd.name = "shutdown";
    int64_t id = 0;
    CHECK(mp.enum_value("static_string_id", "ADC out of range", id));
    sd.params["static_string_id"].i = id;
    m4.on_message(sd, b4.now);
    CHECK(m4.is_shutdown() && m4.shutdown_reason().rfind("board shut down: ADC out of range", 0) == 0);
    CHECK(m4.shutdown_reason().find("heater log") != std::string::npos);
}

static void test_console_and_fans() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    MockBoard b;
    Machine m(plan, mp, b.io());
    m.start(b.now);
    const auto& s = plan.sensors[size_t(plan.heater("extruder")->sensor)];
    m.on_message(adc_msg(s.oid, s.th.calc_adc(24) * 8 * 4095.0), b.now);
    CHECK(m.command("M104 S300", b.now).find("out of range (0.0:250.0)") != std::string::npos);
    CHECK(m.command("SET_HEATER_TEMPERATURE HEATER=extruder TARGET=60", b.now) == "ok extruder target 60");
    CHECK(m.command("M105", b.now).find("extruder: 24.0") != std::string::npos);
    CHECK(m.command("TURN_OFF_HEATERS", b.now) == "ok heaters off");
    CHECK(m.command("G1 X10", b.now).find("Must home axis first: X") != std::string::npos);
    CHECK(m.command("SET_HEATER_TEMPERATURE HEATER=nope TARGET=1", b.now).find("no [nope]") != std::string::npos);
    CHECK(m.command("SET_LED LED=board_led RED=1", b.now) == "ok" && !b.leds.empty());
    CHECK(m.command("HELP", b.now).find("TEST_ALL") != std::string::npos);
    // fan kick start: full power for 0.1 s, then the asked speed (Klipper fan.py)
    const FanObj& f = *plan.fan("fan");
    size_t mark = b.sent.size();
    CHECK(m.command("M106 S128", b.now).rfind("ok fan 50 %", 0) == 0);
    CHECK(last_power(b, f.out.oid, f.out.cycle_ticks) == 1.0);
    b.now += 0.15;
    m.tick(b.now);
    NEAR(last_power(b, f.out.oid, f.out.cycle_ticks), 128.0 / 255.0, 0.001);
    CHECK(m.command("M107", b.now).rfind("ok fan 0 %", 0) == 0 && last_power(b, f.out.oid, f.out.cycle_ticks) == 0);
    (void)mark;
    // TMC: a driver that does not answer is reported, the others still work
    MockBoard b2;
    b2.dead.insert(3);
    Machine m2(plan, mp, b2.io());
    CHECK(m2.start(b2.now).find("3 ok, 1 not answering") != std::string::npos);
    CHECK(m2.status_json(b2.now).find("motor power") != std::string::npos);
    CHECK(m2.command("DUMP_TMC STEPPER=stepper_x", b2.now).find("IOIN: 0x21000040") != std::string::npos);
    // M112
    CHECK(m.command("M112", b.now).find("EMERGENCY STOP") != std::string::npos && m.is_shutdown() && b.has("emergency_stop"));
}

static void test_motor_test() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    MockBoard b;
    Machine m(plan, mp, b.io());
    m.start(b.now);
    const StepperObj& x = *plan.stepper("stepper_x");
    size_t mark = b.sent.size();
    std::string r = m.command("STEPPER_TEST STEPPER=stepper_x", b.now);
    CHECK(r.rfind("ok stepper_x: 20 rpm CW 20 s, CCW 20 s, then shake", 0) == 0);
    auto cmds = b.since(mark);
    std::string S = std::to_string(x.oid), T = std::to_string(x.trsync_oid);
    bool ts = false, stop = false, reset = false, en = false;
    int64_t fwd = 0, back = 0, dir = -1;
    bool cruise = false;
    for (auto& c : cmds) {
        Bytes enc;
        CHECK(mp.encode(c, enc, &err));
        ts |= c == "trsync_start oid=" + T + " report_clock=0 report_ticks=0 expire_reason=0";
        stop |= c == "stepper_stop_on_trigger oid=" + S + " trsync_oid=" + T;
        reset |= c.rfind("reset_step_clock oid=" + S + " clock=", 0) == 0;
        en |= c.rfind("queue_digital_out oid=" + std::to_string(x.enable_oid), 0) == 0 && arg(c, "on_ticks") == "0";   // !gpio12: on = low
        if (c.rfind("set_next_step_dir oid=" + S, 0) == 0) dir = atoi(arg(c, "dir").c_str());
        if (c.rfind("queue_step oid=" + S, 0) == 0) {
            int64_t n = atoll(arg(c, "count").c_str());
            (dir == 0 ? fwd : back) += n;   // dir_pin is inverted (!gpio10): forward = dir 0
            cruise |= arg(c, "interval") == "11250" && n == 21333;   // 20 rpm x 3200 steps/rev = 1066.7 steps/s
        }
    }
    CHECK(ts && stop && reset && en && cruise);
    CHECK(fwd == back && fwd > 21333);
    for (double t = 0; t < 70; t += 0.1) b.now += 0.1, m.tick(b.now);
    const Machine::Test* mt = nullptr;
    for (auto& t : m.tests()) if (t.id == "motor_stepper_x") mt = &t;
    CHECK(mt && mt->state == "passed" && mt->detail.find("back where it started") != std::string::npos);
    CHECK(last_power(b, x.enable_oid, 1) == 1.0);   // driver off again (pin high)
    // abort: the trsync stop clears the board's queue at once, and the driver is switched off
    CHECK(m.command("STEPPER_TEST STEPPER=stepper_y", b.now).rfind("ok", 0) == 0);
    b.now += 2;
    m.tick(b.now);
    mark = b.sent.size();
    CHECK(m.command("TEST_ABORT TEST=motor_stepper_y", b.now) == "ok motor_stepper_y stopped");
    CHECK(b.has("trsync_trigger oid=" + std::to_string(plan.stepper("stepper_y")->trsync_oid) + " reason=1", mark));
    for (auto& t : m.tests()) if (t.id == "motor_stepper_y") CHECK(t.state == "aborted");
    // buzz
    CHECK(m.command("STEPPER_BUZZ STEPPER=extruder", b.now).rfind("ok extruder: buzz", 0) == 0);
    CHECK(m.command("STEPPER_BUZZ STEPPER=extruder", b.now).find("already moving") != std::string::npos);
}

static void test_run_all() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    MockBoard b;
    Machine m(plan, mp, b.io());
    m.start(b.now);
    const HeaterObj& h = *plan.heater("extruder");
    const auto& s = plan.sensors[size_t(h.sensor)];
    const auto& bs = plan.sensors[size_t(plan.heater("heater_bed")->sensor)];
    Plant p;
    double next_adc = 0;
    m.on_message(adc_msg(s.oid, s.th.calc_adc(p.temp) * 8 * 4095.0), b.now);
    CHECK(m.command("TEST_ALL", b.now).rfind("ok running", 0) == 0);
    const Machine::Test* all = nullptr;
    for (auto& t : m.tests()) if (t.id == "all") all = &t;
    for (int i = 0; i < 400 * 20 && all->state == "running"; i++) {
        b.now += 0.05;
        if (b.now >= next_adc) {
            next_adc = b.now + 0.3;
            m.on_message(adc_msg(s.oid, s.th.calc_adc(p.temp) * 8 * 4095.0), b.now);
            m.on_message(adc_msg(bs.oid, bs.th.calc_adc(22) * 8 * 4095.0), b.now);
        }
        m.tick(b.now);
        p.power = last_power(b, h.out.oid, h.out.cycle_ticks);
        p.step(0.05);
    }
    CHECK(all->state == "passed");
    if (all->state != "passed") printf("  run all: %s %s\n", all->state.c_str(), all->detail.c_str());
    int passed = 0;
    for (auto& t : m.tests()) passed += t.state == "passed";
    // not part of "all": the bed heater, two PID tunes, two output checks, two resonance tests (CAPTAIN+)
    CHECK(passed == int(m.tests().size()) - 7);
}

static void test_pid_calibrate() {
    // config_set_option keeps the file as it is apart from the one line
    std::string cfg = "# top\n[extruder]\npid_Kp: 1 # old\nmax_temp: 250\n\n[heater_bed]\nmax_temp: 100\n";
    std::string c2 = config_set_option(cfg, "extruder", "pid_Kp", "22.500");
    CHECK(c2 == "# top\n[extruder]\npid_Kp: 22.500\nmax_temp: 250\n\n[heater_bed]\nmax_temp: 100\n");
    c2 = config_set_option(c2, "heater_bed", "pid_Ki", "1.000");
    CHECK(c2.find("[heater_bed]\nmax_temp: 100\npid_Ki: 1.000\n") != std::string::npos);
    CHECK(config_set_option("[a]\nx: 1\n", "b", "y", "2") == "[a]\nx: 1\n\n[b]\ny: 2\n");

    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    MockBoard b;
    MachineIO io = b.io();
    std::string stored = default_printer_cfg(), saved;
    io.config_text = [&]() { return stored; };
    io.save_config = [&](const std::string& t, std::string*) { saved = t; return true; };
    Machine m(plan, mp, io);
    m.start(b.now);
    const HeaterObj& h = *plan.heater("extruder");
    const auto& s = plan.sensors[size_t(h.sensor)];
    // a hotend with lag: the block heats, the thermistor follows a second behind
    double block = 25, sensor = 25, next_adc = 0;
    m.on_message(adc_msg(s.oid, s.th.calc_adc(sensor) * 8 * 4095.0), b.now);
    CHECK(m.command("PID_CALIBRATE HEATER=extruder TARGET=170", b.now).rfind("ok PID tuning extruder", 0) == 0);
    const Machine::Test* t = nullptr;
    for (auto& x : m.tests()) if (x.id == "pid_extruder") t = &x;
    for (int i = 0; i < 20 * 60 * 20 && t->state == "running"; i++) {
        b.now += 0.05;
        if (b.now >= next_adc) {
            next_adc = b.now + 0.3;
            m.on_message(adc_msg(s.oid, s.th.calc_adc(sensor) * 8 * 4095.0), b.now);
        }
        m.tick(b.now);
        double pw = last_power(b, h.out.oid, h.out.cycle_ticks);
        block += (pw * 3.0 - (block - 25) * 0.012) * 0.05;
        sensor += (block - sensor) * 0.05;   // 1 s lag
    }
    CHECK(t->state == "passed");
    printf("  pid tune: %s\n", t->detail.c_str());
    CHECK(!m.is_shutdown());
    CHECK(m.status_json(b.now).find("\"unsaved_pid\":true") != std::string::npos);
    CHECK(m.command("SAVE_CONFIG", b.now).rfind("ok saved the PID values of extruder", 0) == 0);
    CHECK(saved.find("pid_Kp: 21.527") == std::string::npos && saved.find("[extruder]") != std::string::npos);
    BoardPlan p2 = build_board_plan(saved, mp, 12e6);
    CHECK(p2.errors.empty());
    CHECK(p2.heater("extruder")->kp != plan.heater("heater_bed")->kp);
    // on this simulated hotend, the tuned values overshoot less than Klipper's sample values
    auto overshoot = [&](const std::string& cfgtext) {
        BoardPlan pl = build_board_plan(cfgtext, mp, 12e6);
        MockBoard bb;
        Machine mm(pl, mp, bb.io());
        const HeaterObj& hh = *pl.heater("extruder");
        const auto& ss = pl.sensors[size_t(hh.sensor)];
        double blk = 25, sen = 25, nx = 0, peak = 0;
        mm.on_message(adc_msg(ss.oid, ss.th.calc_adc(sen) * 8 * 4095.0), bb.now);
        mm.command("M104 S170", bb.now);
        for (int i = 0; i < 20 * 300; i++) {
            bb.now += 0.05;
            if (bb.now >= nx) nx = bb.now + 0.3, mm.on_message(adc_msg(ss.oid, ss.th.calc_adc(sen) * 8 * 4095.0), bb.now);
            mm.tick(bb.now);
            double pw = last_power(bb, hh.out.oid, hh.out.cycle_ticks);
            blk += (pw * 3.0 - (blk - 25) * 0.012) * 0.05;
            sen += (blk - sen) * 0.05;
            peak = std::max(peak, sen);
        }
        return peak - 170;
    };
    double before = overshoot(default_printer_cfg()), after = overshoot(saved);
    printf("  overshoot at 170 C: sample PID %.1f C, tuned %.1f C\n", before, after);
    CHECK(after < before);
    CHECK(m.command("HEATER_LOG", b.now).rfind("time_s,heater,temp_c,target_c,power,adc_raw,clock_error_ms\n", 0) == 0);
    CHECK(m.command("STEPPER_SHAKE STEPPER=stepper_x", b.now).rfind("ok stepper_x: shake", 0) == 0);
}

// The heater output check and the stuck-output detector, with a good and a "shorted MOSFET" heater.
static void test_output_check() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg(), mp, 12e6);
    auto run = [&](bool stuck, const std::string& cmd, double secs, Machine*& out_m, MockBoard& b) {
        static BoardPlan* pl;
        pl = &plan;
        out_m = new Machine(*pl, mp, b.io());
        const HeaterObj& h = *pl->heater("extruder");
        const auto& s = pl->sensors[size_t(h.sensor)];
        Plant p;
        double next_adc = 0;
        out_m->on_message(adc_msg(s.oid, s.th.calc_adc(p.temp) * 8 * 4095.0), b.now);
        std::string r = out_m->command(cmd, b.now);
        CHECK(r.rfind("ok", 0) == 0);
        bool shorted = false;
        for (double t = 0; t < secs && !out_m->is_shutdown(); t += 0.05) {
            b.now += 0.05;
            if (b.now >= next_adc) next_adc = b.now + 0.3, out_m->on_message(adc_msg(s.oid, s.th.calc_adc(p.temp) * 8 * 4095.0), b.now);
            out_m->tick(b.now);
            double pw = last_power(b, h.out.oid, h.out.cycle_ticks);
            if (stuck && pw > 0) shorted = true;   // the MOSFET fails short the first time it conducts
            p.power = shorted ? 1.0 : pw;
            p.step(0.05);
        }
    };
    auto state = [](Machine* m, const std::string& id) {
        for (auto& t : m->tests()) if (t.id == id) return t.state + " " + t.detail;
        return std::string();
    };
    MockBoard b1;
    Machine* m = nullptr;
    run(false, "HEATER_OUTPUT_TEST HEATER=extruder", 60, m, b1);
    CHECK(state(m, "out_extruder").rfind("passed", 0) == 0);
    printf("  output check, good heater: %s\n", state(m, "out_extruder").c_str());
    int ons = 0;   // five 1 s pulses at full power
    for (auto& c : b1.sent)
        ons += c.second.rfind("queue_digital_out oid=" + std::to_string(plan.heater("extruder")->out.oid) + " ", 0) == 0 &&
               arg(c.second, "on_ticks") == "1200000";
    CHECK(ons == 5);
    CHECK(!m->is_shutdown());
    delete m;
    MockBoard b2;
    run(true, "HEATER_OUTPUT_TEST HEATER=extruder", 60, m, b2);
    CHECK(state(m, "out_extruder").rfind("failed", 0) == 0);   // by the check itself or the stuck-output detector
    CHECK(m->status_json(b2.now).find("\"power_off_now\":true") != std::string::npos);
    delete m;
    // stuck output while holding a target: heats on with the output off -> shutdown, "switch off the power"
    MockBoard b3;
    run(true, "M104 S75", 300, m, b3);
    CHECK(m->is_shutdown() && m->shutdown_reason().find("HOTTER WITH ITS OUTPUT OFF") != std::string::npos);
    CHECK(b3.has("emergency_stop"));
    printf("  stuck output: %s\n", m->shutdown_reason().substr(0, 90).c_str());
    delete m;
    // a good heater holding 75 C never trips it
    MockBoard b4;
    run(false, "M104 S75", 300, m, b4);
    CHECK(!m->is_shutdown());
    delete m;
}

// 2026-10-14: setting a temperature during a heater test / PID tune: the reply must say what happened,
// and the heater must go to the new temperature (it did, but the messages read as if it was ignored).
static void test_new_target_stops_tests() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    CHECK(plan.errors.empty() && plan.fans.size() == 3 && plan.fan("fan3") && plan.fan("fan3")->out.pin == "gpio20");
    MockBoard b;
    Machine m(plan, mp, b.io());
    m.start(b.now);
    const HeaterObj& h = *plan.heater("extruder");
    const auto& s = plan.sensors[size_t(h.sensor)];
    m.on_message(adc_msg(s.oid, s.th.calc_adc(25) * 8 * 4095.0), b.now);
    auto state = [&](const std::string& id) {
        for (auto& x : m.tests()) if (x.id == id) return x.state + " " + x.detail;
        return std::string();
    };
    CHECK(m.command("TEST_HEATER HEATER=extruder TARGET=75", b.now).rfind("ok heating", 0) == 0);
    std::string r = m.command("M104 S230", b.now);
    CHECK(r == "ok extruder target 230 (the heater test was stopped)");
    CHECK(state("heat_extruder") == "aborted stopped: you set extruder to 230 C");
    CHECK(m.status_json(b.now).find("\"target\":230") != std::string::npos);
    CHECK(m.command("PID_CALIBRATE HEATER=extruder TARGET=200", b.now).rfind("ok PID tuning", 0) == 0);
    r = m.command("SET_HEATER_TEMPERATURE HEATER=extruder TARGET=210", b.now);
    CHECK(r == "ok extruder target 210 (the PID tune was stopped: no new PID values)");
    CHECK(state("pid_extruder") == "aborted stopped: you set extruder to 210 C");
    CHECK(m.status_json(b.now).find("\"target\":210") != std::string::npos);
    CHECK(m.command("M106 S255", b.now) == "ok fan 100 % (gpio17)");
    m.command("TURN_OFF_HEATERS", b.now);
}

// 2026-10-14: stallGuard threshold and rotation_distance from the Test bench, saved into printer.cfg.
static void test_calibration_commands() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(SKR_PICO_DICT, &err));
    BoardPlan plan = build_board_plan(default_printer_cfg_tina2(), mp, 12e6);
    MockBoard b;
    MachineIO io = b.io();
    std::string stored = default_printer_cfg_tina2(), saved;
    io.config_text = [&]() { return stored; };
    io.save_config = [&](const std::string& t, std::string*) { saved = t; return true; };
    Machine m(plan, mp, io);
    m.start(b.now);
    std::string r = m.command("SET_STALLGUARD STEPPER=stepper_x SGTHRS=70 SAVE=1", b.now);
    CHECK(r.rfind("ok saved driver_SGTHRS: 70", 0) == 0);
    size_t sec = saved.find("[tmc2209 stepper_x]");
    CHECK(sec != std::string::npos && saved.find("driver_SGTHRS: 70", sec) < saved.find("[stepper_y]"));
    CHECK(saved.find("driver_SGTHRS: 100", sec) > saved.find("[stepper_y]"));   // replaced, not added twice
    CHECK(m.command("SET_STALLGUARD STEPPER=extruder SGTHRS=70", b.now).find("not set up for sensorless") != std::string::npos);
    CHECK(m.command("SET_STALLGUARD STEPPER=stepper_x SGTHRS=300", b.now).rfind("error", 0) == 0);
    r = m.command("SET_ROTATION_DISTANCE STEPPER=stepper_y VALUE=31.5", b.now);
    CHECK(r.rfind("ok stepper_y rotation_distance 31.5000 (101.587 steps/mm) saved", 0) == 0);
    size_t ys = saved.find("[stepper_y]");
    CHECK(saved.find("rotation_distance: 31.5000", ys) < saved.find("[tmc2209 stepper_y]"));
    std::string js = m.status_json(b.now);
    CHECK(js.find("\"name\":\"stepper_x\"") != std::string::npos && js.find("\"rd\":32.0000,\"spm\":100.000,\"cur\":") != std::string::npos && js.find("\"sgthrs\":100") != std::string::npos);
    JVal jv;
    CHECK(json_parse(js, jv, &err));
    // a motor test drives the stepper directly: homing or jogging meanwhile is refused (it shut the
    // simulated board down with "Can't reset time when stepper active", 2026-10-14)
    CHECK(m.command("STEPPER_TEST STEPPER=stepper_x RPM=20 SECONDS=5", b.now).rfind("ok", 0) == 0);
    CHECK(m.command("G28 X", b.now).find("a motor test is running on stepper_x") != std::string::npos);
    CHECK(m.command("JOG AXIS=Y DIST=5", b.now).find("a motor test is running") != std::string::npos);
}

void test_machine() {
    test_new_target_stops_tests();
    test_calibration_commands();
    test_output_check();
    test_pid_calibrate();
    test_tmc_wire();
    test_default_cfg_plan();
    test_heater_control();
    test_console_and_fans();
    test_motor_test();
    test_run_all();
}
