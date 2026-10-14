// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Board configuration helpers (the end-to-end part runs against fake_mcu.py in test_klink.py).
#include "check.h"
#include "ph/boardcfg.h"
#include "ph/clocksync.h"
#include "ph/gcode.h"
#include "ph/kboard.h"
#include "ph/msgproto.h"
#include "ph/thermistor.h"

using namespace ph;

void test_kboard() {
    // zlib CRC-32 check value
    CHECK(crc32_zlib("123456789") == 0xCBF43926u);
    CHECK(crc32_zlib("") == 0u);
    // Klipper neopixel.py timing on a 12 MHz RP2040: 4 us -> 48 ticks, 50 us -> 600 ticks
    CHECK(neopixel_config_cmd(0, "gpio24", 3, 12e6) ==
          "config_neopixel oid=0 pin=gpio24 data_size=3 bit_max_ticks=48 reset_min_ticks=600");
    // colour order
    CHECK(led_bytes("GRB", 10, 20, 30) == std::vector<uint8_t>({20, 10, 30}));
    CHECK(led_bytes("RGB", 10, 20, 30) == std::vector<uint8_t>({10, 20, 30}));
    CHECK(led_bytes("GRBW", 1, 2, 3).size() == 4);
    // binary byte arguments: data=hex:...
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(R"({"enumerations":{"pin":{"gpio0":[0,30]}},
        "commands":{"neopixel_update oid=%c pos=%hu data=%*s":53},"responses":{}})", &err));
    Bytes b;
    CHECK(mp.encode("neopixel_update oid=0 pos=0 data=hex:ff0020", b, &err));
    CHECK(b == Bytes({53, 0, 0, 3, 0xff, 0x00, 0x20}));
    Bytes b2;
    CHECK(!mp.encode("neopixel_update oid=0 pos=0 data=hex:fff", b2, &err));   // odd length
    CHECK(!mp.encode("neopixel_update oid=0 pos=0 data=hex:zz", b2, &err));    // not hex
}

static const char* RP_DICT = R"({"enumerations":{"pin":{"gpio0":[0,30]}},
    "commands":{"config_neopixel oid=%c pin=%u data_size=%hu bit_max_ticks=%u reset_min_ticks=%u":54,
                "neopixel_update oid=%c pos=%hu data=%*s":53,"neopixel_send oid=%c":52},
    "responses":{},"config":{"CLOCK_FREQ":12000000}})";

static const char* RP_ADC_DICT = R"({"enumerations":{"pin":{"gpio0":[0,30]}},
    "commands":{"config_neopixel oid=%c pin=%u data_size=%hu bit_max_ticks=%u reset_min_ticks=%u":54,
                "config_analog_in oid=%c pin=%u":32,
                "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c":31},
    "responses":{"analog_in_state oid=%c next_clock=%u value=%hu":-16},
    "config":{"CLOCK_FREQ":12000000,"MCU":"rp2040","ADC_MAX":4095}})";

void test_thermistor() {
    // Klipper's own fit points must come back: temperature from the ADC fraction a pullup of
    // 4700 ohm gives at that resistance.
    Thermistor g;
    CHECK(thermistor_preset("Generic 3950", g));
    struct P { double t, r; } gp[] = {{25, 100000}, {150, 1770}, {250, 230}};
    for (auto& p : gp) NEAR(g.calc_temp(p.r / (4700 + p.r)), p.t, 0.01);
    Thermistor e;
    CHECK(thermistor_preset("EPCOS 100K B57560G104F", e));
    NEAR(e.calc_temp(100000.0 / 104700.0), 25.0, 0.01);
    NEAR(e.calc_temp(1641.9 / (4700 + 1641.9)), 150.0, 0.01);
    NEAR(e.calc_temp(226.15 / (4700 + 226.15)), 250.0, 0.01);
    Thermistor a;
    CHECK(thermistor_preset("ATC Semitec 104GT-2", a));
    NEAR(a.calc_temp(126800.0 / (4700 + 126800.0)), 20.0, 0.01);
    NEAR(a.calc_temp(80.65 / (4700 + 80.65)), 300.0, 0.01);
    // beta model: exact at its own point, and the usual shape elsewhere
    Thermistor b;
    CHECK(thermistor_preset("NTC 100K MGB18-104F39050L32", b));
    NEAR(b.calc_temp(100000.0 / 104700.0), 25.0, 0.01);
    CHECK(b.calc_temp(0.5) > 105 && b.calc_temp(0.5) < 115);   // 4.7k: beta 4100 gives 110 C
    // 18 C room temperature on a 100k 3950 reads about 0.97 of full scale
    double adc18 = g.calc_adc(18.0);
    CHECK(adc18 > 0.96 && adc18 < 0.985);
    // round trip over the working range, for the Steinhart-Hart and the beta sensors
    for (double t = 0; t <= 300; t += 10) {
        NEAR(g.calc_temp(g.calc_adc(t)), t, 0.05);
        NEAR(e.calc_temp(e.calc_adc(t)), t, 0.05);
        NEAR(b.calc_temp(b.calc_adc(t)), t, 0.05);
    }
    // temperature falls as the ADC reading rises; the clamps keep open / short finite
    CHECK(g.calc_temp(0.9) < g.calc_temp(0.5));
    CHECK(std::isfinite(g.calc_temp(0.0)) && std::isfinite(g.calc_temp(1.0)));
    Thermistor none;
    CHECK(!thermistor_preset("PT1000", none));
    // an inline resistor shifts the reading
    Thermistor in = g;
    in.inline_resistor = 100;
    CHECK(in.calc_temp(0.9) != g.calc_temp(0.9));
    NEAR(in.calc_temp(in.calc_adc(100)), 100.0, 0.05);
    // impossible input is refused
    Thermistor bad;
    CHECK(!bad.setup_points(25, 0, 150, 1770, 250, 230));
    CHECK(!bad.setup_beta(25, 100000, 0));
}

void test_temperature_sensors() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(RP_ADC_DICT, &err));
    const char* cfg =
        "[mcu]\nserial: uart\n"
        "[temperature_sensor hotend]\nsensor_type: Generic 3950\nsensor_pin: gpio27\n"
        "min_temp: 0\nmax_temp: 280\n"
        "[temperature_sensor bed]\nsensor_type: my probe\nsensor_pin: mcu:gpio26\npullup_resistor: 10000\n"
        "[temperature_sensor mcu_temp]\nsensor_type: temperature_mcu\n"
        "[thermistor my probe]\ntemperature1: 25\nresistance1: 100000\nbeta: 3950\n"
        "[neopixel led]\npin: gpio24\n";
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    CHECK(plan.errors.empty());
    CHECK(plan.sensors.size() == 2);   // the temperature_mcu one is skipped with a note
    CHECK(plan.sensors[0].name == "hotend" && plan.sensors[0].pin == "gpio27");
    CHECK(plan.sensors[1].name == "bed" && plan.sensors[1].th.pullup == 10000.0);
    CHECK(plan.sensors[0].th.pullup == 4700.0);
    CHECK(plan.adc_full_scale == 8.0 * 4095);
    bool note = false, listed = false;
    for (auto& n : plan.notes) {
        note |= n.find("temperature_mcu") != std::string::npos && n.find("not supported yet") != std::string::npos;
        listed |= n.find("[thermistor") != std::string::npos || n.find("[temperature_sensor] ") != std::string::npos;
    }
    CHECK(note && !listed);
    // neopixels get the first oids (the order the commands are sent in matters for the CRC)
    CHECK(plan.neopixels.size() == 1 && plan.neopixels[0].oid == 0);
    CHECK(plan.sensors[0].oid == 1 && plan.sensors[1].oid == 2);
    CHECK(plan.cmds.size() == 3 && plan.cmds[1] == "config_analog_in oid=1 pin=gpio27" &&
          plan.cmds[2] == "config_analog_in oid=2 pin=gpio26");
    // init commands are not part of the config; 1 ms samples x 8, report every 0.3 s at 12 MHz
    auto q = adc_query_cmds(plan, 1000000, 12e6);
    CHECK(q.size() == 2);
    CHECK(q[0] == "query_analog_in oid=1 clock=1000000 sample_ticks=12000 sample_count=8 rest_ticks=3600000 "
                  "min_value=0 max_value=65535 range_check_count=4");
    CHECK(q[1].find("oid=2 clock=1120000 ") != std::string::npos);   // staggered by 10 ms
    Bytes tmp;
    for (auto& c : q) CHECK(mp.encode(c, tmp, &err));
    for (auto& c : plan.cmds) CHECK(mp.encode(c, tmp, &err));
    // readings: a 100k 3950 at 18 C, as the board would report it (sum of 8 samples)
    CHECK(plan.sensors[0].describe() == "waiting for the first reading");
    Message m;
    m.name = "analog_in_state";
    uint16_t v = uint16_t(plan.sensors[0].th.calc_adc(18.0) * plan.adc_full_scale + 0.5);
    m.params["oid"].i = 1;
    m.params["value"].i = v;
    CHECK(apply_adc_message(plan, m));
    NEAR(plan.sensors[0].temp_c, 18.0, 0.05);
    CHECK(plan.sensors[0].state() == 1 && plan.sensors[0].describe() == "18.0 \xC2\xB0" "C");
    m.params["oid"].i = 9;
    CHECK(!apply_adc_message(plan, m));
    // open circuit (pulled to full scale) and short (zero) are called what they are
    m.params["oid"].i = 2;
    m.params["value"].i = 8 * 4095;
    CHECK(apply_adc_message(plan, m) && plan.sensors[1].state() == 2);
    CHECK(plan.sensors[1].describe().find("no sensor") != std::string::npos);
    m.params["value"].i = 3;
    CHECK(apply_adc_message(plan, m) && plan.sensors[1].state() == 3);
    // min/max warnings
    m.params["oid"].i = 1;
    m.params["value"].i = uint16_t(plan.sensors[0].th.calc_adc(300.0) * plan.adc_full_scale);
    apply_adc_message(plan, m);
    CHECK(plan.sensors[0].describe().find("above max_temp") != std::string::npos);
    // errors: not an ADC pin on the rp2040, unknown pin, typo, bad custom thermistor, missing options
    BoardPlan bad = build_board_plan(
        "[temperature_sensor a]\nsensor_type: Generic 3950\nsensor_pin: gpio5\n"
        "[temperature_sensor b]\nsensor_type: Generic 3950\nsensor_pin: PA3\n"
        "[temperature_sensor c]\nsensor_type: Generic 3950\nsensor_pin: gpio26\npullup: 4700\n"
        "[temperature_sensor d]\nsensor_pin: gpio27\n"
        "[thermistor t]\ntemperature1: 25\nresistance1: 0\nbeta: 3950\n", mp, 12e6);
    bool adc_pin = false, unknown_pin = false, typo = false, missing_type = false, bad_th = false;
    for (auto& e : bad.errors) {
        adc_pin |= e.find("'gpio5' is not an ADC pin") != std::string::npos && e.find("line 3") != std::string::npos;
        unknown_pin |= e.find("'PA3' is not a pin on this board") != std::string::npos;
        typo |= e.find("unknown option [temperature_sensor c] pullup") != std::string::npos;
        missing_type |= e.find("missing required option 'sensor_type'") != std::string::npos;
        bad_th |= e.find("do not describe a thermistor") != std::string::npos;
    }
    CHECK(adc_pin && unknown_pin && typo && missing_type && bad_th);
    // a board without ADC support says so
    MsgProto noadc;
    CHECK(noadc.load_dictionary(RP_DICT, &err));
    BoardPlan na = build_board_plan("[temperature_sensor a]\nsensor_type: Generic 3950\nsensor_pin: gpio0\n", noadc, 12e6);
    bool no_adc = false;
    for (auto& e : na.errors) no_adc |= e.find("no ADC") != std::string::npos;
    CHECK(no_adc);
}

void test_clocksync() {
    // A board whose crystal runs 50 ppm fast, sampled once a second with a few ms of jitter,
    // across several 32-bit wraps (12 MHz wraps every ~358 s).
    const double nominal = 12e6, real = 12e6 * (1 + 50e-6);
    ClockSync cs;
    uint64_t start = 0xFFFFF000ull;   // just below a wrap
    cs.reset(nominal, start, 100.0);
    unsigned seed = 1;
    for (int i = 1; i <= 1200; i++) {
        seed = seed * 1103515245u + 12345u;
        double jitter = double((seed >> 16) % 4000) / 1e6;           // 0..4 ms extra on the way back
        double sent = 100.0 + i, recv = sent + 0.0015 + jitter;
        double board_read = sent + 0.0007;                          // when the board read its clock
        uint64_t clk = start + uint64_t((board_read - 100.0) * real);
        cs.add_sample(sent, recv, uint32_t(clk));
    }
    CHECK(cs.ready());
    NEAR(cs.drift_ppm(), 50.0, 3.0);                 // measured drift within a few ppm
    CHECK(cs.last_clock() > (1ull << 32) * 3);      // extended across wraps, not stuck at 32 bits
    double t = 100.0 + 1200.5;
    double expect = double(start) + (t - 100.0) * real;
    NEAR(double(cs.clock_at(t)), expect, 12e6 * 0.003);   // within ~3 ms
    NEAR(cs.time_of(cs.clock_at(t)), t, 0.0005);
    // a slow round trip is rejected once there is history
    CHECK(!cs.add_sample(2000.0, 2000.5, uint32_t(start + uint64_t((1900.25) * real))));
    CHECK(cs.rejected() >= 1);
    // extend() picks the nearest 64-bit value, forwards or backwards
    ClockSync w;
    w.reset(1e6, 0x1FFFFFFF0ull, 0);
    CHECK(w.extend(0x00000010u) == 0x200000010ull);
    CHECK(w.extend(0xFFFFFF00u) == 0x1FFFFFF00ull);
}

void test_boardcfg() {
    std::string name;
    std::map<std::string, std::string> p;
    CHECK(gcode_parse_ext("set_led led=board_led RED=0.5 green=1 ; comment", name, p));
    CHECK(name == "SET_LED" && p["LED"] == "board_led" && p["RED"] == "0.5" && p["GREEN"] == "1" && p.size() == 3);
    CHECK(!gcode_parse_ext("SET_LED LED", name, p));
    CHECK(!gcode_parse_ext("   ", name, p));

    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(RP_DICT, &err));
    const char* cfg =
        "[mcu]\nserial: /dev/ttyAMA0\n"
        "[bed_mesh]\nspeed: 100\n"
        "[neopixel board_led]\npin: gpio24\nchain_count: 1\ncolor_order: GRB\n"
        "initial_GREEN: 0.2\n"
        "[neopixel strip]\npin: mcu:gpio5\nchain_count: 3\ncolor_order: RGBW\n";
    BoardPlan plan = build_board_plan(cfg, mp, 12e6);
    CHECK(plan.errors.empty());
    CHECK(plan.oid_count == 2 && plan.neopixels.size() == 2);
    CHECK(plan.cmds.size() == 2);
    CHECK(plan.cmds[0] == "config_neopixel oid=0 pin=gpio24 data_size=3 bit_max_ticks=48 reset_min_ticks=600");
    CHECK(plan.cmds[1] == "config_neopixel oid=1 pin=gpio5 data_size=12 bit_max_ticks=48 reset_min_ticks=600");
    CHECK(plan.neopixels[0].data == std::vector<uint8_t>({51, 0, 0}));   // GRB, green 0.2 -> 51
    CHECK(plan.notes.size() == 1 && plan.notes[0] == "[bed_mesh] not used yet");
    // SET_LED: all LEDs, then one by INDEX
    p = {{"LED", "strip"}, {"RED", "1"}, {"WHITE", "0.5"}};
    std::string msg;
    NeopixelObj* n = apply_set_led(plan, p, msg);
    CHECK(n && n->oid == 1);
    CHECK(n->data == std::vector<uint8_t>({255, 0, 0, 128, 255, 0, 0, 128, 255, 0, 0, 128}));
    p = {{"LED", "strip"}, {"BLUE", "1"}, {"INDEX", "2"}};
    n = apply_set_led(plan, p, msg);
    CHECK(n && n->data == std::vector<uint8_t>({255, 0, 0, 128, 0, 0, 255, 0, 255, 0, 0, 128}));
    p = {{"LED", "strip"}, {"INDEX", "4"}};
    CHECK(!apply_set_led(plan, p, msg) && msg.find("INDEX") != std::string::npos);
    p = {{"LED", "nope"}};
    CHECK(!apply_set_led(plan, p, msg));
    p = {{"LED", "strip"}, {"RED", "2"}};
    CHECK(!apply_set_led(plan, p, msg));
    // errors carry line numbers and stop the plan
    BoardPlan bad = build_board_plan("[neopixel x]\npin: PA5\ncolor_order: RGGB\nchian_count: 2\n", mp, 12e6);
    CHECK(bad.errors.size() == 3);
    bool pin_line = false, order_line = false, typo = false;
    for (auto& e : bad.errors) {
        pin_line |= e.find("pin 'PA5' is not a pin on this board (line 2)") != std::string::npos;
        order_line |= e.find("color_order 'RGGB'") != std::string::npos && e.find("line 3") != std::string::npos;
        typo |= e.find("chian_count") != std::string::npos;
    }
    CHECK(pin_line && order_line && typo);
    BoardPlan missing = build_board_plan("[neopixel y]\nchain_count: 1\n", mp, 12e6);
    CHECK(missing.errors.size() == 1 && missing.errors[0].find("missing required option 'pin'") != std::string::npos);
}
