// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// PC unit tests. Build: see host/CMakeLists.txt  ->  ./ph_tests
#include "check.h"
#include "ph/config_ini.h"
#include "ph/gcode.h"
#include "ph/motion.h"
#include "ph/msgproto.h"
#include "ph/wire.h"
#include <cmath>
#include <cstdio>
#include <string>

int g_fail = 0, g_pass = 0;

using namespace ph;

// Reference vectors generated with a Python copy of Klipper's msgproto.py.
static void test_vlq() {
    struct { int64_t v; Bytes b; } vec[] = {
        {0, {0}}, {95, {95}}, {96, {128, 96}}, {-32, {96}}, {-33, {255, 95}},
        {12345, {128, 224, 57}}, {-12345, {255, 159, 71}},
        {2147483647, {135, 255, 255, 255, 127}},
        {-2147483648LL, {248, 128, 128, 128, 0}},
    };
    for (auto& t : vec) {
        Bytes o; vlq_encode(o, t.v);
        CHECK(o == t.b);
        size_t pos = 0; int64_t d;
        CHECK(vlq_decode(o.data(), o.size(), pos, true, d) && d == t.v);
    }
    // unsigned 4e9 round-trips as %u
    Bytes o; vlq_encode(o, 4000000000LL);
    size_t pos = 0; int64_t d;
    CHECK(vlq_decode(o.data(), o.size(), pos, false, d) && d == 4000000000LL);
    // fuzz round-trip
    for (int64_t v = -70000; v < 70000; v += 7) {
        Bytes b; vlq_encode(b, v); size_t p = 0; int64_t r;
        if (!vlq_decode(b.data(), b.size(), p, true, r) || r != v) { CHECK(false); break; }
    }
    // truncated
    Bytes t = {0x80}; pos = 0;
    CHECK(!vlq_decode(t.data(), t.size(), pos, true, d));
}

static void test_frame() {
    Bytes content = {1, 0, 40};  // identify offset=0 count=40
    Bytes f = frame_encode(content, 3);
    Bytes want = {0x08, 0x13, 0x01, 0x00, 0x28, 0x7b, 0x52, 0x7e};
    CHECK(f == want);

    // decoder: garbage + 2 frames split across feeds + corrupted frame
    FrameDecoder dec;
    std::vector<FrameDecoder::Block> out;
    Bytes junk = {0x33, 0x99, 0x7e};
    dec.feed(junk.data(), junk.size(), out);
    Bytes two = f; Bytes g = frame_encode({5, 6, 7, 8}, 4);
    two.insert(two.end(), g.begin(), g.end());
    dec.feed(two.data(), 5, out);
    CHECK(out.empty());
    dec.feed(two.data() + 5, two.size() - 5, out);
    CHECK(out.size() == 2);
    if (out.size() == 2) {
        CHECK(out[0].seq == 3 && out[0].content == content);
        CHECK(out[1].seq == 4 && out[1].content == Bytes({5, 6, 7, 8}));
    }
    Bytes bad = f; bad[3] ^= 0xFF;
    out.clear();
    dec.feed(bad.data(), bad.size(), out);
    dec.feed(f.data(), f.size(), out);
    CHECK(out.size() == 1);
    CHECK(dec.crc_errors() == 1);
    // pure ACK (empty content)
    Bytes ack = frame_encode({}, 7);
    out.clear();
    dec.feed(ack.data(), ack.size(), out);
    CHECK(out.size() == 1 && out[0].content.empty() && out[0].seq == 7);
}

static const char* DICT = R"({
  "version": "v0.12.0-test",
  "config": {"CLOCK_FREQ": 16000000, "MCU": "atmega2560"},
  "enumerations": {
    "pin": {"PA0": [0, 8], "PB0": [8, 8], "PC5": 21},
    "spi_bus": {"spi": 0}
  },
  "commands": {
    "get_uptime": 5,
    "config_digital_out oid=%c pin=%u value=%c default_value=%c max_duration=%u": 12,
    "segq_push oid=%c n=%u x=%i y=%i z=%i e=%i flags=%c tag=%hu": 40,
    "debug_write order=%c addr=%u val=%u": 41,
    "set_name name=%s": 42
  },
  "responses": {
    "uptime high=%u clock=%u": 81,
    "segq_status oid=%c free=%hu executing_tag=%hu": 82,
    "shutdown clock=%u static_string_id=%hu": 83,
    "is_shutdown static_string_id=%hu": 84
  }
})";

static void test_msgproto() {
    MsgProto mp;
    std::string err;
    CHECK(mp.load_dictionary(DICT, &err));
    CHECK(mp.config_num("CLOCK_FREQ") == 16000000);
    CHECK(mp.config_str("MCU") == "atmega2560");
    CHECK(mp.version() == "v0.12.0-test");
    int64_t v;
    CHECK(mp.enum_value("pin", "PA7", v) && v == 7);
    CHECK(mp.enum_value("pin", "PB3", v) && v == 11);
    CHECK(mp.enum_value("pin", "PC5", v) && v == 21);
    CHECK(!mp.enum_value("pin", "PA8", v));
    CHECK(mp.enum_summary("pin") == "PA0..PA7, PB0..PB7, PC5");
    auto keys = mp.enum_keys("pin");
    CHECK(keys.size() == 17 && keys[0] == "PA0" && keys[7] == "PA7" && keys[8] == "PB0");
    CHECK(mp.enum_name_of("pin", 11) == "PB3");
    CHECK(mp.enum_name_of("pin", 999).empty());
    CHECK(mp.has_command("get_uptime") && !mp.has_command("reset"));
    MsgProto rp;  // rp2040-style: gpio0..gpio29, natural sort (gpio2 < gpio10)
    CHECK(rp.load_dictionary(R"({"enumerations":{"pin":{"gpio0":[0,30]},
        "static_string_id":{"Invalid oid type":3}},"commands":{},"responses":{}})"));
    CHECK(rp.enum_summary("pin") == "gpio0..gpio29");
    CHECK(rp.enum_keys("pin")[2] == "gpio2" && rp.enum_keys("pin")[10] == "gpio10");
    CHECK(rp.enum_name_of("static_string_id", 3) == "Invalid oid type");

    Bytes b;
    CHECK(mp.encode("config_digital_out oid=3 pin=PB3 value=0 default_value=0 max_duration=0", b, &err));
    CHECK(b == Bytes({12, 3, 11, 0, 0, 0}));

    b.clear();
    CHECK(mp.encode("segq_push oid=1 n=160000 x=10 y=-3 z=0 e=-40 flags=1 tag=7", b));
    Bytes want; vlq_encode(want, 40); vlq_encode(want, 1); vlq_encode(want, 160000);
    vlq_encode(want, 10); vlq_encode(want, -3); vlq_encode(want, 0);
    vlq_encode(want, -40); vlq_encode(want, 1); vlq_encode(want, 7);
    CHECK(b == want);

    b.clear();
    CHECK(!mp.encode("config_digital_out oid=3 pin=PZ9 value=0 default_value=0 max_duration=0", b, &err));
    CHECK(!mp.encode("nope oid=1", b, &err));
    CHECK(!mp.encode("get_uptime extra", b, &err));
    b.clear();
    CHECK(mp.encode("set_name name=ender3", b) && b.size() == 1 + 1 + 6);

    // decode two responses in one block
    Bytes c;
    vlq_encode(c, 82); vlq_encode(c, 1); vlq_encode(c, 120); vlq_encode(c, 9);
    vlq_encode(c, 81); vlq_encode(c, 0); vlq_encode(c, 3000000000LL);
    std::vector<Message> msgs;
    CHECK(mp.decode(c, msgs, &err));
    CHECK(msgs.size() == 2);
    if (msgs.size() == 2) {
        CHECK(msgs[0].name == "segq_status" && msgs[0].get("free") == 120
              && msgs[0].get("executing_tag") == 9);
        CHECK(msgs[1].name == "uptime" && msgs[1].get("clock") == 3000000000LL);
    }
    // identify_response always known
    Bytes id; vlq_encode(id, 0); vlq_encode(id, 0); id.push_back(3);
    id.push_back('a'); id.push_back('b'); id.push_back('c');
    msgs.clear();
    CHECK(mp.decode(id, msgs) && msgs.size() == 1 && msgs[0].params["data"].b.size() == 3);
    // unknown id
    Bytes u; vlq_encode(u, 99); msgs.clear();
    CHECK(!mp.decode(u, msgs, &err));
}

static const char* CFG = R"(
# Ender 3 on stock Creality 4.2.2 (STM32F103)
[mcu]
serial: /dev/ttyUSB0
baud: 250000
mcu_type: stm32f103   ; used for auto-flash

[stepper_x]
step_pin: PC2
dir_pin: PB9
rotation_distance: 40
microsteps: 16
position_max: 235

[stepper   Y]
step_pin: PB8

[extruder]
pressure_advance: 0.05
heater_pin: PA1
enabled: yes

[gcode_macro START]
gcode:
  G28
  G1 Z5
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# [extruder]
#*# pid_Kp = 1
)";

static void test_config() {
    Config c;
    CHECK(c.parse(CFG));
    auto m = c.section("mcu");
    CHECK(m && m->get("serial") == "/dev/ttyUSB0");
    CHECK(m && m->geti("baud", 115200) == 250000);
    CHECK(m && m->get("mcu_type") == "stm32f103");
    auto x = c.section("stepper_x");
    CHECK(x && x->getf("rotation_distance") == 40);
    CHECK(x && x->geti("microsteps", 16, 1, 256) == 16);
    CHECK(x && x->getf("position_min", 0) == 0);
    CHECK(c.has("stepper y"));
    CHECK(c.sections_prefixed("stepper").size() == 2);
    auto e = c.section("extruder");
    CHECK(e && std::fabs(e->getf("pressure_advance", 0) - 0.05) < 1e-12);
    CHECK(e && e->getb("enabled", false));
    CHECK(e && !e->has("pid_kp"));  // SAVE_CONFIG block ignored
    auto mac = c.section("gcode_macro start");
    CHECK(mac && mac->get("gcode") == "G28\nG1 Z5");
    CHECK(c.ok());

    // errors
    x->getf("missing_thing");
    x->geti("microsteps", 16, 1, 8);  // out of range
    CHECK(c.errors().size() == 2);
    auto unused = c.unused_keys();
    CHECK(!unused.empty());  // dir_pin, step_pin etc. never read

    Config bad;
    CHECK(!bad.parse("key: outside\n[ok]\nnot a pair\n[include foo.cfg]\n"));
    CHECK(bad.errors().size() == 3);
}

static void test_gcode() {
    GLine g;
    CHECK(gcode_parse("G1 X10.5 Y-3 E0.25 F1800 ; move", g));
    CHECK(g.is('G', 1) && g.get('X') == 10.5f && g.get('Y') == -3.f);
    CHECK(g.has('E') && g.has('F') && !g.has('Z'));
    CHECK(gcode_parse("g28 x y", g) && g.is('G', 28) && g.has('X') && g.has('Y'));
    CHECK(gcode_parse("M104 S210 (hotend)", g) && g.is('M', 104) && g.get('S') == 210);
    CHECK(gcode_parse("M117 Printing 50% ;c", g) && g.text_arg == "Printing 50%");
    CHECK(gcode_parse("M23 benchy.gcode", g) && g.text_arg == "benchy.gcode");
    CHECK(gcode_parse("; just a comment", g) && g.cmd_letter == 0);
    CHECK(gcode_parse("", g) && g.cmd_letter == 0);
    CHECK(gcode_parse("G29.1", g) && g.cmd_num == 29 && g.cmd_sub == 1);
    CHECK(gcode_parse("N3 T0*58", g) && g.line_no == 3 && g.is('T', 0) && !g.checksum_ok);
    // correct checksum for "N3 T0": xor
    unsigned cs = 0; for (char ch : std::string("N3 T0")) cs ^= uint8_t(ch);
    CHECK(gcode_parse("N3 T0*" + std::to_string(cs), g) && g.checksum_ok);
    CHECK(!gcode_parse("G1 X1.2.3", g));
    CHECK(!gcode_parse("G1 X5 $", g));
    CHECK(gcode_parse("G1 X.5 E-.8", g) && g.get('X') == 0.5f && g.get('E') == -0.8f);
}

static void test_steps() {
    StepAccumulator a;
    a.set_steps_per_mm(AX_X, 80);
    int64_t total = 0;
    for (int i = 0; i < 1000; i++) total += a.add(AX_X, 0.0137);  // 13.7 mm
    CHECK(total == 1096);                                         // 13.7*80
    CHECK(a.committed_steps(AX_X) == 1096);
    for (int i = 0; i < 1000; i++) total += a.add(AX_X, -0.0137);
    CHECK(total == 0);
    a.set_position(AX_X, 100);
    CHECK(a.committed_steps(AX_X) == 8000 && a.add(AX_X, 0) == 0);
}

// Trapezoid move: accel -> cruise -> decel, check PA conserves total E.
static void run_move(PressureAdvance& pa, double len, double e_len, double vmax,
                     double acc, double dt, double& e_out, double& e_base,
                     double& peak_extra) {
    double v = 0, pos = 0;
    while (pos < len - 1e-9) {
        double stop_dist = v * v / (2 * acc);
        double nv = (len - pos <= stop_dist) ? std::max(v - acc * dt, 0.0)
                                             : std::min(v + acc * dt, vmax);
        double d = 0.5 * (v + nv) * dt;
        if (d <= 0) { d = len - pos; nv = 0; }        // creep to end
        if (pos + d > len) { d = len - pos; nv = 0; }
        double e = e_len * d / len;
        double o = pa.process({d, e, nv, dt});
        peak_extra = std::max(peak_extra, o - e);
        e_out += o; e_base += e;
        pos += d; v = nv;
    }
}

static void test_pressure_advance() {
    const double dt = 0.005;
    // K=0: exact pass-through
    PressureAdvance pa0; pa0.configure(0);
    double eo = 0, eb = 0, pk = 0;
    run_move(pa0, 50, 2.0, 100, 2000, dt, eo, eb, pk);
    NEAR(eo, eb, 1e-9);

    // K=0.05: extra during accel, total conserved after flush
    PressureAdvance pa; pa.configure(0.05, 4, 60);
    eo = eb = pk = 0;
    run_move(pa, 50, 2.0, 100, 2000, dt, eo, eb, pk);
    run_move(pa, 30, 1.2, 80, 2000, dt, eo, eb, pk);
    eo += pa.flush();
    NEAR(eo, eb, 1e-9);
    CHECK(pk > 0);

    // travel segment (no E) pulls advance back to zero
    PressureAdvance pb; pb.configure(0.05, 1, 1000);
    double first = pb.process({1.0, 0.04, 100, dt});   // advance +0.2
    NEAR(first, 0.04 + 0.05 * 0.04 * 100, 1e-12);
    double travel = pb.process({1.0, 0.0, 100, dt});
    NEAR(travel, -0.2, 1e-12);

    // clamp: tiny max speed carries debt forward but conserves total
    PressureAdvance pc; pc.configure(0.5, 1, 5.0);  // 5 mm/s -> 0.025 mm/seg
    double sum = 0, base = 0;
    for (int i = 0; i < 20; i++) {
        double o = pc.process({0.5, 0.02, 100, dt});
        CHECK(o <= 0.025 + 1e-12);
        sum += o; base += 0.02;
    }
    for (int i = 0; i < 400; i++) sum += pc.process({0, 0, 0, dt});
    NEAR(sum, base, 1e-9);
}

int main() {
    test_vlq();
    test_frame();
    test_msgproto();
    test_config();
    test_gcode();
    test_steps();
    test_pressure_advance();
    test_segq_core();
    test_engine();
    test_kboard();
    test_clocksync();
    test_boardcfg();
    test_thermistor();
    test_temperature_sensors();
    test_machine();
    test_toolhead();
    test_multipart();
    test_shaping();
    test_resonance();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
