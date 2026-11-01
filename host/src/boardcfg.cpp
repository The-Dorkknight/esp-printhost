// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/boardcfg.h"
#include "ph/delta.h"
#include "ph/shaping.h"   // PH_PRO

#include <sstream>
#include <cctype>

#include "ph/config_ini.h"
#include "ph/gcode.h"

#include <cmath>
#include <cstdlib>
#include <set>

namespace ph {

static const int MAX_NEOPIXEL_BYTES = 500;   // Klipper neopixel.py MAX_MCU_SIZE

NeopixelObj* BoardPlan::neopixel(const std::string& name) {
    for (auto& n : neopixels)
        if (n.name == name) return &n;
    return nullptr;
}

TempSensorObj* BoardPlan::sensor(const std::string& name) {
    for (auto& s : sensors)
        if (s.name == name) return &s;
    return nullptr;
}

static const int ADC_SAMPLE_COUNT = 8;     // klippy/extras/adc_temperature.py
static const double ADC_SAMPLE_TIME = 0.001;
static const double ADC_REPORT_TIME = 0.300;
static const int ADC_RANGE_CHECK_COUNT = 4;

int TempSensorObj::state() const {
    if (adc < 0) return 0;
    if (adc >= 0.99) return 2;
    if (adc <= 0.01) return 3;
    return 1;
}

std::string TempSensorObj::describe() const {
    switch (state()) {
        case 0: return "waiting for the first reading";
        case 2: return "no sensor? (reads open circuit)";
        case 3: return "short circuit? (reads zero)";
    }
    char b[96];
    snprintf(b, sizeof b, "%.1f \xC2\xB0" "C", temp_c);
    std::string s = b;
    if (has_min && temp_c < min_temp) s += " (below min_temp)";
    if (has_max && temp_c > max_temp) s += " (above max_temp)";
    return s;
}

static std::string lower(std::string s) {
    for (auto& c : s) c = char(tolower((unsigned char)c));
    return s;
}

HeaterObj* BoardPlan::heater(const std::string& name) {
    for (auto& h : heaters)
        if (h.name == name) return &h;
    return nullptr;
}

FanObj* BoardPlan::fan(const std::string& name) {
    for (auto& f : fans)
        if (f.name == name) return &f;
    return nullptr;
}

const MacroObj* BoardPlan::macro(const std::string& name) const {
    std::string n = name;
    for (auto& ch : n) ch = char(toupper((unsigned char)ch));
    for (auto& m : macros)
        if (m.name == n) return &m;
    return nullptr;
}

StepperObj* BoardPlan::stepper(const std::string& name) {
    for (auto& s : steppers)
        if (s.name == name) return &s;
    return nullptr;
}

namespace {

// Everything the section builders share.
struct Ctx {
    BoardPlan& plan;
    Config& cfg;
    const MsgProto& mp;
    double freq;
    std::map<std::string, std::string> pin_owner;   // pin -> "[section] key", to catch pins used twice
    std::map<std::string, Thermistor> custom;       // [thermistor NAME]
};

struct PinSpec {
    std::string pin;
    bool invert = false, pullup = false;
};

std::string line_ref(const ConfigSection* s, const std::string& k) {
    return " (line " + std::to_string(s->line_of(k)) + ")";
}

// Klipper pin syntax: optional '^' (pull-up) or '~' (pull-down), optional '!' (invert), optional
// "mcu:" (single board here), then the pin name from the board's own list.
bool parse_pin(Ctx& c, const ConfigSection* s, const std::string& key, PinSpec& out, bool required,
               bool can_pull, bool can_invert, bool shared_ok = false) {
    std::string v = required ? s->get(key) : s->get(key, "");
    if (v.empty()) return false;
    size_t i = 0;
    for (; i < v.size(); i++) {
        if (v[i] == '^' || v[i] == '~') {
            if (!can_pull) {
                s->report("'" + key + ": " + v + "': a pull-up/pull-down ('^' or '~') is not allowed here" + line_ref(s, key));
                return false;
            }
            out.pullup = v[i] == '^';
        } else if (v[i] == '!') {
            if (!can_invert) {
                s->report("'" + key + ": " + v + "': '!' (invert) is not allowed here" + line_ref(s, key));
                return false;
            }
            out.invert = !out.invert;
        } else if (v[i] != ' ') {
            break;
        }
    }
    out.pin = v.substr(i);
    if (out.pin.rfind("mcu:", 0) == 0) out.pin = out.pin.substr(4);
    int64_t pv;
    if (!c.mp.enum_value("pin", out.pin, pv)) {
        s->report(key + " '" + out.pin + "' is not a pin on this board" + line_ref(s, key));
        return false;
    }
    std::string who = "[" + s->name() + "] " + key;
    auto it = c.pin_owner.find(out.pin);
    if (it != c.pin_owner.end() && !shared_ok) {
        s->report("pin " + out.pin + " is used twice: " + it->second + " and " + who + line_ref(s, key));
        return false;
    }
    c.pin_owner.emplace(out.pin, who);
    return true;
}

bool is_rp2040_adc(const std::string& pin) {
    return pin.size() == 6 && pin.rfind("gpio2", 0) == 0 && pin[5] >= '6' && pin[5] <= '9';
}

// A thermistor sensor (sensor_type, sensor_pin, pullup_resistor, inline_resistor). Returns the index
// in plan.sensors, or -1 (with a note for a sensor_type we skip, or an error).
int add_thermistor(Ctx& c, const ConfigSection* s, const std::string& name, bool for_heater) {
    TempSensorObj t;
    t.name = name;
    t.type = s->get("sensor_type");
    Thermistor th;
    bool known = !t.type.empty() && thermistor_preset(t.type, th);
    if (!known && !t.type.empty()) {
        auto it = c.custom.find(lower(t.type));
        if (it != c.custom.end()) th = it->second, known = true;
    }
    if (!known) {
        if (t.type.empty()) return -1;   // get() already reported the missing option
        if (for_heater) {
            s->report("sensor_type '" + t.type + "' is not supported yet (thermistors only: Generic 3950, "
                      "EPCOS 100K B57560G104F, ATC Semitec 104GT-2, NTC 100K MGB18-104F39050L32 or a "
                      "[thermistor NAME] section)" + line_ref(s, "sensor_type"));
            return -1;
        }
        c.plan.skipped_sections.insert(s->name());
        c.plan.notes.push_back("[" + s->name() + "]: sensor_type '" + t.type + "' is not supported yet (thermistors only)");
        return -1;
    }
    std::string raw_pin = s->get("sensor_pin", "");
    std::string rp = lower(raw_pin);
    if (rp.rfind("esp32:", 0) == 0) {   // on the ESP32-S3 itself
#if !PH_PRO
        s->report("sensor_pin '" + raw_pin + "': sensors on the ESP32 itself need the CAPTAIN or SKIPPER build" +
                  line_ref(s, "sensor_pin"));
        return -1;
#endif
        if (for_heater) {
            s->report("a heater's sensor must be on the printer board (esp32: pins are for plain temperature sensors)" +
                      line_ref(s, "sensor_pin"));
            return -1;
        }
        std::string g = rp.substr(6);
        int n = -1;
        if (g.rfind("gpio", 0) == 0 && g.size() > 4) n = atoi(g.c_str() + 4);
        if (n < 1 || n > 10) {
            s->report("sensor_pin '" + raw_pin + "': ESP32-S3 temperature sensors need an ADC1 pin, esp32:gpio1 to esp32:gpio10 "
                      "(ADC2 does not work while WiFi is on)" + line_ref(s, "sensor_pin"));
            return -1;
        }
        t.host_gpio = n;
        t.pin = raw_pin;
        // A thermistor straight to the ESP32's 3.3 V ADC reads best with a pull-up close to its own value;
        // Klipper's 4.7 kohm default would read near full scale (above the ADC's ~3.1 V range) at room temperature.
        th.pullup = s->getf("pullup_resistor", 100000.0, 1.0, 1e9);
        th.inline_resistor = s->getf("inline_resistor", 0.0, 0.0, 1e9);
        t.th = th;
        t.oid = -1;
        c.plan.sensors.push_back(t);
        return int(c.plan.sensors.size()) - 1;
    }
    PinSpec ps;
    if (parse_pin(c, s, "sensor_pin", ps, true, false, false)) {
        t.pin = ps.pin;
        if (c.mp.config_str("MCU") == "rp2040" && !is_rp2040_adc(t.pin))
            s->report("sensor_pin '" + t.pin + "' is not an ADC pin: the rp2040 has ADC on gpio26 to gpio29" +
                      line_ref(s, "sensor_pin"));
    }
    th.pullup = s->getf("pullup_resistor", 4700.0, 1.0, 1e9);
    th.inline_resistor = s->getf("inline_resistor", 0.0, 0.0, 1e9);
    t.th = th;
    t.oid = c.plan.oid_count++;
    c.plan.cmds.push_back("config_analog_in oid=" + std::to_string(t.oid) + " pin=" + t.pin);
    c.plan.sensors.push_back(t);
    return int(c.plan.sensors.size()) - 1;
}

void build_temperature_sensors(Ctx& c) {
    for (auto* s : c.cfg.sections_prefixed("thermistor ")) {
        Thermistor t;
        bool ok;
        std::string beta_s = s->get("beta", "");
        if (!beta_s.empty()) {
            ok = t.setup_beta(s->getf("temperature1"), s->getf("resistance1"), s->getf("beta"));
        } else {
            ok = t.setup_points(s->getf("temperature1"), s->getf("resistance1"), s->getf("temperature2"),
                                s->getf("resistance2"), s->getf("temperature3"), s->getf("resistance3"));
        }
        if (!ok) s->report("these values do not describe a thermistor (resistances must be above zero)");
        c.custom[lower(s->name().substr(11))] = t;
    }
    for (auto* s : c.cfg.sections_prefixed("temperature_sensor ")) {
        int i = add_thermistor(c, s, s->name().substr(19), false);
        if (i < 0) continue;
        auto& t = c.plan.sensors[size_t(i)];
        if (!s->get("min_temp", "").empty()) t.has_min = true, t.min_temp = s->getf("min_temp");
        if (!s->get("max_temp", "").empty()) t.has_max = true, t.max_temp = s->getf("max_temp");
    }
}

// Soft PWM output (Klipper mcu.py MCU_pwm without hardware_pwm): starts and shuts down "off".
void add_pwm(Ctx& c, PwmPin& o, const PinSpec& ps, double cycle_time, double max_duration) {
    o.oid = c.plan.oid_count++;
    o.pin = ps.pin;
    o.invert = ps.invert;
    o.cycle_time = cycle_time;
    o.max_duration = max_duration;
    o.cycle_ticks = uint32_t(cycle_time * c.freq + 0.5);
    uint32_t mdur = uint32_t(max_duration * c.freq + 0.5);
    std::string off = o.invert ? "1" : "0";
    c.plan.cmds.push_back("config_digital_out oid=" + std::to_string(o.oid) + " pin=" + o.pin + " value=" + off +
                          " default_value=" + off + " max_duration=" + std::to_string(mdur));
    c.plan.cmds.push_back("set_digital_out_pwm_cycle oid=" + std::to_string(o.oid) +
                          " cycle_ticks=" + std::to_string(o.cycle_ticks));
}

const int ADC_SAMPLE_COUNT_ = 8;

void build_heaters(Ctx& c) {
    const char* names[] = {"extruder", "heater_bed"};
    for (const char* hn : names) {
        const ConfigSection* s = c.cfg.section(hn);
        if (!s) continue;
        HeaterObj h;
        h.name = hn;
        PinSpec hp;
        bool pin_ok = parse_pin(c, s, "heater_pin", hp, true, false, true);
        h.min_temp = s->getf("min_temp");
        h.max_temp = s->getf("max_temp");
        if (h.max_temp <= h.min_temp) s->report("max_temp must be above min_temp" + line_ref(s, "max_temp"));
        h.max_power = s->getf("max_power", 1.0, 0.0001, 1.0);
        h.smooth_time = s->getf("smooth_time", 1.0, 0.0001, 1e6);
        double cycle = s->getf("pwm_cycle_time", 0.100, 0.0001, 0.300);   // Klipper: maxval = report time
        std::string ctl = lower(s->get("control"));
        if (ctl == "pid") {
            h.pid = true;
            h.kp = s->getf("pid_kp") / 255.0;   // Klipper PID_PARAM_BASE
            h.ki = s->getf("pid_ki") / 255.0;
            h.kd = s->getf("pid_kd") / 255.0;
        } else if (ctl == "watermark") {
            h.pid = false;
            h.max_delta = s->getf("max_delta", 2.0, 0.0001, 1e6);
        } else if (!ctl.empty()) {
            s->report("control must be pid or watermark" + line_ref(s, "control"));
        }
        if (h.name == "heater_bed") h.check_gain_time = 60;
        if (const ConfigSection* v = c.cfg.section(std::string("verify_heater ") + hn)) {
            h.max_error = v->getf("max_error", 120.0, 0.0, 1e9);
            h.check_gain_time = v->getf("check_gain_time", h.check_gain_time, 1.0, 1e9);
            h.hysteresis = v->getf("hysteresis", 5.0, 0.0, 1e9);
            h.heating_gain = v->getf("heating_gain", 2.0, 0.0001, 1e9);
        }
        int si = add_thermistor(c, s, h.name, true);
        if (!pin_ok || si < 0) continue;
        h.sensor = si;
        auto& t = c.plan.sensors[size_t(si)];
        t.heater = int(c.plan.heaters.size());
        t.has_min = true, t.min_temp = h.min_temp;
        t.has_max = true, t.max_temp = h.max_temp;
        // The board checks the raw ADC value itself (Klipper heaters.py setup_minmax + mcu.py).
        double full = ADC_SAMPLE_COUNT_ * c.mp.config_num("ADC_MAX");
        double a = t.th.calc_adc(h.min_temp), b = t.th.calc_adc(h.max_temp);
        double lo = a < b ? a : b, hi = a < b ? b : a;
        long minv = long(lo * full), maxv = long(std::ceil(hi * full));
        t.min_value = uint16_t(minv < 0 ? 0 : minv > 65535 ? 65535 : minv);
        t.max_value = uint16_t(maxv < 0 ? 0 : maxv > 65535 ? 65535 : maxv);
        add_pwm(c, h.out, hp, cycle, 5.0);   // Klipper MAX_HEAT_TIME
        c.plan.heaters.push_back(h);
    }
}

void build_steppers(Ctx& c) {
    std::vector<const ConfigSection*> secs = c.cfg.sections_prefixed("stepper_");
    if (const ConfigSection* e = c.cfg.section("extruder"))
        if (e->has("step_pin")) secs.push_back(e);
    for (auto* s : secs) {
        StepperObj st;
        st.name = s->name();
        PinSpec sp, dp, ep;
        bool ok = parse_pin(c, s, "step_pin", sp, true, false, true);
        ok = parse_pin(c, s, "dir_pin", dp, true, false, true) && ok;
        bool has_en = parse_pin(c, s, "enable_pin", ep, false, false, true);
        long ms = s->geti("microsteps");
        if (ms < 1 || ms > 256 || (ms & (ms - 1)))
            s->report("microsteps must be 1, 2, 4, ... 256" + line_ref(s, "microsteps"));
        st.microsteps = int(ms);
        st.rotation_distance = s->getf("rotation_distance", 0.0, 0.0, 1e9);
        if (st.rotation_distance <= 0) s->report("rotation_distance must be above 0" + line_ref(s, "rotation_distance"));
        {   // gear_ratio: "9.5:1" or "50:17, 80:20" (Klipper stepper.py): the motor turns ratio times per rotation_distance
            std::string gr = s->get("gear_ratio", "");
            double ratio = 1;
            bool bad = false;
            size_t pos = 0;
            while (!gr.empty() && pos <= gr.size()) {
                size_t comma = gr.find(',', pos);
                std::string part = gr.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                size_t colon = part.find(':');
                char *e1 = nullptr, *e2 = nullptr;
                double a = colon == std::string::npos ? 0 : strtod(part.c_str(), &e1);
                double b = colon == std::string::npos ? 0 : strtod(part.c_str() + colon + 1, &e2);
                if (a <= 0 || b <= 0) { bad = true; break; }
                ratio *= a / b;
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
            if (bad) s->report("gear_ratio must look like 3:1 or 50:17, 80:20" + line_ref(s, "gear_ratio"));
            else if (ratio != 1 && st.rotation_distance > 0) st.rotation_distance /= ratio;   // per motor turn
        }
        st.full_steps = int(s->geti("full_steps_per_rotation", 200, 1, 100000));
        if (st.full_steps % 4) s->report("full_steps_per_rotation must be a multiple of 4" + line_ref(s, "full_steps_per_rotation"));
        double pulse = s->getf("step_pulse_duration", 0.000002, 0.0, 0.001);   // Klipper default without the TMC optimisation
        bool delta_rail = false;
        if (const ConfigSection* ps = c.cfg.section("printer"))
            delta_rail = lower(ps->get("kinematics", "none")) == "delta" &&
                         (st.name == "stepper_a" || st.name == "stepper_b" || st.name == "stepper_c");
        if (delta_rail) {   // a delta tower (delta.py): the carriage goes up to its endstop, position_endstop is the nozzle height there
            st.has_rail = true;
            st.endstop_spec = s->get("endstop_pin", "");
            st.endstop_line = s->line_of("endstop_pin");
            st.position_min = 0;
            st.position_endstop = s->getf("position_endstop");
            st.position_max = st.position_endstop;
            st.homing_speed = s->getf("homing_speed", 5.0, 0.001, 1e6);
            st.second_homing_speed = s->getf("second_homing_speed", st.homing_speed / 2, 0.001, 1e6);
            bool sensorless = st.endstop_spec.find("virtual_endstop") != std::string::npos;
            st.homing_retract_dist = s->getf("homing_retract_dist", sensorless ? 0.0 : 5.0, 0.0, 1e6);
            st.homing_positive_dir = true;
            if (st.position_endstop <= 0) s->report("position_endstop must be above 0 on a delta" + line_ref(s, "position_endstop"));
            if (s->has("position_max") || s->has("position_min"))
                s->report("position_min / position_max are not used on a delta tower (position_endstop is the height)");
            const char* nm = st.name.c_str();
            double dflt_angle = nm[8] == 'a' ? 210.0 : nm[8] == 'b' ? 330.0 : 90.0;
            st.angle = s->getf("angle", dflt_angle, -360.0, 360.0);
            st.arm_length = s->getf("arm_length", 0.0, 0.0, 1e5);   // 0: the [printer] arm_length
        } else if (st.name != "extruder") {   // a rail: limits and homing (stepper.py PrinterRail)
            st.has_rail = true;
            st.endstop_spec = s->get("endstop_pin", "");
            st.endstop_line = s->line_of("endstop_pin");
            st.position_min = s->getf("position_min", 0.0);
            st.position_max = s->getf("position_max");
            bool probe_es = lower(st.endstop_spec) == "probe:z_virtual_endstop";   // Z homes on the probe
            st.position_endstop = (st.endstop_spec.empty() || probe_es) ? st.position_min : s->getf("position_endstop");
            st.homing_speed = s->getf("homing_speed", 5.0, 0.001, 1e6);
            st.second_homing_speed = s->getf("second_homing_speed", st.homing_speed / 2, 0.001, 1e6);
            bool sensorless = st.endstop_spec.find("virtual_endstop") != std::string::npos;
            st.homing_retract_dist = s->getf("homing_retract_dist", sensorless ? 0.0 : 5.0, 0.0, 1e6);
            if (s->has("position_max") && st.position_max <= st.position_min)
                s->report("position_max must be above position_min" + line_ref(s, "position_max"));
            if (!st.endstop_spec.empty() && !probe_es &&
                (st.position_endstop < st.position_min || st.position_endstop > st.position_max))
                s->report("position_endstop must be between position_min and position_max" + line_ref(s, "position_endstop"));
            std::string hp = lower(s->get("homing_positive_dir", ""));
            double len = st.position_max - st.position_min;
            if (probe_es) st.homing_positive_dir = false;
            else if (hp == "true" || hp == "1") st.homing_positive_dir = true;
            else if (hp == "false" || hp == "0") st.homing_positive_dir = false;
            else if (st.position_endstop <= st.position_min + len / 4) st.homing_positive_dir = false;
            else if (st.position_endstop >= st.position_max - len / 4) st.homing_positive_dir = true;
            else if (!st.endstop_spec.empty())
                s->report("set homing_positive_dir: position_endstop is in the middle of the axis");
        }
        if (!ok) continue;
        st.step_pin = sp.pin, st.invert_step = sp.invert;
        st.dir_pin = dp.pin, st.invert_dir = dp.invert;
        st.pulse_ticks = uint32_t(pulse * c.freq + 0.5);
        st.oid = c.plan.oid_count++;
        c.plan.cmds.push_back("config_stepper oid=" + std::to_string(st.oid) + " step_pin=" + st.step_pin +
                              " dir_pin=" + st.dir_pin + " invert_step=" + (st.invert_step ? "1" : "0") +
                              " step_pulse_ticks=" + std::to_string(st.pulse_ticks));
        st.trsync_oid = c.plan.oid_count++;
        c.plan.cmds.push_back("config_trsync oid=" + std::to_string(st.trsync_oid));
        if (has_en) {
            st.enable_pin = ep.pin, st.invert_enable = ep.invert;
            st.enable_oid = c.plan.oid_count++;
            std::string off = st.invert_enable ? "1" : "0";
            c.plan.cmds.push_back("config_digital_out oid=" + std::to_string(st.enable_oid) + " pin=" + st.enable_pin +
                                  " value=" + off + " default_value=" + off + " max_duration=0");
        }
        c.plan.steppers.push_back(st);
    }
}

}  // namespace

// ---- TMC2209 (Klipper tmc2209.py / tmc2208.py / tmc2130.py TMCCurrentHelper / tmc.py helpers) ----

static int tmc_cs(double current, bool vsense, double rsense) {
    double r = rsense + 0.020, vref = vsense ? 0.18 : 0.32;
    int cs = int(32.0 * r * current * std::sqrt(2.0) / vref + 0.5) - 1;
    return cs < 0 ? 0 : cs > 31 ? 31 : cs;
}

static double tmc_current(int cs, bool vsense, double rsense) {
    double r = rsense + 0.020, vref = vsense ? 0.18 : 0.32;
    return (cs + 1) * vref / (32.0 * r * std::sqrt(2.0));
}

int tmc2209_cs(double current, bool vsense, double rsense) { return tmc_cs(current, vsense, rsense); }
double tmc2209_current(int cs, bool vsense, double rsense) { return tmc_current(cs, vsense, rsense); }

bool tmc2209_registers(TmcObj& t, int microsteps, double rotation_distance, int full_steps, double rsense,
                       bool has_stealth, double stealth_velocity, bool interpolate, std::string* err) {
    static const std::map<int, int> MRES = {{256, 0}, {128, 1}, {64, 2}, {32, 3}, {16, 4}, {8, 5}, {4, 6}, {2, 7}, {1, 8}};
    auto m = MRES.find(microsteps);
    if (m == MRES.end()) {
        if (err) *err = "microsteps must be 1, 2, 4, ... 256";
        return false;
    }
    int mres = m->second;
    // current (TMCCurrentHelper._calc_current)
    bool vsense = true;
    int irun = tmc_cs(t.run_current, true, rsense);
    if (irun == 31) {
        double cur = tmc_current(irun, true, rsense);
        if (cur < t.run_current) {
            int irun2 = tmc_cs(t.run_current, false, rsense);
            double cur2 = tmc_current(irun2, false, rsense);
            if (std::fabs(t.run_current - cur2) < std::fabs(t.run_current - cur)) vsense = false, irun = irun2;
        }
    }
    int ihold = tmc_cs(t.hold_current < t.run_current ? t.hold_current : t.run_current, vsense, rsense);
    t.irun = irun, t.ihold = ihold, t.vsense = vsense, t.stealthchop = has_stealth, t.rsense = rsense;
    // TPWMTHRS (TMCtstepHelper): 0xfffff = never stealthChop
    uint32_t tpwmthrs = 0xfffff;
    if (has_stealth) {
        if (stealth_velocity <= 0) {
            tpwmthrs = 0xfffff;
        } else {
            double step_dist = rotation_distance / full_steps / microsteps;   // Klipper's step distance
            double step_dist_256 = step_dist / double(1 << mres);
            long thr = long(12000000.0 * step_dist_256 / stealth_velocity + 0.5);   // TMC_FREQUENCY
            tpwmthrs = uint32_t(thr < 0 ? 0 : thr > 0xfffff ? 0xfffff : thr);
        }
    }
    uint32_t gconf = (1u << 6) | (1u << 7) | (1u << 8) | (has_stealth ? 0u : (1u << 2));   // pdn_disable, mstep_reg_select, multistep_filt, en_spreadcycle
    uint32_t chop = 3u | (5u << 4) | (0u << 7) | (2u << 15) | (uint32_t(vsense) << 17) | (uint32_t(mres) << 24) |
                    (uint32_t(interpolate) << 28);   // toff 3, hstrt 5, hend 0, tbl 2
    uint32_t ihr = uint32_t(ihold) | (uint32_t(irun) << 8) | (8u << 16);   // iholddelay 8
    uint32_t pwmconf = 36u | (14u << 8) | (1u << 16) | (1u << 18) | (1u << 19) | (8u << 24) | (12u << 28);
    t.init_regs = {{"GCONF", 0x00, gconf},     {"SLAVECONF", 0x03, 2u << 8}, {"CHOPCONF", 0x6c, chop},
                   {"IHOLD_IRUN", 0x10, ihr},  {"TPWMTHRS", 0x13, tpwmthrs}, {"TCOOLTHRS", 0x14, 0},
                   {"COOLCONF", 0x42, 0},      {"PWMCONF", 0x70, pwmconf},   {"TPOWERDOWN", 0x11, 20},
                   {"SGTHRS", 0x40, 0}};
    return true;
}

namespace {

void build_tmcs(Ctx& c) {
    std::map<std::string, int> uart_oids;          // "rx|tx" -> oid
    std::set<std::pair<int, int>> addrs;           // (uart oid, address)
    for (auto* s : c.cfg.sections_prefixed("tmc2209 ")) {
        TmcObj t;
        t.stepper = s->name().substr(8);
        StepperObj* st = c.plan.stepper(t.stepper);
        if (!st) {
            if (!c.cfg.section(t.stepper))
                s->report("[" + s->name() + "] needs a [" + t.stepper + "] section");
            continue;   // the stepper section has its own errors
        }
        PinSpec rx, tx;
        bool ok = parse_pin(c, s, "uart_pin", rx, true, true, false, true);
        bool has_tx = s->has("tx_pin");
        if (has_tx) ok = parse_pin(c, s, "tx_pin", tx, true, false, false, true) && ok;
        else tx = rx;
        if (s->has("select_pins")) s->report("select_pins (analog UART mux) is not supported yet" + line_ref(s, "select_pins"));
        t.addr = int(s->geti("uart_address", 0, 0, 3));
        t.run_current = s->getf("run_current", 0.0, 0.0, 2.0);
        if (t.run_current <= 0) s->report("run_current is required (0 to 2.0 A)" + line_ref(s, "run_current"));
        t.hold_current = s->getf("hold_current", 2.0, 0.0001, 2.0);   // Klipper MAX_CURRENT
        double rsense = s->getf("sense_resistor", 0.110, 0.0001, 10.0);
        bool has_stealth = s->has("stealthchop_threshold");
        double sv = s->getf("stealthchop_threshold", 0.0, 0.0, 1e12);
        bool interp = s->getb("interpolate", true);
        PinSpec dg;
        if (s->has("diag_pin") && parse_pin(c, s, "diag_pin", dg, true, true, false, true))
            t.diag_pin = dg.pin, t.diag_pullup = dg.pullup;
        t.sgthrs = int(s->geti("driver_sgthrs", 0, 0, 255));
        for (auto* k : {"coolstep_threshold", "high_velocity_threshold", "home_current", "current_change_dwell_time"})
            if (s->has(k)) c.plan.notes.push_back("[" + s->name() + "] " + k + " is not applied yet");
        if (!ok || t.run_current <= 0) continue;
        std::string key = rx.pin + "|" + tx.pin;
        auto it = uart_oids.find(key);
        if (it == uart_oids.end()) {
            int oid = c.plan.oid_count++;
            uint32_t bit_ticks = uint32_t(c.freq / 40000.0 + 0.5);   // Klipper TMC_BAUD_RATE
            c.plan.cmds.push_back("config_tmcuart oid=" + std::to_string(oid) + " rx_pin=" + rx.pin + " pull_up=" +
                                  (rx.pullup ? "1" : "0") + " tx_pin=" + tx.pin + " bit_time=" + std::to_string(bit_ticks));
            it = uart_oids.emplace(key, oid).first;
        }
        t.uart_oid = it->second;
        if (!addrs.insert({t.uart_oid, t.addr}).second) {
            s->report("uart_address " + std::to_string(t.addr) + " is already used on these UART pins" + line_ref(s, "uart_address"));
            continue;
        }
        std::string e;
        if (!tmc2209_registers(t, st->microsteps, st->rotation_distance, st->full_steps, rsense, has_stealth, sv, interp, &e)) {
            s->report(e);
            continue;
        }
        for (auto& reg : t.init_regs) {
            if (reg.name == "SGTHRS") reg.value = uint32_t(t.sgthrs);
            if (reg.name == "GCONF") t.gconf = reg.value;
            if (reg.name == "TPWMTHRS") t.tpwmthrs = reg.value;
            if (reg.name == "TCOOLTHRS") t.tcoolthrs = reg.value;
        }
        st->tmc = int(c.plan.tmcs.size());
        c.plan.tmcs.push_back(t);
    }
}

// endstop_pin / [probe] pin: a board pin, or "tmc2209_<stepper>:virtual_endstop" (sensorless).
int resolve_endstop(Ctx& c, const ConfigSection* s, const std::string& key, const std::string& spec) {
    auto add = [&](const std::string& pin, bool pullup, bool invert, int tmc) {
        for (size_t i = 0; i < c.plan.endstops.size(); i++)
            if (c.plan.endstops[i].pin == pin) return int(i);
        EndstopObj e;
        e.oid = c.plan.oid_count++;
        e.pin = pin, e.pullup = pullup, e.invert = invert, e.tmc = tmc;
        c.plan.cmds.push_back("config_endstop oid=" + std::to_string(e.oid) + " pin=" + pin + " pull_up=" + (pullup ? "1" : "0"));
        c.plan.endstops.push_back(e);
        return int(c.plan.endstops.size()) - 1;
    };
    size_t colon = spec.find(':');
    if (colon != std::string::npos && spec.substr(colon + 1) == "virtual_endstop") {
        std::string chip = spec.substr(0, colon);   // "tmc2209_stepper_x"
        if (chip.rfind("tmc2209_", 0) != 0) {
            s->report(key + ": only tmc2209 virtual endstops are supported" + line_ref(s, key));
            return -1;
        }
        std::string stepper = chip.substr(8);
        for (size_t i = 0; i < c.plan.tmcs.size(); i++) {
            const TmcObj& t = c.plan.tmcs[i];
            if (t.stepper != stepper) continue;
            if (t.diag_pin.empty()) {
                s->report(key + ": sensorless homing needs diag_pin in [tmc2209 " + stepper + "]" + line_ref(s, key));
                return -1;
            }
            return add(t.diag_pin, t.diag_pullup, false, int(i));
        }
        s->report(key + ": no [tmc2209 " + stepper + "] section" + line_ref(s, key));
        return -1;
    }
    PinSpec ps;
    if (!parse_pin(c, s, key, ps, true, true, true, true)) return -1;
    return add(ps.pin, ps.pullup, ps.invert, -1);
}

void build_motion(Ctx& c) {
    if (const ConfigSection* s = c.cfg.section("probe")) {
        ProbeObj& p = c.plan.probe;
        p.present = true;
        p.endstop = resolve_endstop(c, s, "pin", s->get("pin"));
        p.x_offset = s->getf("x_offset", 0.0);
        p.y_offset = s->getf("y_offset", 0.0);
        p.z_offset = s->getf("z_offset");
        p.speed = s->getf("speed", 5.0, 0.001, 1e6);
        p.lift_speed = s->getf("lift_speed", p.speed, 0.001, 1e6);
    }
    for (auto& st : c.plan.steppers) {
        if (!st.has_rail || st.endstop_spec.empty()) continue;
        const ConfigSection* s = c.cfg.section(st.name);
        if (lower(st.endstop_spec) == "probe:z_virtual_endstop") {   // Klipper probe.py: Z homes on the probe
            if (st.name != "stepper_z") {
                s->report("endstop_pin: probe:z_virtual_endstop is only for stepper_z" + line_ref(s, "endstop_pin"));
                continue;
            }
            if (!c.plan.probe.present || c.plan.probe.endstop < 0) {
                s->report("endstop_pin: probe:z_virtual_endstop needs a [probe] section" + line_ref(s, "endstop_pin"));
                continue;
            }
            if (s->has("position_endstop"))
                s->report("position_endstop: not used with probe:z_virtual_endstop (the [probe] z_offset is)" +
                          line_ref(s, "position_endstop"));
            st.endstop = c.plan.probe.endstop;
            st.position_endstop = c.plan.probe.z_offset;   // where the nozzle is when the probe triggers
            continue;
        }
        st.endstop = resolve_endstop(c, s, "endstop_pin", st.endstop_spec);
    }
    if (const ConfigSection* s = c.cfg.section("printer")) {
        PrinterLimits& pr = c.plan.printer;
        pr.present = true;
        std::string k = lower(s->get("kinematics", "none"));
        if (k == "corexy") {
#if PH_KIN_COREXY
            pr.corexy = true;
#else
            s->report("kinematics: corexy is not in this build (the DECKHAND delta build has cartesian and delta)" + line_ref(s, "kinematics"));
#endif
        } else if (k == "delta") {
#if PH_KIN_DELTA
            pr.delta = true;
            pr.delta_radius = s->getf("delta_radius", 0.0, 0.0, 1e5);
            pr.print_radius = s->getf("print_radius", pr.delta_radius, 0.0, 1e5);
            pr.minimum_z_position = s->getf("minimum_z_position", 0.0, -1e5, 1e5);
            pr.arm_length = s->getf("arm_length", 0.0, 0.0, 1e5);
            if (pr.delta_radius <= 0) s->report("delta_radius is required on a delta (above 0)" + line_ref(s, "kinematics"));
            DeltaGeom g;
            double arm[3], ang[3], endst[3];
            bool have = true;
            const char* tn[3] = {"stepper_a", "stepper_b", "stepper_c"};
            for (int i = 0; i < 3; i++) {
                const StepperObj* t = c.plan.stepper(tn[i]);
                if (!t || !t->has_rail) { s->report(std::string("kinematics: delta needs [") + tn[i] + "]" + line_ref(s, "kinematics")); have = false; break; }
                arm[i] = t->arm_length > 0 ? t->arm_length : pr.arm_length;
                ang[i] = t->angle;
                endst[i] = t->position_endstop;
                if (arm[i] <= 0) { s->report(std::string("arm_length is required for ") + tn[i] + " (or once in [printer])"); have = false; }
                if (t->endstop_spec.empty()) { s->report(std::string("endstop_pin is required for ") + tn[i] + " on a delta"); have = false; }
            }
            if (have) {
                g.set(arm, ang, pr.delta_radius);
                if (!g.valid) s->report("delta: arm_length must be longer than delta_radius");
                else {
                    double mz = std::min(endst[0], std::min(endst[1], endst[2]));
                    if (pr.minimum_z_position > mz) s->report("minimum_z_position must not be above the lowest position_endstop");
                }
            }
#else
            s->report("kinematics: delta is not in this build (it is in CAPTAIN and SKIPPER, and in the DECKHAND delta build)" + line_ref(s, "kinematics"));
#endif
        } else if (k != "cartesian" && k != "none")
            s->report("kinematics: cartesian, corexy and delta are supported" + line_ref(s, "kinematics"));
        pr.max_velocity = s->getf("max_velocity", 0.0, 0.0, 1e6);
        pr.max_accel = s->getf("max_accel", 0.0, 0.0, 1e7);
        if (!c.plan.steppers.empty() && (pr.max_velocity <= 0 || pr.max_accel <= 0))
            s->report("max_velocity and max_accel are required (above 0)");
        pr.max_z_velocity = s->getf("max_z_velocity", pr.max_velocity, 0.0, 1e6);
        pr.max_z_accel = s->getf("max_z_accel", pr.max_accel, 0.0, 1e7);
        pr.square_corner_velocity = s->getf("square_corner_velocity", 5.0, 0.0, 1e6);
    }
    if (const ConfigSection* e = c.cfg.section("extruder")) {
        c.plan.printer.min_extrude_temp = e->getf("min_extrude_temp", 170.0);
        c.plan.printer.max_extrude_only_velocity = e->getf("max_extrude_only_velocity", 0.0, 0.0, 1e6);
        c.plan.shaping.pa = e->getf("pressure_advance", 0.0, 0.0, 10.0);
        c.plan.shaping.pa_smooth = e->getf("pressure_advance_smooth_time", 0.040, 0.0, 0.200);
        if (c.plan.shaping.pa > 0 && c.plan.shaping.pa_smooth <= 0) c.plan.shaping.pa_smooth = 0.040;
        c.plan.shaping.pa_on = c.plan.shaping.pa > 0;
#if !PH_PRO
        if (c.plan.shaping.pa > 0) c.plan.notes.push_back("pressure_advance needs the CAPTAIN or SKIPPER build (ignored)");
#endif
    }
#if PH_PRO
    // "x, y" (or "x,y,z") option values
    auto nums = [](const ConfigSection* s, const std::string& k, double* out, int n) {
        std::string v = s->get(k, "");
        for (auto& ch : v)
            if (ch == ',') ch = ' ';
        std::istringstream in(v);
        int got = 0;
        double d;
        while (got < n && in >> d) out[got++] = d;
        std::string rest;
        if (got != n || (in >> rest)) {
            s->report(k + ": needs " + std::to_string(n) + " numbers separated by commas" + line_ref(s, k));
            return false;
        }
        return true;
    };
    if (const ConfigSection* s = c.cfg.section("bed_mesh")) {
        MeshConfig& m = c.plan.mesh;
        double a[2], b[2], pc[2] = {3, 3};
        bool ok = nums(s, "mesh_min", a, 2) & nums(s, "mesh_max", b, 2);
        std::string cnt = s->get("probe_count", "3");
        if (cnt.find(',') == std::string::npos) pc[0] = pc[1] = atof(cnt.c_str());
        else ok &= nums(s, "probe_count", pc, 2);
        if (ok && (pc[0] < 3 || pc[1] < 3 || pc[0] > 10 || pc[1] > 10)) ok = false, s->report("probe_count: 3 to 10 each way" + line_ref(s, "probe_count"));
        if (ok && (b[0] <= a[0] || b[1] <= a[1])) ok = false, s->report("mesh_max must be above mesh_min" + line_ref(s, "mesh_max"));
        m.present = ok;
        m.min_x = a[0], m.min_y = a[1], m.max_x = b[0], m.max_y = b[1];
        m.count_x = int(pc[0]), m.count_y = int(pc[1]);
        m.horizontal_move_z = s->getf("horizontal_move_z", 5.0, -10, 1000);
        m.speed = s->getf("speed", 50.0, 1, 1000);
        for (auto* k : {"fade_start", "fade_end", "fade_target", "split_delta_z", "move_check_distance", "mesh_pps", "algorithm",
                        "bicubic_tension", "zero_reference_position", "faulty_region_1_min", "faulty_region_1_max", "scan_overshoot"})
            if (s->has(k)) s->get(k, "");   // accepted, not used (no fade, bilinear on the probed points)
    }
    if (const ConfigSection* s = c.cfg.section("bed_mesh default")) {
        std::string err;
        double mnx = s->getf("min_x", 0.0), mxx = s->getf("max_x", 0.0), mny = s->getf("min_y", 0.0), mxy = s->getf("max_y", 0.0);
        int xc = int(s->getf("x_count", 0.0)), yc = int(s->getf("y_count", 0.0));
        for (auto* k : {"version", "mesh_x_pps", "mesh_y_pps", "algo", "tension"})
            if (s->has(k)) s->get(k, "");
        if (!c.plan.mesh_default.from_profile(s->get("points", ""), xc, yc, mnx, mxx, mny, mxy, &err))
            s->report("[bed_mesh default]: " + err);
    }
    if (const ConfigSection* s = c.cfg.section("screws_tilt_adjust")) {
        ScrewsConfig& sc = c.plan.screws;
        for (int i = 1; i < 100; i++) {
            std::string k = "screw" + std::to_string(i);
            if (!s->has(k)) break;
            double xy[2];
            if (!nums(s, k, xy, 2)) continue;
            char nm[64];
            snprintf(nm, sizeof nm, "screw at %.3f,%.3f", xy[0], xy[1]);
            sc.screws.push_back({s->get(k + "_name", nm), xy[0], xy[1]});
        }
        sc.thread = screw_thread_index(s->get("screw_thread", "CW-M3"));
        if (sc.thread < 0) s->report("screw_thread: CW-M3, CCW-M3, CW-M4, CCW-M4, CW-M5, CCW-M5, CW-M6 or CCW-M6" + line_ref(s, "screw_thread")), sc.thread = 0;
        sc.horizontal_move_z = s->getf("horizontal_move_z", 10.0, -10, 1000);
        sc.speed = s->getf("speed", 50.0, 1, 1000);
        sc.present = sc.screws.size() >= 3;
        if (!sc.present) s->report("screws_tilt_adjust: needs at least three screws (screw1, screw2, screw3)");
    }
    if (const ConfigSection* s = c.cfg.section("firmware_retraction")) {
        RetractConfig& r = c.plan.retract;
        r.length = s->getf("retract_length", 0.0, 0.0, 100.0);
        r.speed = s->getf("retract_speed", 20.0, 0.1, 1000.0);
        r.unretract_extra = s->getf("unretract_extra_length", 0.0, 0.0, 100.0);
        r.unretract_speed = s->getf("unretract_speed", 10.0, 0.1, 1000.0);
    }
    if (const ConfigSection* s = c.cfg.section("gcode_arcs")) c.plan.arc_resolution = s->getf("resolution", 1.0, 0.01, 100.0);
    if (const ConfigSection* s = c.cfg.section("resonance_tester")) {
        ResonanceParams& r = c.plan.resonance;
        r.min_freq = s->getf("min_freq", 5.0, 1.0, 300.0);
        r.max_freq = s->getf("max_freq", 135.0, r.min_freq, 300.0);
        r.accel_per_hz = s->getf("accel_per_hz", 60.0, 1.0, 1000.0);
        r.hz_per_sec = s->getf("hz_per_sec", 1.0, 0.1, 2.0);
        r.sweeping_accel = s->getf("sweeping_accel", 400.0, 1.0, 10000.0);
        r.sweeping_period = s->getf("sweeping_period", 1.2, 0.0, 10.0);
        if (s->has("probe_points")) {
            std::string v = s->get("probe_points", "");
            v = v.substr(0, v.find_first_of(";\n", v.find_first_not_of("\n ")));   // the first point
            double p[3];
            for (auto& ch : v)
                if (ch == ',') ch = ' ';
            std::istringstream in(v);
            if (in >> p[0] >> p[1] >> p[2]) {
                c.plan.resonance_point_set = true;
                for (int i = 0; i < 3; i++) c.plan.resonance_point[i] = p[i];
            } else {
                s->report("probe_points: x, y, z" + line_ref(s, "probe_points"));
            }
        }
        for (auto* k : {"accel_chip", "accel_chip_x", "accel_chip_y", "max_smoothing", "move_speed"})
            if (s->has(k)) s->get(k, "");
    }
    if (const ConfigSection* s = c.cfg.section("input_shaper")) {   // Klipper input_shaper.py
        ShapingConfig& sc = c.plan.shaping;
        std::string def = lower(s->get("shaper_type", "mzv"));
        for (int ax = 0; ax < 2; ax++) {
            const char* x = ax ? "y" : "x";
            double f = s->getf(std::string("shaper_freq_") + x, 0.0, 0.0, 1000.0);
            std::string t = lower(s->get(std::string("shaper_type_") + x, def));
            double d = s->getf(std::string("damping_ratio_") + x, DEFAULT_DAMPING_RATIO, 0.0, 0.999);
            std::vector<double> A, T;
            if (f > 0 && !shaper_def(t, f, d, A, T)) {
                s->report("unknown shaper_type '" + t + "' (zv, mzv, zvd, ei, 2hump_ei, 3hump_ei)" +
                          line_ref(s, std::string("shaper_type_") + x));
                continue;
            }
            (ax ? sc.y : sc.x) = f > 0 ? shaper_pulses(A, T) : ShaperPulses();
            (ax ? sc.y_type : sc.x_type) = t;
            (ax ? sc.y_freq : sc.x_freq) = f;
            (ax ? sc.y_damping : sc.x_damping) = d;
        }
    }
#else
    for (auto* sec : {"input_shaper", "bed_mesh", "bed_mesh default", "screws_tilt_adjust", "firmware_retraction", "gcode_arcs",
                      "resonance_tester"})
        if (c.cfg.section(sec)) {
            c.plan.skipped_sections.insert(sec);
            c.plan.notes.push_back(std::string("[") + sec + "] needs the CAPTAIN or SKIPPER build");
        }
#endif
    if (!c.plan.endstops.empty() && (!c.mp.has_command("config_endstop") || !c.mp.has_command("endstop_home")))
        c.cfg.errors().push_back("this board's firmware has no endstop support");
}

void build_fans(Ctx& c) {
    std::vector<const ConfigSection*> secs;
    if (const ConfigSection* f = c.cfg.section("fan")) secs.push_back(f);
    for (auto* s : c.cfg.sections_prefixed("heater_fan ")) secs.push_back(s);
    for (auto* s : secs) {
        FanObj f;
        f.heater_fan = s->name() != "fan";
        f.name = f.heater_fan ? s->name().substr(11) : "fan";
        PinSpec ps;
        bool ok = parse_pin(c, s, "pin", ps, true, false, true);
        f.max_power = s->getf("max_power", 1.0, 0.0001, 1.0);
        f.kick_start_time = s->getf("kick_start_time", 0.1, 0.0, 10.0);
        f.off_below = s->getf("off_below", 0.0, 0.0, 1.0);
        double cycle = s->getf("cycle_time", 0.010, 0.0001, 1.0);
        if (s->getb("hardware_pwm", false)) c.plan.notes.push_back("[" + s->name() + "] hardware_pwm is not used yet (soft PWM instead)");
        if (s->has("enable_pin")) s->report("enable_pin on a fan is not supported yet" + line_ref(s, "enable_pin"));
        if (f.heater_fan) {
            f.heater = s->get("heater", "extruder");
            f.heater_temp = s->getf("heater_temp", 50.0, -273.0, 1000.0);
            f.fan_speed = s->getf("fan_speed", 1.0, 0.0, 1.0);
            if (!c.plan.heater(f.heater))
                s->report("heater '" + f.heater + "' is not a heater in printer.cfg" + line_ref(s, "heater"));
        }
        if (!ok) continue;
        add_pwm(c, f.out, ps, cycle, 0.0);
        c.plan.fans.push_back(f);
    }
}

// Options Klipper knows for a section kind that this project accepts without using them yet.
bool known_unused(const std::string& sec, const std::string& key) {
    static const std::set<std::string> stepper = {"homing_retract_speed"};
    static const std::set<std::string> extruder = {
        "nozzle_diameter", "filament_diameter", "max_extrude_cross_section", "instantaneous_corner_velocity",
        "max_extrude_only_distance", "max_extrude_only_velocity", "max_extrude_only_accel", "min_extrude_temp"};
    static const std::set<std::string> tmc = {};
    static const std::set<std::string> printer = {"minimum_cruise_ratio", "max_accel_to_decel"};
    static const std::set<std::string> probe = {"samples", "sample_retract_dist", "samples_result", "samples_tolerance",
                                                "samples_tolerance_retries", "activate_gcode", "deactivate_gcode"};
    if (sec == "printer") return printer.count(key) > 0;
    if (sec == "probe") return probe.count(key) > 0;
    static const std::set<std::string> fan = {"shutdown_speed", "tachometer_pin", "tachometer_ppr",
                                              "tachometer_poll_interval", "min_power"};
    if (sec.rfind("stepper_", 0) == 0) return stepper.count(key) > 0;
    if (sec == "extruder") return stepper.count(key) > 0 || extruder.count(key) > 0;
    if (sec.rfind("tmc2209 ", 0) == 0) return tmc.count(key) > 0 || key.rfind("driver_", 0) == 0;
    if (sec == "fan" || sec.rfind("heater_fan ", 0) == 0) return fan.count(key) > 0;
    return false;
}

bool handled_section(const std::string& n) {
    for (auto* p : {"neopixel ", "temperature_sensor ", "thermistor ", "stepper_", "tmc2209 ", "heater_fan ", "verify_heater ",
                    "gcode_macro "})
        if (n.rfind(p, 0) == 0) return true;
    return n == "extruder" || n == "heater_bed" || n == "fan" || n == "printer" || n == "probe" || n == "homing_override" ||
           n == "input_shaper" || n == "bed_mesh" || n == "bed_mesh default" || n == "screws_tilt_adjust" ||
           n == "firmware_retraction" || n == "gcode_arcs" || n == "resonance_tester" || n == "exclude_object";
}

// G-code lines of a macro: comments and blank lines dropped. Klipper's Jinja templates ({...}) are
// not supported: refused with a clear message instead of running something half-understood.
std::vector<std::string> macro_lines(const ConfigSection* s, const std::string& what) {
    std::vector<std::string> out;
    std::string text = s->get("gcode", "");
    size_t start = 0;
    while (start <= text.size()) {
        size_t e = text.find('\n', start);
        std::string l = text.substr(start, e == std::string::npos ? std::string::npos : e - start);
        start = e == std::string::npos ? text.size() + 1 : e + 1;
        size_t c = l.find_first_of(";#");
        if (c != std::string::npos) l = l.substr(0, c);
        while (!l.empty() && isspace((unsigned char)l.back())) l.pop_back();
        size_t b = l.find_first_not_of(" \t");
        if (b == std::string::npos) continue;
        l = l.substr(b);
        if (l.find('{') != std::string::npos || l.find("%}") != std::string::npos) {
            s->report(what + ": templates ({...}) are not supported yet, only plain G-code lines (line " +
                      std::to_string(s->line_of("gcode")) + ")");
            return {};
        }
        out.push_back(l);
    }
    if (out.empty()) s->report(what + " has no gcode lines");
    return out;
}

void build_macros(Ctx& c) {
    for (auto* s : c.cfg.sections_prefixed("gcode_macro ")) {
        MacroObj m;
        m.name = s->name().substr(12);
        for (auto& ch : m.name) ch = char(toupper((unsigned char)ch));
        if (m.name.empty() || m.name.find_first_of(" =") != std::string::npos) {
            s->report("macro name '" + m.name + "' must be one word");
            continue;
        }
        s->get("description", "");
        m.lines = macro_lines(s, "[" + s->name() + "]");
        if (!m.lines.empty()) c.plan.macros.push_back(m);
    }
    if (const ConfigSection* s = c.cfg.section("homing_override")) {
        c.plan.homing_override = macro_lines(s, "[homing_override]");
        std::string ax = s->get("axes", "xyz");
        for (auto& ch : ax) ch = char(tolower((unsigned char)ch));
        if (ax.find_first_not_of("xyz") != std::string::npos || ax.empty())
            s->report("axes must be letters from xyz (line " + std::to_string(s->line_of("axes")) + ")");
        c.plan.homing_override_axes = ax;
        for (int a = 0; a < 3; a++) {
            std::string k = std::string("set_position_") + "xyz"[a];
            if (s->has(k)) c.plan.homing_override_set[a] = s->getf(k, 0.0, -1e6, 1e6);
        }
    }
}

}  // namespace

BoardPlan build_board_plan(const std::string& cfg_text, const MsgProto& mp, double clock_freq) {
    BoardPlan plan;
    plan.clock_freq = clock_freq;
    Config cfg;
    cfg.parse(cfg_text);
    Ctx c{plan, cfg, mp, clock_freq, {}, {}};
    for (auto* s : cfg.sections_prefixed("neopixel ")) {
        NeopixelObj n;
        n.name = s->name().substr(9);
        PinSpec ps;
        if (parse_pin(c, s, "pin", ps, true, false, false)) n.pin = ps.pin;
        n.chain_count = int(s->geti("chain_count", 1, 1, MAX_NEOPIXEL_BYTES));
        n.order = s->get("color_order", "GRB");
        for (auto& ch : n.order) ch = char(toupper((unsigned char)ch));
        std::set<char> seen;
        bool order_ok = !n.order.empty() && n.order.size() <= 4;
        for (char ch : n.order) order_ok = order_ok && std::string("RGBW").find(ch) != std::string::npos && seen.insert(ch).second;
        if (!order_ok)
            s->report("color_order '" + n.order + "' must use R, G, B and W once each (line " +
                      std::to_string(s->line_of("color_order")) + ")");
        if (n.chain_count * int(n.order.size()) > MAX_NEOPIXEL_BYTES)
            s->report("chain too long (" + std::to_string(n.chain_count * n.order.size()) + " bytes, max " +
                      std::to_string(MAX_NEOPIXEL_BYTES) + ")");
        // option names are case-insensitive (the parser lower-cases them), as in Klipper
        double r = s->getf("initial_red", 0, 0, 1), g = s->getf("initial_green", 0, 0, 1);
        double b = s->getf("initial_blue", 0, 0, 1), w = s->getf("initial_white", 0, 0, 1);
        n.oid = plan.oid_count++;
        for (int i = 0; i < n.chain_count; i++)
            for (char ch : n.order)
                n.data.push_back(uint8_t((ch == 'R' ? r : ch == 'G' ? g : ch == 'B' ? b : w) * 255.0 + 0.5));
        plan.cmds.push_back(neopixel_config_cmd(n.oid, n.pin, int(n.data.size()), clock_freq));
        plan.neopixels.push_back(n);
    }
    build_temperature_sensors(c);
    build_heaters(c);
    build_steppers(c);
    build_tmcs(c);
    build_motion(c);
    build_fans(c);
    build_macros(c);
    if (!plan.sensors.empty()) {
        plan.adc_full_scale = ADC_SAMPLE_COUNT * mp.config_num("ADC_MAX");
        if (plan.adc_full_scale <= 0 || !mp.has_command("config_analog_in") || !mp.has_command("query_analog_in"))
            cfg.errors().push_back("this board's firmware has no ADC (temperature) support");
    }
    if (!plan.neopixels.empty() && !mp.has_command("config_neopixel"))
        cfg.errors().push_back("this board's firmware has no neopixel support");
    if ((!plan.heaters.empty() || !plan.fans.empty() || !plan.steppers.empty()) &&
        (!mp.has_command("config_digital_out") || !mp.has_command("queue_digital_out") ||
         !mp.has_command("set_digital_out_pwm_cycle")))
        cfg.errors().push_back("this board's firmware has no digital output (heater, fan, enable pin) support");
    if (!plan.steppers.empty() && (!mp.has_command("config_stepper") || !mp.has_command("queue_step") ||
                                   !mp.has_command("config_trsync") || !mp.has_command("stepper_stop_on_trigger")))
        cfg.errors().push_back("this board's firmware has no stepper support");
    if (!plan.tmcs.empty() && (!mp.has_command("config_tmcuart") || !mp.has_command("tmcuart_send")))
        cfg.errors().push_back("this board's firmware has no TMC UART support");
    // Options we do not know are probably typos. Only check the sections we support.
    for (auto& k : cfg.unused_keys()) {
        std::string sec = k.substr(1, k.find(']') - 1);
        if (!handled_section(sec) || plan.skipped_sections.count(sec)) continue;
        std::string key = k.substr(k.find(']') + 2);
        key = key.substr(0, key.find(' '));
        if (known_unused(sec, key)) continue;
        cfg.errors().push_back("unknown option " + k);
    }
    plan.errors = cfg.errors();
    // Everything else in a Klipper printer.cfg is accepted but not used yet.
    std::set<std::string> listed;
    for (auto* s : cfg.sections_prefixed("")) {
        std::string n = s->name();
        if (handled_section(n) || n == "mcu") continue;
        std::string kind = n.substr(0, n.find(' '));
        if (listed.insert(kind).second) plan.notes.push_back("[" + kind + "] not used yet");
    }
    return plan;
}

std::vector<std::string> adc_query_cmds(const BoardPlan& plan, uint32_t start_clock, double clock_freq) {
    std::vector<std::string> v;
    uint32_t sample_ticks = uint32_t(ADC_SAMPLE_TIME * clock_freq + 0.5);
    uint32_t rest_ticks = uint32_t(ADC_REPORT_TIME * clock_freq + 0.5);
    for (size_t i = 0; i < plan.sensors.size(); i++) {
        if (plan.sensors[i].host_gpio >= 0) continue;   // read by the ESP32 (Machine::tick)
        uint32_t clock = start_clock + uint32_t(double(i) * 0.01 * clock_freq);   // stagger like Klipper
        v.push_back("query_analog_in oid=" + std::to_string(plan.sensors[i].oid) + " clock=" + std::to_string(clock) +
                    " sample_ticks=" + std::to_string(sample_ticks) + " sample_count=" + std::to_string(ADC_SAMPLE_COUNT) +
                    " rest_ticks=" + std::to_string(rest_ticks) + " min_value=" + std::to_string(plan.sensors[i].min_value) +
                    " max_value=" + std::to_string(plan.sensors[i].max_value) + " range_check_count=" +
                    std::to_string(ADC_RANGE_CHECK_COUNT));
    }
    return v;
}

bool apply_adc_message(BoardPlan& plan, const Message& m) {
    if (m.name != "analog_in_state" || plan.adc_full_scale <= 0) return false;
    int64_t oid = m.get("oid");
    for (auto& s : plan.sensors) {
        if (s.oid != oid) continue;
        s.adc = double(m.get("value")) / plan.adc_full_scale;
        s.temp_c = s.th.calc_temp(s.adc);
        s.updates++;
        return true;
    }
    return false;
}

static bool parse_unit(const std::map<std::string, std::string>& p, const char* k, double& v, std::string& msg) {
    auto it = p.find(k);
    if (it == p.end()) return true;
    char* end = nullptr;
    v = strtod(it->second.c_str(), &end);
    if (end == it->second.c_str() || *end || v < 0 || v > 1) {
        msg = std::string(k) + " must be a number from 0.0 to 1.0";
        return false;
    }
    return true;
}

NeopixelObj* apply_set_led(BoardPlan& plan, const std::map<std::string, std::string>& params, std::string& msg) {
    auto it = params.find("LED");
    if (it == params.end()) {
        msg = "SET_LED needs LED=<name>";
        return nullptr;
    }
    NeopixelObj* n = plan.neopixel(it->second);
    if (!n) n = plan.neopixel(lower(it->second));
    if (!n) {
        msg = "no LED named '" + it->second + "' in printer.cfg";
        return nullptr;
    }
    double r = 0, g = 0, b = 0, w = 0;
    if (!parse_unit(params, "RED", r, msg) || !parse_unit(params, "GREEN", g, msg) ||
        !parse_unit(params, "BLUE", b, msg) || !parse_unit(params, "WHITE", w, msg))
        return nullptr;
    int first = 1, last = n->chain_count;
    auto ix = params.find("INDEX");
    if (ix != params.end()) {
        char* end = nullptr;
        long i = strtol(ix->second.c_str(), &end, 10);
        if (end == ix->second.c_str() || *end || i < 1 || i > n->chain_count) {
            msg = "INDEX must be 1 to " + std::to_string(n->chain_count);
            return nullptr;
        }
        first = last = int(i);
    }
    size_t per = n->order.size();
    for (int led = first; led <= last; led++)
        for (size_t k = 0; k < per; k++) {
            char c = n->order[k];
            double v = c == 'R' ? r : c == 'G' ? g : c == 'B' ? b : w;
            n->data[size_t(led - 1) * per + k] = uint8_t(v * 255.0 + 0.5);
        }
    msg = "ok";
    return n;
}

std::string run_console_command(BoardPlan& plan, KBoard& board, const std::string& line) {
    std::string name;
    std::map<std::string, std::string> params;
    if (!gcode_parse_ext(line, name, params)) {
        // Find the first word that is not NAME=value so the message can point at it.
        std::string l = line.substr(0, line.find_first_of(";#")), w, bad;
        bool first = true;
        for (size_t i = 0; i <= l.size() && bad.empty(); i++) {
            if (i < l.size() && !isspace((unsigned char)l[i])) { w += l[i]; continue; }
            if (!w.empty()) {
                auto eq = w.find('=');
                if (!first && (eq == std::string::npos || eq == 0)) bad = w;
                first = false;
                w.clear();
            }
        }
        if (bad.empty()) return "error: empty command. Try: HELP";
        return "error: '" + bad + "' is not NAME=value (no spaces around '='). Example: SET_LED LED=board_led GREEN=1";
    }
    if (name == "HELP")
        return "Supported: SET_LED LED=<name> RED= GREEN= BLUE= [WHITE=] [INDEX=] (0.0-1.0), M105 (temperatures), HELP";
    if (name == "M105") {
        if (plan.sensors.empty()) return "error: no [temperature_sensor] in printer.cfg";
        std::string r = "ok";
        for (size_t i = 0; i < plan.sensors.size(); i++)
            r += (i ? ", " : " ") + plan.sensors[i].name + ": " + plan.sensors[i].describe();
        return r;
    }
    if (name == "SET_LED") {
        std::string msg;
        NeopixelObj* n = apply_set_led(plan, params, msg);
        if (!n) return "error: " + msg;
        std::string err;
        if (!board.neopixel_write(n->oid, n->data, &err)) return "error: " + err;
        return "ok";
    }
    return "error: " + name + " is not supported yet (try HELP)";
}

}  // namespace ph
