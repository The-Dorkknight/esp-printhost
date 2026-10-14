// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Heater control and checks follow Klipper v0.13.0 klippy/extras/heaters.py and verify_heater.py;
// fans follow fan.py / heater_fan.py; TMC2209 UART follows tmc_uart.py / tmc2209.py; the motor test
// uses the board's own stepper commands (queue_step, set_next_step_dir, reset_step_clock) and
// Klipper's trsync stop (as homing does) for the abort.
#include "ph/machine.h"

#include "ph/config_ini.h"
#include "ph/gcode.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace ph {

static const double MAX_HEAT_TIME = 5.0;      // heaters.py
static const double PWM_DELAY = 0.3;          // heater pwm_delay = ADC report time
static const double FAN_DELAY = 0.1;
static const double BUZZ_DISTANCE = 1.0, BUZZ_VELOCITY = 1.0 / 0.250;   // force_move.py STEPPER_BUZZ

// ---------------------------------------------------------------- TMC UART datagrams (tmc_uart.py)

uint8_t tmc_crc8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (uint8_t b : data)
        for (int i = 0; i < 8; i++) {
            if (((crc >> 7) ^ (b & 0x01)) != 0)
                crc = uint8_t((crc << 1) ^ 0x07);
            else
                crc = uint8_t(crc << 1);
            b >>= 1;
        }
    return crc;
}

std::vector<uint8_t> tmc_add_serial_bits(const std::vector<uint8_t>& data) {
    // Each byte becomes 10 bits (start 0, 8 data bits LSB first, stop 1), packed LSB first.
    std::vector<uint8_t> out((data.size() * 10 + 7) / 8, 0);
    size_t pos = 0;
    for (uint8_t d : data) {
        uint32_t b = (uint32_t(d) << 1) | 0x200;
        for (int i = 0; i < 10; i++, pos++)
            if (b & (1u << i)) out[pos / 8] |= uint8_t(1u << (pos % 8));
    }
    return out;
}

std::vector<uint8_t> tmc_encode_read(uint8_t sync, uint8_t addr, uint8_t reg) {
    std::vector<uint8_t> m = {sync, addr, reg};
    m.push_back(tmc_crc8(m));
    return tmc_add_serial_bits(m);
}

std::vector<uint8_t> tmc_encode_write(uint8_t sync, uint8_t addr, uint8_t reg, uint32_t val) {
    std::vector<uint8_t> m = {sync, addr, reg, uint8_t(val >> 24), uint8_t(val >> 16), uint8_t(val >> 8), uint8_t(val)};
    m.push_back(tmc_crc8(m));
    return tmc_add_serial_bits(m);
}

bool tmc_decode_read(uint8_t reg, const std::vector<uint8_t>& data, uint32_t& val) {
    if (data.size() != 10) return false;
    auto byte_at = [&](int bit) {
        uint32_t v = 0;
        for (int i = 0; i < 8; i++) {
            int p = bit + i;
            if (data[size_t(p / 8)] & (1u << (p % 8))) v |= 1u << i;
        }
        return v;
    };
    val = (byte_at(31) << 24) | (byte_at(41) << 16) | (byte_at(51) << 8) | byte_at(61);
    return tmc_encode_write(0x05, 0xff, reg, val) == data;   // start/stop bits and CRC
}

// ---------------------------------------------------------------- small helpers

static std::string fmt(const char* f, double a) {
    char b[64];
    snprintf(b, sizeof b, f, a);
    return b;
}

static std::string json_str(const std::string& s) {
    std::string o = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') o += '\\', o += char(c);
        else if (c == '\n') o += "\\n";
        else if (c < 0x20) o += ' ';
        else o += char(c);
    }
    return o + "\"";
}

static bool num(const std::map<std::string, std::string>& p, const char* k, double& v, double lo, double hi,
                std::string& err) {
    auto it = p.find(k);
    if (it == p.end()) return true;
    char* end = nullptr;
    double d = strtod(it->second.c_str(), &end);
    if (end == it->second.c_str() || *end || d < lo || d > hi) {
        err = std::string(k) + " must be a number from " + fmt("%g", lo) + " to " + fmt("%g", hi);
        return false;
    }
    v = d;
    return true;
}

// ---------------------------------------------------------------- Machine

Machine::Machine(BoardPlan& plan, const MsgProto& mp, MachineIO io) : plan_(plan), mp_(mp), io_(std::move(io)) {
    hr_.resize(plan_.heaters.size());
    fr_.resize(plan_.fans.size());
    sr_.resize(plan_.steppers.size());
    sensor_rate_.assign(plan_.sensors.size(), 0);
    sensor_prev_.assign(plan_.sensors.size(), 0);
    sensor_prev_time_.assign(plan_.sensors.size(), -1);
    last_clock_.assign(size_t(plan_.oid_count > 0 ? plan_.oid_count : 1), 0);
    heat_tests_.resize(plan_.heaters.size());
    motor_tests_.resize(plan_.steppers.size());
    fan_tests_.resize(plan_.fans.size());
    for (auto& h : plan_.heaters)
        tests_.push_back({"heat_" + h.name, h.name == "extruder" ? "Hotend heater" : "Bed heater"});
    for (auto& h : plan_.heaters)
        tests_.push_back({"out_" + h.name, h.name == "extruder" ? "Hotend output check (MOSFET)" : "Bed output check (MOSFET)"});
    for (auto& h : plan_.heaters)
        tests_.push_back({"pid_" + h.name, h.name == "extruder" ? "PID tune hotend" : "PID tune bed"});
    for (auto& s : plan_.steppers) tests_.push_back({"motor_" + s.name, "Motor " + s.name});
    for (auto& s : plan_.steppers)   // sensorless axes: stallGuard readings and the DIAG wire
        if (s.endstop >= 0 && plan_.endstops[size_t(s.endstop)].tmc >= 0)
            tests_.push_back({"sg_" + s.name, "StallGuard " + s.name});
    for (auto& f : plan_.fans) tests_.push_back({"fan_" + f.name, "Fan " + f.name});
    if (!plan_.neopixels.empty()) tests_.push_back({"led", "LED colours"});
    if (!plan_.tmcs.empty()) tests_.push_back({"tmc", "TMC2209 drivers"});
    tests_.push_back({"all", "Run all tests"});
    tests_.reserve(tests_.size() + 8);
    if (plan_.stepper("stepper_x") || plan_.printer.present) {
        ToolheadIO tio;
        tio.send = io_.send;
        tio.query = io_.query;
        tio.clock_at = io_.clock_at;
        tio.log = io_.log;
        tio.now = io_.now;
        tio.largest_free = io_.largest_free;
        tio.enable = [this](size_t i, bool on, double now) { stepper_enable(i, on, now); };
        tio.tmc_write = [this](int t, int reg, uint32_t val) {
            std::string err;
            return tmc_write(plan_.tmcs[size_t(t)], uint8_t(reg), val, &err);
        };
        tio.extruder_temp = [this]() {
            for (size_t i = 0; i < plan_.heaters.size(); i++)
                if (plan_.heaters[i].name == "extruder") return hr_[i].have_temp ? hr_[i].smoothed : 0.0;
            return 1000.0;   // no hotend heater configured: nothing to protect
        };
        th_.reset(new Toolhead(plan_, tio));
    }
}

uint32_t Machine::clock32(double t) const { return uint32_t(io_.clock_at(t)); }

bool Machine::send(const std::vector<std::string>& cmds, std::string* err) {
    std::string e;
    bool ok = io_.send(cmds, &e);
    if (!ok) {
        if (io_.log) io_.log("send failed: " + e);
        if (err) *err = e;
    }
    return ok;
}

void Machine::set_pwm(const PwmPin& p, double value, double when) {
    if (p.oid < 0) return;
    if (p.invert) value = 1.0 - value;
    value = value < 0 ? 0 : value > 1 ? 1 : value;
    uint32_t v = uint32_t(value * p.cycle_ticks + 0.5);
    if (io_.now) when = std::max(when, io_.now() + 0.05);   // never in the past after a blocking step
    uint32_t clock = clock32(when);
    uint32_t& last = last_clock_[size_t(p.oid)];
    if (last && int32_t(clock - last) <= 0) clock = last + 1;   // the board's queue must stay in time order
    last = clock;
    send({"queue_digital_out oid=" + std::to_string(p.oid) + " clock=" + std::to_string(clock) +
          " on_ticks=" + std::to_string(v)});
}

std::string Machine::start(double now) {
    // First PWM values (Klipper sends these as init commands): everything off.
    for (auto& h : plan_.heaters) set_pwm(h.out, 0, now + 0.2);
    for (auto& f : plan_.fans) set_pwm(f.out, 0, now + 0.2);
    // ADC reports (with the heater range checks).
    if (!plan_.sensors.empty()) {
        uint64_t startc = io_.clock_at(now + 0.5);
        send(adc_query_cmds(plan_, uint32_t(startc), plan_.clock_freq));
    }
    // TMC2209 drivers.
    int ok = 0, bad = 0;
    for (size_t i = 0; i < plan_.steppers.size(); i++) {
        if (plan_.steppers[i].tmc < 0) continue;
        tmc_setup(i);
        (sr_[i].tmc_ok ? ok : bad)++;
    }
    std::string r = std::to_string(plan_.heaters.size()) + " heaters, " + std::to_string(plan_.fans.size()) + " fans, " +
                    std::to_string(plan_.steppers.size()) + " steppers";
    if (ok + bad) r += ", TMC2209: " + std::to_string(ok) + " ok, " + std::to_string(bad) + " not answering";
    return r;
}

// ---- heaters

void Machine::heater_set_pwm(size_t i, double read_time, double value) {
    HeaterRun& h = hr_[i];
    if (h.target <= 0 || shutdown_) value = 0;
    if ((read_time < h.next_pwm_time || h.last_pwm == 0) && std::fabs(value - h.last_pwm) < 0.05) return;
    double pwm_time = read_time + PWM_DELAY;
    h.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME;
    h.last_pwm = value;
    if (value > h.max_power_seen) h.max_power_seen = value;
    h.pending = true;
    h.pending_value = value;
    h.pending_time = pwm_time;
}

void Machine::heater_temp_update(size_t i, double temp, double now) {
    const HeaterObj& o = plan_.heaters[i];
    HeaterRun& h = hr_[i];
    double time_diff = now - h.last_time;
    if (!h.have_temp) {
        h.smoothed = temp;
        h.prev_time = now;
        h.prev_temp = temp;
        time_diff = 0;
    }
    h.last_temp = temp;
    h.last_time = now;
    // control (heaters.py ControlPID / ControlBangBang; pid_calibrate.py while tuning)
    if (h.out_test) {
        // the output check drives the pin itself
    } else if (h.tuning) {
        tune_update(i, temp, now);
    } else if (o.pid) {
        double dt = now - h.prev_time;
        double temp_diff = temp - h.prev_temp;
        double deriv;
        if (dt >= o.smooth_time)
            deriv = temp_diff / dt;
        else
            deriv = (h.prev_deriv * (o.smooth_time - dt) + temp_diff) / o.smooth_time;
        double err = h.target - temp;
        double integ_max = o.ki > 0 ? o.max_power / o.ki : 0;
        double integ = h.prev_integ + err * dt;
        integ = integ < 0 ? 0 : integ > integ_max ? integ_max : integ;
        double co = o.kp * err + o.ki * integ - o.kd * deriv;
        double bounded = co < 0 ? 0 : co > o.max_power ? o.max_power : co;
        heater_set_pwm(i, now, bounded);
        h.prev_temp = temp;
        h.prev_time = now;
        h.prev_deriv = deriv;
        if (co == bounded) h.prev_integ = integ;
    } else {
        if (h.heating && temp >= h.target + o.max_delta)
            h.heating = false;
        else if (!h.heating && temp <= h.target - o.max_delta)
            h.heating = true;
        heater_set_pwm(i, now, h.heating ? o.max_power : 0.0);
    }
    double adj = o.smooth_time > 0 ? time_diff / o.smooth_time : 1;
    if (adj > 1) adj = 1;
    h.smoothed += (temp - h.smoothed) * adj;
    h.have_temp = true;
}

void Machine::set_target(size_t i, double t) {
    HeaterRun& h = hr_[i];
    if (t != h.target && t > 0) h.max_power_seen = 0;
    h.target = t;
    if (t <= 0) {
        // switch off now rather than at the next temperature report
        h.target = 0;
        if (h.last_pwm != 0) {
            h.last_pwm = 0;
            h.pending = true;
            h.pending_value = 0;
            h.pending_time = h.last_time + PWM_DELAY;
        }
    }
}

void Machine::heater_check(size_t i, double now) {
    // verify_heater.py check_event, once a second
    const HeaterObj& o = plan_.heaters[i];
    HeaterRun& h = hr_[i];
    if (now < h.next_check) return;
    h.next_check = now + 1.0;
    double temp = (now - h.last_time > 5.0 || !h.have_temp) ? 0.0 : h.smoothed;
    double target = h.target;
    // Not in Klipper: a heater that keeps getting hotter while its output is off has a stuck output
    // (a shorted MOSFET, or a wiring fault bypassing it). Software cannot switch it off then.
    if (h.last_pwm == 0 && h.have_temp && now - h.last_time < 5.0) {
        if (h.off_since < 0) h.off_since = now, h.off_min = 1e9;
        double settle = o.name == "heater_bed" ? 60.0 : 20.0;   // heat already in the block still arrives
        if (now - h.off_since >= settle) {
            if (h.smoothed < h.off_min) h.off_min = h.smoothed;
            if (h.smoothed > h.off_min + 10.0) {
                power_off_now_ = true;
                shutdown("Heater " + o.name + " is getting HOTTER WITH ITS OUTPUT OFF (up " + fmt("%.0f C", h.smoothed - h.off_min) +
                             " while off for " + fmt("%.0f s", now - h.off_since) +
                             "): the MOSFET may be stuck on (shorted). SWITCH OFF THE 12-24 V POWER SUPPLY NOW.", now);
                return;
            }
        }
    } else if (h.last_pwm != 0) {
        h.off_since = -1;
    }
    if (h.have_temp && h.smoothed > o.max_temp) {
        shutdown("Heater " + o.name + " above max_temp (" + fmt("%.1f", h.smoothed) + " > " + fmt("%.0f", o.max_temp) + ")", now);
        return;
    }
    if (temp >= target - o.hysteresis || target <= 0) {
        h.approaching = h.starting = false;
        if (temp <= target + o.hysteresis) h.error = 0;
        h.last_target = target;
        return;
    }
    h.error += (target - o.hysteresis) - temp;
    if (!h.approaching) {
        if (target != h.last_target) {
            h.approaching = h.starting = true;
            h.goal_temp = temp + o.heating_gain;
            h.goal_time = now + o.check_gain_time;
        } else if (h.error >= o.max_error) {
            shutdown("Heater " + o.name + " not heating at expected rate (verify_heater)", now);
            return;
        }
    } else if (temp >= h.goal_temp) {
        h.starting = false;
        h.error = 0;
        h.goal_temp = temp + o.heating_gain;
        h.goal_time = now + o.check_gain_time;
    } else if (now >= h.goal_time) {
        h.approaching = false;
    } else if (h.starting) {
        h.goal_temp = std::min(h.goal_temp, temp + o.heating_gain);
    }
    h.last_target = target;
}

// ---- PID_CALIBRATE (pid_calibrate.py ControlAutoTune): heat at full power until the target, off
// until 5 C below it, again and again; the swing and period give the PID values (Astrom-Hagglund
// estimate of Ku and Tu, then Ziegler-Nichols).

static const double TUNE_PID_DELTA = 5.0;

std::string Machine::start_pid_tune(size_t i, double target, double now) {
    const HeaterObj& o = plan_.heaters[i];
    Test& t = test("pid_" + o.name);
    if (t.state == "running") return "error: already tuning " + o.name;
    for (auto& tt : tests_)
        if (tt.id == "heat_" + o.name && tt.state == "running") return "error: the " + o.name + " heat test is running";
    const TempSensorObj& sn = plan_.sensors[size_t(o.sensor)];
    if (sn.state() != 1) return "error: " + o.name + " sensor: " + sn.describe();
    if (target < o.min_temp || target > o.max_temp - 10)
        return "error: TARGET must be at least 10 C below max_temp (" + fmt("%.0f", o.max_temp) + ")";
    HeaterRun& h = hr_[i];
    h.tuning = true;
    h.tune_heating = false;
    h.tune_temp = target;
    h.peak = 0;
    h.peak_time = 0;
    h.peaks.clear();
    h.tune_started = now;
    h.target = target;
    h.max_power_seen = 0;
    t.state = "running";
    t.started = now;
    t.detail = "heating to " + fmt("%.0f C", target);
    return "ok PID tuning " + o.name + " at " + fmt("%.0f C", target) + ": it heats and cools about 6 times (a few minutes)";
}

void Machine::tune_check_peaks(size_t i) {
    HeaterRun& h = hr_[i];
    h.peaks.push_back({h.peak, h.peak_time});
    h.peak = h.tune_heating ? 9999999.0 : -9999999.0;
}

void Machine::tune_update(size_t i, double temp, double now) {
    const HeaterObj& o = plan_.heaters[i];
    HeaterRun& h = hr_[i];
    if (h.tune_heating && temp >= h.target) {
        h.tune_heating = false;
        tune_check_peaks(i);
        h.target = h.tune_temp - TUNE_PID_DELTA;
    } else if (!h.tune_heating && temp <= h.target) {
        h.tune_heating = true;
        tune_check_peaks(i);
        h.target = h.tune_temp;
    }
    if (h.tune_heating) {
        heater_set_pwm(i, now, o.max_power);
        if (temp < h.peak) h.peak = temp, h.peak_time = now;
    } else {
        heater_set_pwm(i, now, 0.0);
        if (temp > h.peak) h.peak = temp, h.peak_time = now;
    }
}

void Machine::tune_pid(size_t i, size_t pos, double& kp, double& ki, double& kd) const {
    const HeaterRun& h = hr_[i];
    double temp_diff = h.peaks[pos].first - h.peaks[pos - 1].first;
    double time_diff = h.peaks[pos].second - h.peaks[pos - 2].second;
    double amplitude = 0.5 * std::fabs(temp_diff);
    double ku = 4.0 * plan_.heaters[i].max_power / (M_PI * amplitude);
    double tu = time_diff;
    double ti = 0.5 * tu, td = 0.125 * tu;
    kp = 0.6 * ku * 255.0;
    ki = kp / ti;
    kd = kp * td;
}

std::string Machine::save_config(double now) {
    (void)now;
    if (!io_.config_text || !io_.save_config) return "error: saving printer.cfg is not available here";
    std::string text = io_.config_text(), changed;
    for (size_t i = 0; i < hr_.size(); i++) {
        if (!hr_[i].unsaved) continue;
        const std::string& n = plan_.heaters[i].name;
        text = config_set_option(text, n, "control", "pid");
        text = config_set_option(text, n, "pid_Kp", fmt("%.3f", hr_[i].new_kp));
        text = config_set_option(text, n, "pid_Ki", fmt("%.3f", hr_[i].new_ki));
        text = config_set_option(text, n, "pid_Kd", fmt("%.3f", hr_[i].new_kd));
        changed += n + " ";
    }
    if (changed.empty()) return "error: nothing to save (run a PID tune first)";
    for (auto& h : hr_)
        if (h.target > 0) return "error: switch the heaters off first (saving restarts the board)";
    std::string err;
    if (!io_.save_config(text, &err)) return "error: could not save printer.cfg: " + err;
    for (auto& h : hr_) h.unsaved = false;
    return "ok saved the PID values of " + changed + "to printer.cfg; the board restarts with them";
}

std::string Machine::heater_log_csv() const {
    std::string out = "time_s,heater,temp_c,target_c,power,adc_raw,clock_error_ms\n";
    double t0 = -1;
    for (size_t i = 0; i < hr_.size(); i++)
        for (auto& smp : hr_[i].hist)
            if (t0 < 0 || smp.t < t0) t0 = smp.t;
    for (size_t i = 0; i < hr_.size(); i++) {
        const HeaterRun& h = hr_[i];
        size_t n = h.hist.size();
        for (size_t k = 0; k < n; k++) {
            const auto& smp = h.hist[(h.hist_pos + k) % n];
            char b[120];
            snprintf(b, sizeof b, "%.1f,%s,%.2f,%.1f,%.3f,%u,%.1f\n", smp.t - t0, plan_.heaters[i].name.c_str(), smp.temp,
                     smp.target, smp.power, unsigned(smp.raw), smp.clock_err_ms);
            out += b;
        }
    }
    return out;
}

// ---- heater output check: 5 s watching, 5 pulses of 1 s full power (watch the output's LED), then
// 40 s off. The heater must warm during the pulses and stop warming soon after: if it keeps rising
// with the output off, the MOSFET is stuck on.

std::string Machine::start_output_test(size_t i, double now) {
    const HeaterObj& o = plan_.heaters[i];
    Test& t = test("out_" + o.name);
    HeaterRun& h = hr_[i];
    if (t.state == "running") return "error: already running";
    if (h.target > 0 || h.tuning) return "error: switch " + o.name + " off first";
    const TempSensorObj& sn = plan_.sensors[size_t(o.sensor)];
    if (sn.state() != 1) return "error: " + o.name + " sensor: " + sn.describe();
    if (h.smoothed > 60) return "error: let " + o.name + " cool below 60 C first (it is " + fmt("%.0f C", h.smoothed) + ")";
    h.out_test = true;
    h.out_on = false;
    h.out_t0 = now;
    h.out_base = h.smoothed;
    h.out_peak = -1e9;
    t.state = "running";
    t.started = now;
    t.detail = "watching for 5 s with the output OFF";
    return "ok " + o.name + " output check: watch the heater LED on the board. 5 s off, then 5 blinks (1 s on, 1 s off), then 40 s off";
}

void Machine::run_output_test(size_t i, double now) {
    const HeaterObj& o = plan_.heaters[i];
    HeaterRun& h = hr_[i];
    Test& t = test("out_" + o.name);
    double e = now - h.out_t0, temp = h.smoothed;
    auto drive = [&](bool on) {
        if (on == h.out_on) return;
        h.out_on = on;
        h.last_pwm = on ? 1.0 : 0.0;
        if (on) h.off_since = -1;
        set_pwm(o.out, on ? 1.0 : 0.0, now + 0.05);
    };
    if (e < 5) {
        h.out_base = temp;
        return;
    }
    if (e < 15) {
        int k = int(e - 5);
        drive(k % 2 == 0);
        t.detail = std::string(h.out_on ? "ON: the heater LED should be LIT" : "OFF: the heater LED should be DARK") +
                   " (blink " + std::to_string(k / 2 + 1) + " of 5), " + fmt("%.1f C", temp);
        h.out_after_on = temp;
        return;
    }
    drive(false);
    if (temp > h.out_peak) h.out_peak = temp, h.out_peak_t = e;
    if (e < 30) h.out_mid = temp;
    if (e < 55) {
        t.detail = "OFF: the LED should be DARK; watching the temperature (" + fmt("%.1f C", temp) + ", " + fmt("%.0f s left)", 55 - e);
        return;
    }
    h.out_test = false;
    double rise = h.out_peak - h.out_base, late = temp - h.out_mid;
    std::string nums = "start " + fmt("%.1f C", h.out_base) + ", end of blinks " + fmt("%.1f C", h.out_after_on) + ", peak " +
                       fmt("%.1f C", h.out_peak) + " at " + fmt("%.0f s", h.out_peak_t) + ", end " + fmt("%.1f C", temp);
    bool bed = o.name == "heater_bed";
    if (late > 2.0) {
        power_off_now_ = true;
        finish(t, "failed", "still getting hotter 15-40 s after switching off (+" + fmt("%.1f C", late) + "): the output may be "
                            "stuck ON (shorted MOSFET). Switch off the 12-24 V supply and check the board. " + nums);
    } else if (rise < (bed ? 0.3 : 1.0)) {
        finish(t, "failed", "no temperature rise during the blinks: heater not connected, wrong heater_pin, no 12-24 V, "
                            "or the output never switches on. Did the LED blink? " + nums);
    } else {
        finish(t, "passed", "it heated during the blinks and stopped heating after them. Check that the LED blinked 5 times "
                            "and stayed dark at the end. " + nums);
    }
}

// ---- fans (fan.py _apply_speed, with kick start)

void Machine::fan_set(size_t i, double speed, double now) {
    const FanObj& o = plan_.fans[i];
    FanRun& f = fr_[i];
    f.requested = speed;
    if (speed < o.off_below) speed = 0;
    double value = speed * o.max_power;
    value = value < 0 ? 0 : value > o.max_power ? o.max_power : value;
    if (value == f.value && f.kick_until == 0) return;
    if (value > 0 && o.kick_start_time > 0 && (f.value == 0 || value - f.value > 0.5) && value < o.max_power) {
        f.value = o.max_power;
        f.kick_until = now + o.kick_start_time;
        f.after_kick = value;
        set_pwm(o.out, o.max_power, now + FAN_DELAY);
        return;
    }
    f.kick_until = 0;
    f.value = value;
    set_pwm(o.out, value, now + FAN_DELAY);
}

// ---- steppers

void Machine::stepper_enable(size_t i, bool on, double now) {
    const StepperObj& s = plan_.steppers[i];
    sr_[i].enabled = on;
    if (s.enable_oid < 0) return;
    bool level = on != s.invert_enable;
    uint32_t clock = clock32(fresh(now) + 0.05);
    uint32_t& last = last_clock_[size_t(s.enable_oid)];
    if (last && int32_t(clock - last) <= 0) clock = last + 1;
    last = clock;
    send({"queue_digital_out oid=" + std::to_string(s.enable_oid) + " clock=" + std::to_string(clock) +
          " on_ticks=" + (level ? "1" : "0")});
}

bool Machine::stepper_position(size_t i, int64_t& pos) {
    Message m;
    if (!io_.query("stepper_get_position oid=" + std::to_string(plan_.steppers[i].oid), "stepper_position",
                   plan_.steppers[i].oid, m, 200))
        return false;
    pos = int32_t(uint32_t(m.get("pos")));
    return true;
}

// ---- shutdown

void Machine::mark_shutdown(const std::string& why) {
    if (!shutdown_ && io_.log) io_.log("SHUTDOWN: " + why);
    if (!shutdown_) shutdown_reason_ = why;
    shutdown_ = true;
    for (auto& h : hr_) h.target = 0, h.pending = false, h.last_pwm = 0, h.tuning = false, h.out_test = false;
    for (auto& f : fr_) f.value = 0, f.kick_until = 0, f.override_active = false;
    for (auto& s : sr_) s.enabled = false, s.busy = false;
    for (auto& t : tests_)
        if (t.state == "running") finish(t, "failed", "stopped: " + shutdown_reason_);
    all_queue_.clear();
    all_running_ = false;
    if (th_) th_->stop(last_now_);
    if (printing()) {
        job_.state = "error";
        job_.msg = "stopped: " + shutdown_reason_;
        job_.ended = last_now_;
    }
}

void Machine::shutdown(const std::string& why, double now) {
    (void)now;
    mark_shutdown(why);   // first: the board's own "shutdown" reply arrives while sending
    send({"emergency_stop"});
}

void Machine::note_board_shutdown(const std::string& why) { mark_shutdown(why); }

void Machine::all_off(double now) {
    if (shutdown_) return;
    for (auto& h : hr_) {
        if (h.out_test && h.last_pwm == 0) h.last_pwm = 1;   // make set_target send the 0
        h.out_test = false, h.tuning = false;
    }
    for (size_t i = 0; i < hr_.size(); i++) set_target(i, 0);
    for (auto& h : hr_)
        if (h.pending) h.pending_time = now + 0.1;
    for (size_t i = 0; i < hr_.size(); i++)
        if (hr_[i].pending) {
            set_pwm(plan_.heaters[i].out, 0, hr_[i].pending_time);
            hr_[i].pending = false;
        }
    for (size_t i = 0; i < fr_.size(); i++) fan_set(i, 0, now);
    for (size_t i = 0; i < sr_.size(); i++) {
        if (sr_[i].busy) send({"trsync_trigger oid=" + std::to_string(plan_.steppers[i].trsync_oid) + " reason=1"});
        if (sr_[i].enabled || sr_[i].busy) stepper_enable(i, false, now);
        sr_[i].busy = false;
    }
}

// ---- messages from the board

void Machine::on_message(const Message& m, double now) {
    if (th_) th_->on_message(m, now);
    if (m.name == "analog_in_state") {
        if (!apply_adc_message(plan_, m)) return;
        if (io_.clock_at && plan_.clock_freq > 0) {
            // the board sends this right after sampling (8 x 1 ms) for the period ending at next_clock - 0.3 s
            uint32_t board_now = uint32_t(m.get("next_clock")) - uint32_t(0.300 * plan_.clock_freq) +
                                 uint32_t(0.008 * plan_.clock_freq);
            double e = double(int32_t(uint32_t(io_.clock_at(now)) - board_now)) / plan_.clock_freq * 1000.0;
            clock_err_ms_ = have_clock_err_ ? clock_err_ms_ + (e - clock_err_ms_) * 0.1 : e;
            have_clock_err_ = true;
        }
        for (size_t i = 0; i < plan_.sensors.size(); i++) {
            auto& s = plan_.sensors[i];
            if (s.oid != m.get("oid")) continue;
            if (sensor_prev_time_[i] >= 0 && now - sensor_prev_time_[i] > 0.05) {
                double r = (s.temp_c - sensor_prev_[i]) / (now - sensor_prev_time_[i]);
                sensor_rate_[i] += (r - sensor_rate_[i]) * 0.3;   // gentle smoothing for the display
            }
            sensor_prev_[i] = s.temp_c;
            sensor_prev_time_[i] = now;
            if (s.heater >= 0 && s.state() == 1) heater_temp_update(size_t(s.heater), s.temp_c, now);
        }
    } else if (m.name == "shutdown" || m.name == "is_shutdown") {
        std::string why = mp_.enum_name_of("static_string_id", m.get("static_string_id"));
        if (why.empty()) why = "code " + std::to_string(m.get("static_string_id"));
        if (why == "ADC out of range" && !shutdown_) {
            // The reading that tripped it never reaches us; show the last ones and the limits.
            for (auto& s : plan_.sensors) {
                if (s.adc < 0) continue;
                char b[160];
                snprintf(b, sizeof b, "; %s last read %.1f C (raw %u, allowed %u..%u)", s.name.c_str(), s.temp_c,
                         unsigned(s.adc * plan_.adc_full_scale + 0.5), unsigned(s.min_value), unsigned(s.max_value));
                why += b;
            }
            for (size_t i = 0; i < hr_.size(); i++) {
                const HeaterRun& h = hr_[i];
                if (h.off_since >= 0 && now - h.off_since > 20 && h.have_temp && h.smoothed > h.off_min + 5) {
                    power_off_now_ = true;
                    why += ". WARNING: " + plan_.heaters[i].name + " output was OFF for the last " +
                           fmt("%.0f s", now - h.off_since) + " while it heated by " + fmt("%.0f C", h.smoothed - h.off_min) +
                           ": the MOSFET may be stuck on. SWITCH OFF THE 12-24 V POWER SUPPLY";
                }
            }
            why += ". Raw below the range = hotter than max_temp; above it = colder than min_temp or the thermistor "
                   "disconnected. The heater log (/heater_log.csv) has the last 5 minutes.";
        }
        mark_shutdown("board shut down: " + why);
    }
}

// ---- periodic work

void Machine::tick(double now) {
    last_now_ = now;
    if (shutdown_) return;
    if (th_) th_->tick(now);
    print_tick(now);
    poll_diag(now);
    // Klipper's idle_timeout: 10 minutes with nothing happening switches heaters and motors off
    bool active = printing() || (th_ && !th_->idle(now));
    for (auto& t : tests_) active |= t.state == "running";
    if (active) last_activity_ = now;
    if (last_activity_ > 0 && now - last_activity_ > 600) {
        last_activity_ = now;
        bool any = false;
        for (size_t i = 0; i < hr_.size(); i++) any |= hr_[i].target > 0, set_target(i, 0);
        for (auto& sr : sr_) any |= sr.enabled;
        if (th_) th_->motors_off(now);
        for (size_t i = 0; i < sr_.size(); i++)
            if (sr_[i].enabled) stepper_enable(i, false, now);
        if (any && io_.log) io_.log("idle for 10 minutes: heaters and motors off");
    }
    for (size_t i = 0; i < hr_.size(); i++) {
        HeaterRun& hh = hr_[i];
        if (hh.have_temp && now >= hh.next_hist) {   // history, one sample a second, 5 minutes
            hh.next_hist = now + 1.0;
            const TempSensorObj& sn = plan_.sensors[size_t(plan_.heaters[i].sensor)];
            HeaterRun::Sample smp{float(now), float(hh.smoothed), float(hh.target), float(hh.last_pwm),
                                  float(clock_err_ms_), uint16_t(sn.adc < 0 ? 0 : sn.adc * plan_.adc_full_scale + 0.5)};
            if (hh.hist.size() < 300) hh.hist.push_back(smp);
            else hh.hist[hh.hist_pos] = smp, hh.hist_pos = (hh.hist_pos + 1) % 300;
        }
        if (hh.tuning) {
            Test& t = test("pid_" + plan_.heaters[i].name);
            if (!hh.tune_heating && hh.peaks.size() >= 12) {
                std::vector<std::pair<double, size_t>> cyc;
                for (size_t pos = 4; pos < hh.peaks.size(); pos++)
                    cyc.push_back({hh.peaks[pos].second - hh.peaks[pos - 2].second, pos});
                std::sort(cyc.begin(), cyc.end());
                double kp, ki, kd;
                tune_pid(i, cyc[cyc.size() / 2].second, kp, ki, kd);
                hh.tuning = false;
                set_target(i, 0);
                HeaterObj& o = plan_.heaters[i];
                o.pid = true, o.kp = kp / 255.0, o.ki = ki / 255.0, o.kd = kd / 255.0;   // used from now on
                hh.prev_integ = 0, hh.prev_deriv = 0;
                hh.unsaved = true, hh.new_kp = kp, hh.new_ki = ki, hh.new_kd = kd;
                finish(t, "passed", "pid_Kp=" + fmt("%.3f", kp) + " pid_Ki=" + fmt("%.3f", ki) + " pid_Kd=" + fmt("%.3f", kd) +
                                        ". In use now; press Save PID (SAVE_CONFIG) to keep them. Heater off.");
            } else if (now - t.started > 1800) {
                hh.tuning = false;
                set_target(i, 0);
                finish(t, "failed", "did not finish in 30 minutes; heater off");
            } else {
                t.detail = (hh.tune_heating ? "heating" : "cooling") + std::string(" ") + fmt("%.1f C", hh.smoothed) +
                           ", swing " + std::to_string(hh.peaks.size()) + " of 12, " + fmt("%.0f s", now - t.started);
            }
        }
        if (hh.out_test) run_output_test(i, now);
        if (hr_[i].pending) {
            hr_[i].pending = false;
            double when = hr_[i].pending_time < now + 0.05 ? now + 0.05 : hr_[i].pending_time;
            set_pwm(plan_.heaters[i].out, hr_[i].pending_value, when);
        }
        heater_check(i, now);
        if (shutdown_) return;
    }
    for (size_t i = 0; i < fr_.size(); i++)
        if (fr_[i].kick_until > 0 && now >= fr_[i].kick_until) {
            double v = fr_[i].after_kick;
            fr_[i].kick_until = 0;
            fr_[i].value = v;
            set_pwm(plan_.fans[i].out, v, now + FAN_DELAY);
        }
    if (now - last_fan_auto_ >= 0.3) {   // heater_fan.py
        last_fan_auto_ = now;
        for (size_t i = 0; i < fr_.size(); i++) {
            const FanObj& f = plan_.fans[i];
            if (!f.heater_fan || fr_[i].override_active) continue;
            const HeaterObj* h = plan_.heater(f.heater);
            if (!h) continue;
            const HeaterRun& r = hr_[size_t(h - plan_.heaters.data())];
            double speed = (r.target > 0 || (r.have_temp && r.smoothed > f.heater_temp)) ? f.fan_speed : 0.0;
            if (speed != fr_[i].requested) fan_set(i, speed, now);
        }
    }
    for (size_t i = 0; i < sr_.size(); i++)
        if (sr_[i].busy && now > sr_[i].busy_until + 5.0) sr_[i].busy = false;   // safety net
    run_tests(now);
}

// ---- TMC2209

bool Machine::tmc_read(const TmcObj& t, uint8_t reg, uint32_t& val, std::string* err) {
    static const char* hexd = "0123456789abcdef";
    std::vector<uint8_t> w = tmc_encode_read(0xf5, uint8_t(t.addr), reg);
    std::string hx = "hex:";
    for (uint8_t b : w) hx += hexd[b >> 4], hx += hexd[b & 15];
    for (int attempt = 0; attempt < 5; attempt++) {   // tmc_uart.py retries 5 times
        Message m;
        if (!io_.query("tmcuart_send oid=" + std::to_string(t.uart_oid) + " write=" + hx + " read=10",
                       "tmcuart_response", t.uart_oid, m, 100))
            continue;
        auto it = m.params.find("read");
        if (it == m.params.end()) continue;
        std::vector<uint8_t> d(it->second.b.begin(), it->second.b.end());
        if (tmc_decode_read(reg, d, val)) return true;
    }
    if (err) *err = "no answer";
    return false;
}

bool Machine::tmc_write(const TmcObj& t, uint8_t reg, uint32_t val, std::string* err) {
    static const char* hexd = "0123456789abcdef";
    std::vector<uint8_t> w = tmc_encode_write(0xf5, uint8_t(t.addr), uint8_t(reg | 0x80), val);
    std::string hx = "hex:";
    for (uint8_t b : w) hx += hexd[b >> 4], hx += hexd[b & 15];
    for (int attempt = 0; attempt < 5; attempt++) {
        uint32_t before = 0, after = 0;
        if (!tmc_read(t, 0x02, before, err)) return false;   // IFCNT
        Message m;
        io_.query("tmcuart_send oid=" + std::to_string(t.uart_oid) + " write=" + hx + " read=0", "tmcuart_response",
                  t.uart_oid, m, 100);
        if (!tmc_read(t, 0x02, after, err)) return false;
        if ((after & 0xff) == ((before + 1) & 0xff)) return true;   // the driver counted the write
    }
    if (err) *err = "writes not acknowledged (IFCNT did not change)";
    return false;
}

std::string Machine::tmc_setup(size_t si) {
    const StepperObj& s = plan_.steppers[si];
    StepperRun& r = sr_[si];
    r.tmc_ok = false;
    if (s.tmc < 0) return r.tmc_status = "no [tmc2209 " + s.name + "] section";
    const TmcObj& t = plan_.tmcs[size_t(s.tmc)];
    std::string err;
    uint32_t ioin = 0;
    if (!tmc_read(t, 0x06, ioin, &err))
        return r.tmc_status = "no answer from the TMC2209 at UART address " + std::to_string(t.addr) +
                              " (is the 12-24 V motor power on? The driver needs it to answer)";
    uint32_t version = ioin >> 24;
    if (version != 0x21) {
        char b[96];
        snprintf(b, sizeof b, "unexpected driver version 0x%02x (a TMC2209 reports 0x21)", unsigned(version));
        return r.tmc_status = b;
    }
    for (auto& reg : t.init_regs)
        if (!tmc_write(t, uint8_t(reg.addr), reg.value, &err))
            return r.tmc_status = "could not set " + reg.name + ": " + err;
    r.tmc_ok = true;
    return r.tmc_status = tmc_status_line(si);
}

std::string Machine::tmc_status_line(size_t si, uint32_t* drv_out) {
    const StepperObj& s = plan_.steppers[si];
    const TmcObj& t = plan_.tmcs[size_t(s.tmc)];
    std::string line = "TMC2209 ok: " + fmt("%.2f A", t.run_current) + " (irun " + std::to_string(t.irun) + "), " +
                       std::to_string(s.microsteps) + " microsteps, " + (t.stealthchop ? "stealthChop" : "spreadCycle");
    uint32_t drv = 0, gstat = 0;
    std::string err;
    if (tmc_read(t, 0x6f, drv, &err)) {
        if (drv_out) *drv_out = drv;
        std::string flags;
        static const struct { int bit; const char* name; } F[] = {
            {1, "over-temperature"}, {0, "temperature warning"}, {2, "short to ground A"}, {3, "short to ground B"},
            {4, "short to supply A"}, {5, "short to supply B"}};
        for (auto& f : F)
            if (drv & (1u << f.bit)) flags += std::string(flags.empty() ? "" : ", ") + f.name;
        if (!flags.empty()) line += "; DRIVER REPORTS: " + flags;
    }
    if (tmc_read(t, 0x01, gstat, &err) && (gstat & 0x02)) line += "; GSTAT drv_err (driver shut itself down)";
    if (gstat & 0x04) line += "; GSTAT uv_cp (motor supply too low)";
    return line;
}

std::string Machine::dump_tmc(size_t si) {
    const StepperObj& s = plan_.steppers[si];
    if (s.tmc < 0) return "error: no [tmc2209 " + s.name + "] section";
    const TmcObj& t = plan_.tmcs[size_t(s.tmc)];
    static const struct { const char* name; int addr; } R[] = {
        {"GCONF", 0x00}, {"GSTAT", 0x01}, {"IFCNT", 0x02}, {"IOIN", 0x06}, {"IHOLD_IRUN (write-only, sent)", -1},
        {"TSTEP", 0x12}, {"MSCNT", 0x6a}, {"CHOPCONF", 0x6c}, {"DRV_STATUS", 0x6f}, {"PWMCONF", 0x70},
        {"PWM_SCALE", 0x71}, {"PWM_AUTO", 0x72}, {"SG_RESULT", 0x41}};
    std::string out = "ok " + s.name + " TMC2209 at address " + std::to_string(t.addr) + ":";
    for (auto& r : R) {
        if (r.addr < 0) {
            for (auto& ir : t.init_regs)
                if (ir.name == "IHOLD_IRUN") {
                    char b[64];
                    snprintf(b, sizeof b, "\nIHOLD_IRUN: 0x%08lx (sent)", (unsigned long)ir.value);
                    out += b;
                }
            continue;
        }
        uint32_t v = 0;
        std::string err;
        if (tmc_read(t, uint8_t(r.addr), v, &err)) {
            char b[64];
            snprintf(b, sizeof b, "\n%s: 0x%08lx", r.name, (unsigned long)v);
            out += b;
        } else {
            out += std::string("\n") + r.name + ": no answer";
        }
    }
    return out;
}

// ---------------------------------------------------------------- tests

Machine::Test& Machine::test(const std::string& id) {
    for (auto& t : tests_)
        if (t.id == id) return t;
    tests_.push_back({id, id});
    return tests_.back();
}

void Machine::finish(Test& t, const std::string& state, const std::string& detail) {
    t.state = state;
    t.detail = detail;
    if (io_.log) io_.log("test " + t.id + ": " + state + " - " + detail);
}

std::string Machine::start_heat_test(size_t i, double target, double hold, double now) {
    const HeaterObj& o = plan_.heaters[i];
    Test& t = test("heat_" + o.name);
    if (t.state == "running") return "error: the " + o.name + " test is already running";
    const TempSensorObj& s = plan_.sensors[size_t(o.sensor)];
    if (s.state() != 1) return "error: " + o.name + " sensor: " + s.describe() + " - fix the thermistor first";
    if (target < o.min_temp || target > o.max_temp)
        return "error: TARGET must be within min_temp..max_temp (" + fmt("%.0f", o.min_temp) + ".." + fmt("%.0f", o.max_temp) + ")";
    if (target <= hr_[i].smoothed + 5)
        return "error: TARGET must be at least 5 C above the current " + fmt("%.1f C", hr_[i].smoothed);
    HeatTest& h = heat_tests_[i];
    h = HeatTest();
    h.heater = int(i);
    h.target = target;
    h.hold = hold;
    h.timeout = o.name == "heater_bed" ? 900 : 300;
    set_target(i, target);
    t.state = "running";
    t.started = now;
    t.detail = "heating to " + fmt("%.0f", target) + " C";
    return "ok heating " + o.name + " to " + fmt("%.0f", target) + " C, then holding it for " + fmt("%.0f", hold) + " s";
}

std::string Machine::start_motor_test(size_t i, double rpm, double seconds, int mode, double now, double sg_speed,
                                      double sg_dist) {
    bool buzz_only = mode == 1 || mode == 2;
    const StepperObj& s = plan_.steppers[i];
    Test& t = test((mode == 3 ? "sg_" : "motor_") + s.name);
    if (t.state == "running" || sr_[i].busy) return "error: " + s.name + " is already moving";
    if (mode == 3) {
        if (s.endstop < 0 || plan_.endstops[size_t(s.endstop)].tmc < 0)
            return "error: " + s.name + " has no sensorless endstop (endstop_pin: tmc2209_" + s.name + ":virtual_endstop)";
        if (!sr_[i].tmc_ok) return "error: the TMC2209 of " + s.name + " is not answering (run the TMC2209 check)";
        rpm = sg_speed / s.rotation_distance * 60.0;
        seconds = sg_dist / sg_speed;
    }
    if (printing()) return "error: a print is running";
    if (th_ && !th_->idle(now)) return "error: moves are still running";
    if (th_) th_->stepper_used_elsewhere(i);
    MotorTest& mt = motor_tests_[i];
    mt = MotorTest();
    mt.stepper = int(i);
    mt.buzz_only = buzz_only;
    if (mode == 3) {
        mt.sg = true;
        mt.sg_speed = sg_speed;
        mt.sg_endstop = s.endstop;
        mt.sg_tmc = plan_.endstops[size_t(s.endstop)].tmc;
        // TSTEP (time between 1/256 microsteps, in 12 MHz driver clocks) while cruising
        mt.sg_tstep = 12e6 / (sg_speed * s.steps_per_mm() * (256.0 / s.microsteps));
        sg_registers(i, true, plan_.tmcs[size_t(mt.sg_tmc)].sgthrs);
    }
    mt.have_start_pos = stepper_position(i, mt.start_pos);
    double f = plan_.clock_freq;
    int spr = s.steps_per_rev();
    std::vector<std::string> cmds;
    std::string T = std::to_string(s.trsync_oid), S = std::to_string(s.oid);
    cmds.push_back("trsync_start oid=" + T + " report_clock=0 report_ticks=0 expire_reason=0");
    cmds.push_back("stepper_stop_on_trigger oid=" + S + " trsync_oid=" + T);
    now = fresh(now);   // the position query above took time
    double t0 = now + 0.5;
    cmds.push_back("reset_step_clock oid=" + S + " clock=" + std::to_string(clock32(t0)));
    int dir = -1;
    uint64_t ticks = 0, pause = 0;
    auto step = [&](int d, uint32_t interval, uint32_t count) {
        while (count) {
            if (d != dir) {
                dir = d;
                cmds.push_back("set_next_step_dir oid=" + S + " dir=" + std::to_string(d ^ (s.invert_dir ? 1 : 0)));
            }
            uint32_t first = interval;
            if (pause) {   // a pause before a move: delay its first step
                cmds.push_back("queue_step oid=" + S + " interval=" + std::to_string(pause + interval) + " count=1 add=0");
                ticks += pause + interval;
                pause = 0;
                count--;
                continue;
            }
            uint32_t n = count > 60000 ? 60000 : count;
            cmds.push_back("queue_step oid=" + S + " interval=" + std::to_string(first) + " count=" + std::to_string(n) + " add=0");
            ticks += uint64_t(first) * n;
            count -= n;
        }
    };
    auto run = [&](int d, double steps_per_s, double secs) {
        const int CH = 6;
        const double ramp = 0.5;
        for (int k = 0; k < CH; k++) {   // speed up
            double v = steps_per_s * (k + 0.5) / CH;
            step(d, uint32_t(f / v + 0.5), uint32_t(std::max(1.0, std::round(v * ramp / CH))));
        }
        step(d, uint32_t(f / steps_per_s + 0.5), uint32_t(std::max(1.0, std::round(steps_per_s * secs))));
        for (int k = CH - 1; k >= 0; k--) {   // slow down
            double v = steps_per_s * (k + 0.5) / CH;
            step(d, uint32_t(f / v + 0.5), uint32_t(std::max(1.0, std::round(v * ramp / CH))));
        }
    };
    auto buzz = [&]() {   // Klipper STEPPER_BUZZ: 1 mm forward and back, 10 times
        uint32_t n = uint32_t(std::max(1.0, std::round(BUZZ_DISTANCE / s.rotation_distance * spr)));
        uint32_t iv = uint32_t(f / (BUZZ_VELOCITY / s.rotation_distance * spr) + 0.5);
        for (int k = 0; k < 10; k++) {
            step(1, iv, n);
            pause = uint64_t(0.05 * f);
            step(0, iv, n);
            pause = uint64_t(0.45 * f);
        }
        pause = 0;
    };
    auto shake = [&]() {   // a clearly visible shake: 1/8 turn forward and back, 10 times, at 60 rpm
        uint32_t n = uint32_t(std::max(1, spr / 8));
        uint32_t iv = uint32_t(f / double(spr) + 0.5);
        for (int k = 0; k < 10; k++) {
            step(1, iv, n);
            pause = uint64_t(0.1 * f);
            step(0, iv, n);
            pause = uint64_t(0.1 * f);
        }
        pause = 0;
    };
    double sps = rpm / 60.0 * spr;
    if (f / sps < 4.0 * s.pulse_ticks + 100) return "error: RPM too high for this stepper's step pulse";
    if (mode == 1) {
        mt.phases.push_back({t0, "buzz (Klipper STEPPER_BUZZ): 1 mm forward and back, 10 times"});
        buzz();
    } else if (mode == 2) {
        mt.phases.push_back({t0, "shake: 1/8 turn forward and back, 10 times"});
        shake();
    } else if (mode == 3) {
        const TmcObj& tm = plan_.tmcs[size_t(mt.sg_tmc)];
        std::string a = "driver_SGTHRS " + std::to_string(tm.sgthrs) + ": forward and back at " + fmt("%.0f mm/s", sg_speed);
        mt.phases.push_back({t0, a});
        run(1, sps, seconds);
        pause = uint64_t(0.3 * f);
        run(0, sps, seconds);
        pause = uint64_t(0.8 * f);
        mt.sg_switch_at = t0 + double(ticks) / f + 0.2;   // during the pause
        mt.phases.push_back({t0 + double(ticks + pause) / f, "SGTHRS 255 (wiring check: DIAG must go high): forward and back"});
        run(1, sps, seconds);
        pause = uint64_t(0.3 * f);
        run(0, sps, seconds);
    } else {
        mt.phases.push_back({t0, "CW (forward) at " + fmt("%.0f", rpm) + " rpm"});
        run(1, sps, seconds);
        pause = uint64_t(0.5 * f);
        mt.phases.push_back({t0 + double(ticks + pause) / f, "CCW (back) at " + fmt("%.0f", rpm) + " rpm"});
        run(0, sps, seconds);
        pause = uint64_t(0.5 * f);
        mt.phases.push_back({t0 + double(ticks + pause) / f, "shake: 1/8 turn forward and back, 10 times"});
        shake();
    }
    mt.t0 = t0;
    mt.end = t0 + double(ticks) / f;
    mt.cruise_check = t0 + 3.0;
    stepper_enable(i, true, now);
    std::string err;
    if (!send(cmds, &err)) {
        stepper_enable(i, false, now);
        return "error: could not send the moves: " + err;
    }
    sr_[i].busy = true;
    sr_[i].busy_until = mt.end;
    t.state = "running";
    t.started = now;
    t.detail = "starting";
    if (mode == 3)
        return "ok " + s.name + ": stallGuard test, " + fmt("%.0f mm", sg_dist) + " forward and back twice at " +
               fmt("%.0f mm/s", sg_speed) + " (" + fmt("%.0f s", mt.end - now) + "). The axis must have room both ways.";
    return "ok " + s.name + ": " + (mode == 1 ? std::string("buzz") : mode == 2 ? std::string("shake") : fmt("%.0f rpm", rpm) + " CW " + fmt("%.0f s", seconds) +
                                    ", CCW " + fmt("%.0f s", seconds) + ", then shake") + " (" + fmt("%.0f s", mt.end - now) + ")";
}

std::string Machine::start_fan_test(size_t i, double now) {
    Test& t = test("fan_" + plan_.fans[i].name);
    if (t.state == "running") return "error: already running";
    fan_tests_[i].fan = int(i);
    fan_tests_[i].t0 = now;
    fr_[i].override_active = true;
    fan_set(i, 1.0, now);
    t.state = "running";
    t.started = now;
    t.detail = "100 % on " + plan_.fans[i].out.pin + " (watch the fan and the board's LED next to the port)";
    return "ok fan " + plan_.fans[i].name + " (" + plan_.fans[i].out.pin + "): 100 % for 3 s, 50 % for 3 s, then off";
}

std::string Machine::start_led_test(double now) {
    if (plan_.neopixels.empty()) return "error: no [neopixel] in printer.cfg";
    Test& t = test("led");
    if (t.state == "running") return "error: already running";
    led_test_.t0 = now;
    led_test_.step = -1;
    t.state = "running";
    t.started = now;
    t.detail = "red";
    return "ok LEDs: red, green, blue, white, 1 s each";
}

std::string Machine::start_all(double now) {
    Test& t = test("all");
    if (t.state == "running") return "error: already running";
    for (auto& h : hr_)
        if (h.target > 0) return "error: switch the heaters off first (TURN_OFF_HEATERS): the run starts with the TMC check";
    t.state = "running";
    t.started = now;
    t.detail = "starting";
    std::string r = "ok running: ";
    // The TMC check first, on its own: it holds the board link for a few seconds, which must not
    // happen while a heater is on (the board turns a heater off if not refreshed within 5 s).
    if (!plan_.tmcs.empty()) {
        r += tmc_check_all(now) + "; ";
    }
    all_queue_.clear();
    if (!plan_.neopixels.empty()) all_queue_.push_back("led");
    for (auto& f : plan_.fans) all_queue_.push_back("fan_" + f.name);
    for (auto& s : plan_.steppers) all_queue_.push_back("motor_" + s.name);
    if (HeaterObj* h = plan_.heater("extruder")) {
        size_t hi = size_t(h - plan_.heaters.data());
        std::string hr = start_heat_test(hi, 75, 60, now);
        r += "hotend to 75 C (" + hr + "), ";
        if (hr.rfind("error", 0) == 0) finish(test("heat_extruder"), "failed", hr);
    }
    for (auto& q : all_queue_) r += q + " ";
    all_running_ = true;
    return r;
}

std::string Machine::tmc_check_all(double now) {
    Test& t = test("tmc");
    t.state = "running";
    t.started = now;
    int okc = 0;
    std::string bad, r;
    for (size_t i = 0; i < plan_.steppers.size(); i++) {
        if (plan_.steppers[i].tmc < 0) continue;
        std::string st = tmc_setup(i);
        r += "\n" + plan_.steppers[i].name + ": " + st;
        if (sr_[i].tmc_ok) okc++;
        else bad += plan_.steppers[i].name + " ";
    }
    finish(t, bad.empty() ? "passed" : "failed", std::to_string(okc) + " drivers ok" + (bad.empty() ? "" : "; not answering: " + bad));
    return (bad.empty() ? "TMC check passed" : "TMC check: some drivers did not answer") + r;
}

void Machine::abort_test(const std::string& id, double now, const std::string& why) {
    for (size_t i = 0; i < plan_.heaters.size(); i++)
        if (id == "heat_" + plan_.heaters[i].name && test(id).state == "running") {
            set_target(i, 0);
            finish(test(id), "aborted", why + "; heater off");
        }
    for (size_t i = 0; i < plan_.heaters.size(); i++)
        if (id == "out_" + plan_.heaters[i].name && test(id).state == "running") {
            hr_[i].out_test = false;
            hr_[i].out_on = false;
            hr_[i].last_pwm = 0;
            set_pwm(plan_.heaters[i].out, 0, now + 0.05);
            finish(test(id), "aborted", why + "; heater off");
        }
    for (size_t i = 0; i < plan_.heaters.size(); i++)
        if (id == "pid_" + plan_.heaters[i].name && test(id).state == "running") {
            hr_[i].tuning = false;
            set_target(i, 0);
            finish(test(id), "aborted", why + "; heater off");
        }
    for (size_t i = 0; i < plan_.steppers.size(); i++)
        if (id == "sg_" + plan_.steppers[i].name && test(id).state == "running") {
            send({"trsync_trigger oid=" + std::to_string(plan_.steppers[i].trsync_oid) + " reason=1"});
            sg_registers(i, false, plan_.tmcs[size_t(motor_tests_[i].sg_tmc)].sgthrs);
            motor_tests_[i].sg = false;
            stepper_enable(i, false, now);
            sr_[i].busy = false;
            if (th_) th_->stepper_used_elsewhere(i);
            finish(test(id), "aborted", why + "; motor stopped and driver off");
        }
    for (size_t i = 0; i < plan_.steppers.size(); i++)
        if (id == "motor_" + plan_.steppers[i].name && (test(id).state == "running" || sr_[i].busy)) {
            send({"trsync_trigger oid=" + std::to_string(plan_.steppers[i].trsync_oid) + " reason=1"});
            stepper_enable(i, false, now);
            sr_[i].busy = false;
            if (test(id).state == "running") finish(test(id), "aborted", why + "; motor stopped and driver off");
        }
    for (size_t i = 0; i < plan_.fans.size(); i++)
        if (id == "fan_" + plan_.fans[i].name && test(id).state == "running") {
            fr_[i].override_active = false;
            fan_set(i, 0, now);
            finish(test(id), "aborted", why);
        }
    if (id == "led" && !plan_.neopixels.empty() && test("led").state == "running") {
        for (auto& n : plan_.neopixels) io_.neopixel_write(n.oid, n.data, nullptr);
        finish(test("led"), "aborted", why);
    }
    if (id == "all" && test("all").state == "running") {
        all_queue_.clear();
        all_running_ = false;
        for (auto& t : tests_)
            if (t.id != "all" && t.state == "running") abort_test(t.id, now, why);
        finish(test("all"), "aborted", why);
    }
}

// ---- STALLGUARD_TEST: what Klipper's Sensorless_Homing.md asks you to find by trial, measured instead.
// TMC2209: DIAG goes high when SG_RESULT <= 2 * SGTHRS (while TCOOLTHRS >= TSTEP > TPWMTHRS); SG_RESULT is
// high while the motor runs freely and drops towards 0 under load.
void Machine::sg_registers(size_t i, bool sensorless, int sgthrs) {
    const StepperObj& s = plan_.steppers[i];
    if (s.endstop < 0) return;
    int ti = plan_.endstops[size_t(s.endstop)].tmc;
    if (ti < 0) return;
    const TmcObj& t = plan_.tmcs[size_t(ti)];
    std::string err;
    if (sensorless) {   // as tmc.py does for homing: stealthChop on, stallGuard active at every speed
        tmc_write(t, 0x13, 0, &err);
        tmc_write(t, 0x00, t.gconf & ~(1u << 2), &err);
        if (t.tcoolthrs == 0) tmc_write(t, 0x14, 0xfffff, &err);
    } else {
        tmc_write(t, 0x13, t.tpwmthrs, &err);
        tmc_write(t, 0x00, t.gconf, &err);
        tmc_write(t, 0x14, t.tcoolthrs, &err);
    }
    tmc_write(t, 0x40, uint32_t(sgthrs), &err);
}

void Machine::sg_sample(size_t i, double now) {
    MotorTest& m = motor_tests_[i];
    const TmcObj& t = plan_.tmcs[size_t(m.sg_tmc)];
    if (!m.sg_switched && now >= m.sg_switch_at) {
        std::string err;
        tmc_write(t, 0x40, 255, &err);
        m.sg_switched = true;
    }
    if (now < m.t0 || now - m.sg_last < 0.04) return;
    m.sg_last = now;
    uint32_t tstep = 0, sgr = 0;
    std::string err;
    if (!tmc_read(t, 0x12, tstep, &err) || !tmc_read(t, 0x41, sgr, &err)) {
        m.sg_err_reads++;
        return;
    }
    tstep &= 0xfffff;
    // cruising only: TSTEP within 25 % of the cruise value (speeding up / slowing down / stopped excluded)
    if (std::fabs(double(tstep) - m.sg_tstep) > 0.25 * m.sg_tstep) return;
    // only samples well inside one phase (not across the SGTHRS switch)
    int ph = m.sg_switched ? 1 : 0;
    const EndstopObj& e = plan_.endstops[size_t(m.sg_endstop)];
    Message msg;
    if (!io_.query || !io_.query("endstop_query_state oid=" + std::to_string(e.oid), "endstop_state", e.oid, msg, 200)) {
        m.sg_err_reads++;
        return;
    }
    bool high = (msg.get("pin_value") != 0) != e.invert;
    m.sg_vals[ph].push_back(int(sgr & 0x3ff));
    m.diag_n[ph]++;
    m.diag_hi[ph] += high ? 1 : 0;
}

std::string Machine::sg_report(size_t i, bool& ok) {
    MotorTest& m = motor_tests_[i];
    const TmcObj& t = plan_.tmcs[size_t(m.sg_tmc)];
    const EndstopObj& e = plan_.endstops[size_t(m.sg_endstop)];
    std::vector<int> all = m.sg_vals[0];
    all.insert(all.end(), m.sg_vals[1].begin(), m.sg_vals[1].end());
    ok = false;
    if (all.empty())
        return "No readings while cruising (" + std::to_string(m.sg_err_reads) +
               " failed reads). Is the motor turning? Try a longer DIST.";
    std::sort(all.begin(), all.end());
    int lo = all.front(), hi = all.back();
    double avg = 0;
    for (int v : all) avg += v;
    avg /= double(all.size());
    int p10 = all[all.size() / 10];
    std::string d = "SG_RESULT while moving freely at " + fmt("%.0f mm/s", m.sg_speed) + ": min " + std::to_string(lo) +
                    ", typical " + fmt("%.0f", avg) + ", max " + std::to_string(hi) + " (" + std::to_string(all.size()) +
                    " readings; it falls towards 0 when the motor is blocked). ";
    d += "DIAG on " + e.pin + ": with driver_SGTHRS " + std::to_string(t.sgthrs) + " (triggers at SG_RESULT <= " +
         std::to_string(2 * t.sgthrs) + ") high in " + std::to_string(m.diag_hi[0]) + " of " + std::to_string(m.diag_n[0]) +
         " readings; with 255 (must trigger) high in " + std::to_string(m.diag_hi[1]) + " of " + std::to_string(m.diag_n[1]) + ". ";
    bool wired = m.diag_n[1] > 0 && m.diag_hi[1] * 2 >= m.diag_n[1];
    if (m.diag_n[1] == 0) {
        d += "No readings in the wiring-check half: inconclusive. ";
    } else if (!wired) {
        d += "DIAG NEVER REACHES " + e.pin + ": fit the SKR Pico's DIAG jumper for this driver (and check diag_pin). ";
        return d;
    } else {
        d += "DIAG wiring OK. ";
    }
    if (lo < 20) {
        d += "SG_RESULT is near 0 even moving freely: the speed is too low for stallGuard, or the motor is "
             "blocked. Try a higher SPEED (and the same as homing_speed). ";
        return d;
    }
    if (m.diag_hi[0] > 0)
        d += "With your driver_SGTHRS it already triggers while moving freely: homing would stop early. ";
    // from the low end of the free-running readings (10th percentile: one odd reading does not decide)
    int sug = std::max(1, std::min(255, int(p10 * 0.3)));
    d += "Suggested start: driver_SGTHRS: " + std::to_string(sug) + " (triggers at " + std::to_string(2 * sug) +
         ", well below the free-running readings) with homing_speed: " + fmt("%.0f", m.sg_speed) +
         ". Then home: stops early -> lower it by 10; grinds without stopping -> raise it by 10.";
    ok = wired;
    return d;
}

// ---- live DIAG reading, stallGuard threshold and rotation_distance from the Test bench

void Machine::poll_diag(double now) {
    if (now < next_diag_poll_ || printing() || !io_.query) return;
    next_diag_poll_ = now + 0.25;
    for (size_t i = 0; i < plan_.steppers.size(); i++) {
        const StepperObj& s = plan_.steppers[i];
        if (s.endstop < 0 || plan_.endstops[size_t(s.endstop)].tmc < 0) continue;
        const EndstopObj& e = plan_.endstops[size_t(s.endstop)];
        Message m;
        if (!io_.query("endstop_query_state oid=" + std::to_string(e.oid), "endstop_state", e.oid, m, 100)) {
            sr_[i].diag = -1;
            continue;
        }
        sr_[i].diag = ((m.get("pin_value") != 0) != e.invert) ? 1 : 0;
        if (sr_[i].diag == 1) sr_[i].diag_high_at = now;
    }
}

std::string Machine::set_stallguard(size_t i, int sgthrs, bool save) {
    const StepperObj& s = plan_.steppers[i];
    if (s.endstop < 0 || plan_.endstops[size_t(s.endstop)].tmc < 0)
        return "error: " + s.name + " is not set up for sensorless homing";
    TmcObj& t = plan_.tmcs[size_t(plan_.endstops[size_t(s.endstop)].tmc)];
    if (save) {
        if (!io_.config_text || !io_.save_config) return "error: saving printer.cfg is not available here";
        if (printing()) return "error: a print is running";
        std::string text = config_set_option(io_.config_text(), "tmc2209 " + s.name, "driver_SGTHRS", std::to_string(sgthrs));
        std::string err;
        if (!io_.save_config(text, &err)) return "error: could not save printer.cfg: " + err;
        return "ok saved driver_SGTHRS: " + std::to_string(sgthrs) + " for " + s.name + " in printer.cfg; the board restarts with it";
    }
    std::string err;
    if (!tmc_write(t, 0x40, uint32_t(sgthrs), &err)) return "error: the TMC2209 did not take it: " + err;
    t.sgthrs = sgthrs;
    return "ok " + s.name + " driver_SGTHRS " + std::to_string(sgthrs) + " (triggers at SG_RESULT <= " +
           std::to_string(2 * sgthrs) + "), in use until the board restarts: Save keeps it";
}

std::string Machine::set_rotation_distance(size_t i, double rd, double now) {
    const StepperObj& s = plan_.steppers[i];
    if (!io_.config_text || !io_.save_config) return "error: saving printer.cfg is not available here";
    if (printing()) return "error: a print is running";
    if (sr_[i].busy || (th_ && !th_->idle(now))) return "error: motors are moving";
    std::string text = config_set_option(io_.config_text(), s.name, "rotation_distance", fmt("%.4f", rd));
    std::string err;
    if (!io_.save_config(text, &err)) return "error: could not save printer.cfg: " + err;
    double spm = s.full_steps * s.microsteps / rd;
    return "ok " + s.name + " rotation_distance " + fmt("%.4f", rd) + " (" + fmt("%.3f", spm) +
           " steps/mm) saved in printer.cfg; the board restarts with it (axes need homing again)";
}

void Machine::run_tests(double now) {
    // heaters
    for (size_t i = 0; i < heat_tests_.size(); i++) {
        Test& t = test("heat_" + plan_.heaters[i].name);
        if (t.state != "running") continue;
        HeatTest& h = heat_tests_[i];
        HeaterRun& r = hr_[i];
        if (r.target != h.target) {
            finish(t, "aborted", "stopped: the target was changed (" + fmt("%.0f C", r.target) + ")");
            continue;
        }
        double temp = r.smoothed;
        if (r.last_pwm > h.max_power) h.max_power = r.last_pwm;
        if (h.reached_at < 0) {
            if (temp >= h.target - 1.0) {
                h.reached_at = now;
            } else if (now - t.started > h.timeout) {
                set_target(i, 0);
                finish(t, "failed", "only reached " + fmt("%.1f C", temp) + " in " + fmt("%.0f s", now - t.started) +
                                        " (target " + fmt("%.0f", h.target) + "); heater off");
                continue;
            }
            t.detail = "heating " + fmt("%.1f", temp) + " -> " + fmt("%.0f C", h.target) + ", " +
                       fmt("%.0f s", now - t.started) + ", power " + fmt("%.0f %%", r.last_pwm * 100);
            continue;
        }
        if (temp < h.lo) h.lo = temp;
        if (temp > h.hi) h.hi = temp;
        double held = now - h.reached_at;
        if (held >= h.hold) {
            set_target(i, 0);
            // pass only if it stayed within 5 C of the target (Klipper's verify_heater hysteresis)
            bool held_ok = h.hi <= h.target + 5.0 && h.lo >= h.target - 5.0;
            finish(t, held_ok ? "passed" : "failed",
                   std::string(held_ok ? "" : "NOT within 5 C of the target: ") + "reached " + fmt("%.0f C", h.target) +
                       " in " + fmt("%.0f s", h.reached_at - t.started) + ", held " + fmt("%.1f", h.lo) + "-" +
                       fmt("%.1f C", h.hi) + " for " + fmt("%.0f s", h.hold) + " (max power " +
                       fmt("%.0f %%", h.max_power * 100) + "). Heater now off." +
                       (h.hi > h.target + 3 ? " It overshot by " + fmt("%.1f C", h.hi - h.target) +
                                                  ": run the PID tune (and if the heater is over-powered for its supply, "
                                                  "set max_power in printer.cfg)." : ""));
            continue;
        }
        t.detail = "holding " + fmt("%.1f C", temp) + " (" + fmt("%.0f", held) + " of " + fmt("%.0f s", h.hold) + ", range " +
                   fmt("%.1f", h.lo) + "-" + fmt("%.1f", h.hi) + ", power " + fmt("%.0f %%", r.last_pwm * 100) + ")";
    }
    // motors
    for (size_t i = 0; i < motor_tests_.size(); i++) {
        MotorTest& m = motor_tests_[i];
        Test& t = test((m.sg ? "sg_" : "motor_") + plan_.steppers[i].name);
        if (t.state != "running") continue;
        const StepperObj& s = plan_.steppers[i];
        if (m.sg && now < m.end + 0.3) {
            sg_sample(i, now);
            for (auto& p : m.phases)
                if (now >= p.first) t.detail = p.second;
            t.detail += ", " + fmt("%.0f s left", m.end - now > 0 ? m.end - now : 0) + "; SG_RESULT samples " +
                        std::to_string(m.sg_vals[0].size() + m.sg_vals[1].size());
            continue;
        }
        if (m.sg) {
            int64_t pos = 0;
            bool have = m.have_start_pos && stepper_position(i, pos);
            sg_registers(i, false, plan_.tmcs[size_t(m.sg_tmc)].sgthrs);
            stepper_enable(i, false, now);
            sr_[i].busy = false;
            if (th_) th_->stepper_used_elsewhere(i);
            bool ok = false;
            std::string d = sg_report(i, ok);
            if (have && pos != m.start_pos) d += " Step counter " + std::to_string(pos - m.start_pos) + " steps off.";
            finish(t, ok ? "passed" : "failed", d + " Driver off.");
            m.sg = false;
            continue;
        }
        if (!m.checked_drv && s.tmc >= 0 && now >= m.cruise_check && !m.buzz_only) {
            m.checked_drv = true;
            uint32_t drv = 0;
            std::string line = tmc_status_line(i, &drv);
            if (line.find("DRIVER REPORTS") != std::string::npos || line.find("GSTAT") != std::string::npos)
                m.drv_note = line.substr(line.find(';') + 2);
            else if (drv & ((1u << 6) | (1u << 7)))
                m.drv_note = "open load flagged (ola/olb): motor maybe not connected (only a hint in stealthChop)";
        }
        if (now < m.end + 0.3) {
            std::string ph = "starting";
            for (auto& p : m.phases)
                if (now >= p.first) ph = p.second;
            t.detail = ph + ", " + fmt("%.0f s left", m.end - now > 0 ? m.end - now : 0);
            continue;
        }
        int64_t pos = 0;
        bool have = m.have_start_pos && stepper_position(i, pos);
        stepper_enable(i, false, now);
        sr_[i].busy = false;
        std::string d = m.buzz_only ? "moved forward and back 10 times" : "ran CW, CCW and shake";
        if (have && pos == m.start_pos)
            d += "; the board's step counter is back where it started";
        else if (have)
            d += "; step counter is " + std::to_string(pos - m.start_pos) + " steps off (expected 0)";
        if (!m.drv_note.empty()) d += "; " + m.drv_note;
        d += ". Driver off. Steps were sent: whether the motor turned, you can see.";
        finish(t, have && pos != m.start_pos ? "failed" : "passed", d);
    }
    // fans
    for (size_t i = 0; i < fan_tests_.size(); i++) {
        Test& t = test("fan_" + plan_.fans[i].name);
        if (t.state != "running") continue;
        double e = now - fan_tests_[i].t0;
        if (e < 3) continue;
        const std::string pin = plan_.fans[i].out.pin;
        if (e < 6) {
            if (fr_[i].requested != 0.5) fan_set(i, 0.5, now);
            t.detail = "50 % on " + pin;
            continue;
        }
        fan_set(i, 0, now);
        fr_[i].override_active = false;
        finish(t, "passed", "ran " + pin + " at 100 % (steady on) and 50 % (no tachometer: check by eye that it spun, and the "
                            "SKR Pico's fan LED next to that port lit). If the LED lit but the fan did not turn: fan wiring "
                            "or voltage. If the LED did not light: this is not the port the fan is on - try the other fan "
                            "tests. Now off.");
    }
    // LEDs
    if (!plan_.neopixels.empty() && test("led").state == "running") {
        int step = int((now - led_test_.t0) / 1.0);
        if (step != led_test_.step) {
            led_test_.step = step;
            static const char* names[] = {"red", "green", "blue", "white"};
            if (step >= 4) {
                for (auto& n : plan_.neopixels) io_.neopixel_write(n.oid, n.data, nullptr);
                finish(test("led"), "passed", "showed red, green, blue, white; back to the initial colour");
            } else {
                for (auto& n : plan_.neopixels) {
                    std::vector<uint8_t> d(n.data.size(), 0);
                    for (size_t k = 0; k < d.size(); k++) {
                        char c = n.order[k % n.order.size()];
                        bool on = (step == 0 && c == 'R') || (step == 1 && c == 'G') || (step == 2 && c == 'B') ||
                                  (step == 3 && (n.order.find('W') != std::string::npos ? c == 'W' : true));
                        d[k] = on ? 128 : 0;
                    }
                    io_.neopixel_write(n.oid, d, nullptr);
                }
                test("led").detail = names[step];
            }
        }
    }
    // run all: one test at a time (the hotend heats alongside)
    if (all_running_) {
        Test& a = test("all");
        bool busy = false;
        for (auto& t : tests_)
            if (t.id != "all" && t.id.rfind("heat_", 0) != 0 && t.state == "running") busy = true;
        if (!busy && !all_queue_.empty()) {
            std::string id = all_queue_.front();
            all_queue_.erase(all_queue_.begin());
            std::string r;
            if (id == "led") {
                r = start_led_test(now);
            } else if (id.rfind("fan_", 0) == 0) {
                for (size_t i = 0; i < plan_.fans.size(); i++)
                    if (id == "fan_" + plan_.fans[i].name) r = start_fan_test(i, now);
            } else if (id.rfind("motor_", 0) == 0) {
                for (size_t i = 0; i < plan_.steppers.size(); i++)
                    if (id == "motor_" + plan_.steppers[i].name) r = start_motor_test(i, 20, 20, 0, now);
            }
            if (r.rfind("error", 0) == 0) finish(test(id), "failed", r);
            a.detail = "running " + id;
        } else if (!busy && all_queue_.empty()) {
            bool heat_running = false;
            for (auto& t : tests_)
                if (t.id.rfind("heat_", 0) == 0 && t.state == "running") heat_running = true, a.detail = "waiting for " + t.id;
            if (!heat_running) {
                all_running_ = false;
                std::string failed;
                for (auto& t : tests_)
                    if (t.id != "all" && t.started >= a.started && t.state != "passed" && t.state != "idle")
                        failed += t.id + " (" + t.state + ") ";
                finish(a, failed.empty() ? "passed" : "failed", failed.empty() ? "every test passed" : "not passed: " + failed);
            }
        }
    }
}

// ---------------------------------------------------------------- console

static std::string parse_error(const std::string& line) {
    std::string l = line.substr(0, line.find_first_of(";#")), w, bad;
    bool first = true;
    for (size_t i = 0; i <= l.size() && bad.empty(); i++) {
        if (i < l.size() && !isspace((unsigned char)l[i])) {
            w += l[i];
            continue;
        }
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

static const char* HELP_TEXT =
    "Heaters: M104 S<temp> / M109 S<temp> (hotend), M140 S / M190 S (bed), "
    "SET_HEATER_TEMPERATURE HEATER=<extruder|heater_bed> TARGET=<temp>, TURN_OFF_HEATERS, M105. "
    "Fans: M106 S<0-255>, M107. Motors: M18/M84 (off), STEPPER_BUZZ STEPPER=<name>. "
    "LED: SET_LED LED=<name> RED= GREEN= BLUE= [WHITE=] [INDEX=]. "
    "Tests: TEST_HEATER HEATER=<name> [TARGET=] [HOLD=], STEPPER_TEST STEPPER=<name> [RPM=20] [SECONDS=20], "
    "FAN_TEST FAN=<name>, LED_TEST, TMC_CHECK, DUMP_TMC STEPPER=<name>, TEST_ALL, TEST_ABORT [TEST=<id>]. "
    "PID: PID_CALIBRATE HEATER=<name> TARGET=<temp>, then SAVE_CONFIG. HEATER_OUTPUT_TEST HEATER=<name> (MOSFET check). HEATER_LOG (last 5 minutes, CSV). "
    "Motors: STEPPER_SHAKE STEPPER=<name>. Sensorless: STALLGUARD_TEST STEPPER=<name> [SPEED=homing_speed] [DIST=10] "
    "(SG_RESULT readings, DIAG wiring check, a suggested driver_SGTHRS), SET_STALLGUARD STEPPER= SGTHRS= [SAVE=1], "
    "SET_ROTATION_DISTANCE STEPPER= VALUE= (saves printer.cfg), QUERY_ENDSTOPS. "
    "Moves: G28, G0/G1, PROBE, JOG, M114. Safety: M112 (emergency stop), FIRMWARE_RESTART.";

std::string Machine::command(const std::string& raw, double now) {
    std::string line = raw;
    while (!line.empty() && isspace((unsigned char)line.back())) line.pop_back();
    size_t b = line.find_first_not_of(" \t");
    if (b == std::string::npos) return "error: empty command. Try: HELP";
    line = line.substr(b);
    std::string name;
    std::map<std::string, std::string> p;
    bool classic = (toupper((unsigned char)line[0]) == 'G' || toupper((unsigned char)line[0]) == 'M' ||
                    toupper((unsigned char)line[0]) == 'T') &&
                   line.size() > 1 && isdigit((unsigned char)line[1]);
    GLine g;
    if (classic) {
        if (!gcode_parse(line, g)) return "error: could not read '" + line + "'";
        name = std::string(1, g.cmd_letter) + std::to_string(g.cmd_num);
    } else {
        if (!gcode_parse_ext(line, name, p)) return parse_error(line);
    }
    // Always allowed
    if (name == "HELP") return HELP_TEXT;
    if (name == "M112" || name == "STOP" || name == "EMERGENCY_STOP") {
        shutdown("emergency stop (M112)", now);
        return "ok EMERGENCY STOP: everything is off. Send FIRMWARE_RESTART (Restart button) to continue.";
    }
    if (name == "FIRMWARE_RESTART" || name == "RESTART") {
        if (!shutdown_) all_off(now);
        restart_ = true;
        return "ok restarting the board and reloading printer.cfg";
    }
    if (name == "M105") {
        if (plan_.sensors.empty()) return "error: no temperature sensors in printer.cfg";
        std::string r = "ok";
        for (size_t i = 0; i < plan_.sensors.size(); i++) {
            auto& s = plan_.sensors[i];
            r += (i ? ", " : " ") + s.name + ": " + s.describe();
            if (s.heater >= 0 && hr_[size_t(s.heater)].target > 0) r += " -> " + fmt("%.0f", hr_[size_t(s.heater)].target);
        }
        return r;
    }
    if (name == "HEATER_LOG") return heater_log_csv();
    if (name == "TEST_ABORT") {
        auto it = p.find("TEST");
        std::string id = it == p.end() ? "all" : it->second;
        if (id == "all" || id == "ALL") {
            for (auto& t : tests_)
                if (t.state == "running") abort_test(t.id, now);
            for (size_t i = 0; i < sr_.size(); i++)
                if (sr_[i].busy) abort_test("motor_" + plan_.steppers[i].name, now);
            return "ok every running test stopped";
        }
        bool known = false;
        for (auto& t : tests_) known |= t.id == id;
        if (!known) return "error: no test '" + id + "'";
        abort_test(id, now);
        return "ok " + id + " stopped";
    }
    if (name == "CANCEL_PRINT") {
        if (!printing()) return "error: no print is running";
        print_end("cancelled", "cancelled from the console", now);
        return "ok print cancelled: heaters and fan off (moves already planned finish first)";
    }
    if (name == "PRINT_STATUS")
        return "ok " + job_.state + (job_.file.empty() ? "" : " " + job_.file) + (job_.msg.empty() ? "" : ": " + job_.msg);
    if (shutdown_)
        return "error: the board is shut down (" + shutdown_reason_ + "). Send FIRMWARE_RESTART (Restart button).";
    last_activity_ = now;
    if (name == "SDCARD_PRINT_FILE" || name == "PRINT_FILE" || (name == "M24" && job_.state != "paused")) return print_start(now);
    if (name == "PAUSE" || name == "M25") {
        if (job_.state != "printing") return "error: no print is running";
        job_.state = "paused";
        job_.msg = "paused (moves already planned finish first)";
        if (th_) th_->flush_lookahead();
        return "ok paused";
    }
    if (name == "RESUME" || name == "M24") {
        if (job_.state != "paused") return "error: the print is not paused";
        job_.state = "printing";
        job_.msg.clear();
        return "ok resumed";
    }
    std::string jerr;
    if (name == "JOG") {   // JOG AXIS=X DIST=10 [SPEED=mm/s]: the jog buttons
        if (!th_) return "error: no steppers in printer.cfg";
        if (printing()) return "error: a print is running (PAUSE or CANCEL_PRINT first)";
        auto it = p.find("AXIS");
        std::string ax = it == p.end() ? "" : it->second;
        int a = ax == "X" || ax == "x" ? 0 : ax == "Y" || ax == "y" ? 1 : ax == "Z" || ax == "z" ? 2 : ax == "E" || ax == "e" ? 3 : -1;
        if (a < 0) return "error: JOG needs AXIS=X|Y|Z|E";
        for (size_t i = 0; i < sr_.size(); i++)
            if (sr_[i].busy) return "error: a motor test is running on " + plan_.steppers[i].name + " (wait, or TEST_ABORT)";
        double d = 0, sp = a == 2 ? 5 : a == 3 ? 3 : 40;
        if (!num(p, "DIST", d, -1000, 1000, jerr) || !num(p, "SPEED", sp, 0.1, 500, jerr)) return "error: " + jerr;
        return th_->jog(a, d, sp, now);
    }
    if (th_) {
        static const char* RUNNING_OK[] = {"M114", "GET_POSITION", "QUERY_ENDSTOPS", "M400"};
        bool ro = false;
        for (auto* n : RUNNING_OK) ro |= name == n;
        std::string tr;
        if (printing() && !ro) {
            static const char* MOTION[] = {"G0", "G1", "G4", "G28", "G90", "G91", "G92", "M82", "M83", "M204", "M220",
                                           "M221", "PROBE", "SET_KINEMATIC_POSITION", "M84", "M18"};
            for (auto* n : MOTION)
                if (name == n) return "error: a print is running (PAUSE or CANCEL_PRINT first)";
        }
        {   // a motor / stallGuard test drives a stepper directly: moving it too would collide on the board
            static const char* MOVES[] = {"G0", "G1", "G28", "PROBE"};
            bool mv = false;
            for (auto* n : MOVES) mv |= name == n;
            for (size_t i = 0; mv && i < sr_.size(); i++)
                if (sr_[i].busy) return "error: a motor test is running on " + plan_.steppers[i].name + " (wait, or TEST_ABORT)";
        }
        tr = th_->gcode(name, classic ? &g : nullptr, p, true, now);
        if (!tr.empty()) return tr;
    }

    auto heater_idx = [&](const std::string& n) -> int {
        for (size_t i = 0; i < plan_.heaters.size(); i++)
            if (plan_.heaters[i].name == n) return int(i);
        return -1;
    };
    auto set_heater = [&](const std::string& hn, double t) -> std::string {
        int i = heater_idx(hn);
        if (i < 0) return "error: no [" + hn + "] heater in printer.cfg";
        const HeaterObj& o = plan_.heaters[size_t(i)];
        if (t != 0 && (t < o.min_temp || t > o.max_temp))
            return "error: requested temperature (" + fmt("%.1f", t) + ") out of range (" + fmt("%.1f", o.min_temp) + ":" +
                   fmt("%.1f", o.max_temp) + ")";
        const TempSensorObj& s = plan_.sensors[size_t(o.sensor)];
        if (t > 0 && s.state() != 1) return "error: " + hn + " sensor: " + s.describe();
        if (hr_[size_t(i)].out_test) return "error: the " + hn + " output check is running";
        // A new temperature from you wins over a running test of this heater: stop the test, say so,
        // and heat to what you asked for (normal PID control).
        std::string stopped, why = "stopped: you set " + hn + " to " + (t > 0 ? fmt("%.0f C", t) : std::string("off"));
        if (hr_[size_t(i)].tuning) {
            hr_[size_t(i)].tuning = false;
            finish(test("pid_" + hn), "aborted", why);
            stopped = " (the PID tune was stopped: no new PID values)";
        }
        Test& ht = test("heat_" + hn);
        if (ht.state == "running") {
            finish(ht, "aborted", why);
            stopped = " (the heater test was stopped)";
        }
        set_target(size_t(i), t);
        return (t > 0 ? "ok " + hn + " target " + fmt("%.0f", t) : "ok " + hn + " off") + stopped;
    };

    if (classic) {
        if (g.is('M', 104) || g.is('M', 109)) {
            if (g.has('T') && g.get('T') != 0) return "error: only one extruder (T0)";
            std::string r = set_heater("extruder", g.get('S', 0));
            if (g.is('M', 109) && r.rfind("ok", 0) == 0) r += " (this console does not wait: watch the status page)";
            return r;
        }
        if (g.is('M', 140) || g.is('M', 190)) {
            std::string r = set_heater("heater_bed", g.get('S', 0));
            if (g.is('M', 190) && r.rfind("ok", 0) == 0) r += " (this console does not wait: watch the status page)";
            return r;
        }
        if (g.is('M', 106) || g.is('M', 107)) {
            FanObj* f = plan_.fan("fan");
            if (!f) return "error: no [fan] in printer.cfg";
            size_t i = size_t(f - plan_.fans.data());
            double v = g.is('M', 107) ? 0 : g.get('S', 255) / 255.0;
            v = v < 0 ? 0 : v > 1 ? 1 : v;
            fr_[i].override_active = false;
            fan_set(i, v, now);
            return "ok fan " + fmt("%.0f %%", v * 100) + " (" + f->out.pin + ")";
        }
        if (g.is('M', 18) || g.is('M', 84)) {
            if (th_) th_->motors_off(now);
            for (size_t i = 0; i < sr_.size(); i++) {
                if (sr_[i].busy) abort_test("motor_" + plan_.steppers[i].name, now, "motors switched off (M84)");
                stepper_enable(i, false, now);
            }
            return "ok motors off";
        }
        if (g.cmd_letter == 'G' || g.is('M', 82) || g.is('M', 83))
            return "error: " + name + ": moves are not set up (" + (th_ ? th_->why_not() : std::string("no steppers")) + ")";
        return "error: " + name + " is not supported yet (try HELP)";
    }

    std::string err;
    if (name == "SET_HEATER_TEMPERATURE") {
        auto it = p.find("HEATER");
        if (it == p.end()) return "error: SET_HEATER_TEMPERATURE needs HEATER=<extruder|heater_bed>";
        double t = 0;
        if (!num(p, "TARGET", t, 0, 1000, err)) return "error: " + err;
        return set_heater(it->second, t);
    }
    if (name == "TURN_OFF_HEATERS") {
        for (size_t i = 0; i < hr_.size(); i++) set_target(i, 0);
        return "ok heaters off";
    }
    if (name == "SET_LED") {
        std::string msg;
        NeopixelObj* n = apply_set_led(plan_, p, msg);
        if (!n) return "error: " + msg;
        if (!io_.neopixel_write(n->oid, n->data, &err)) return "error: " + err;
        return "ok";
    }
    if (name == "TEST_HEATER") {
        auto it = p.find("HEATER");
        std::string hn = it == p.end() ? "extruder" : it->second;
        int i = heater_idx(hn);
        if (i < 0) return "error: no [" + hn + "] heater in printer.cfg";
        double t = hn == "heater_bed" ? 50 : 75, hold = 60;
        if (!num(p, "TARGET", t, 0, 1000, err) || !num(p, "HOLD", hold, 0, 3600, err)) return "error: " + err;
        return start_heat_test(size_t(i), t, hold, now);
    }
    if (name == "SET_STALLGUARD") {
        auto it = p.find("STEPPER");
        if (it == p.end()) return "error: SET_STALLGUARD needs STEPPER=<name> SGTHRS=<0-255> [SAVE=1]";
        StepperObj* s = plan_.stepper(it->second);
        if (!s) return "error: no stepper '" + it->second + "' in printer.cfg";
        double v = -1, sv = 0;
        if (!num(p, "SGTHRS", v, 0, 255, err) || v < 0) return "error: SGTHRS must be 0 to 255" + (err.empty() ? "" : ": " + err);
        if (!num(p, "SAVE", sv, 0, 1, err)) return "error: " + err;
        return set_stallguard(size_t(s - plan_.steppers.data()), int(v + 0.5), sv > 0);
    }
    if (name == "SET_ROTATION_DISTANCE") {
        auto it = p.find("STEPPER");
        if (it == p.end()) return "error: SET_ROTATION_DISTANCE needs STEPPER=<name> VALUE=<mm>";
        StepperObj* s = plan_.stepper(it->second);
        if (!s) return "error: no stepper '" + it->second + "' in printer.cfg";
        double v = 0;
        if (!num(p, "VALUE", v, 0.1, 1000, err) || v <= 0) return "error: VALUE must be 0.1 to 1000 mm" + (err.empty() ? "" : ": " + err);
        return set_rotation_distance(size_t(s - plan_.steppers.data()), v, now);
    }
    if (name == "STALLGUARD_TEST") {
        auto it = p.find("STEPPER");
        if (it == p.end()) return "error: STALLGUARD_TEST needs STEPPER=<name>";
        StepperObj* s = plan_.stepper(it->second);
        if (!s) return "error: no stepper '" + it->second + "' in printer.cfg";
        double speed = s->homing_speed, dist = 10;
        if (!num(p, "SPEED", speed, 1, 200, err) || !num(p, "DIST", dist, 2, 100, err)) return "error: " + err;
        return start_motor_test(size_t(s - plan_.steppers.data()), 0, 0, 3, now, speed, dist);
    }
    if (name == "STEPPER_TEST" || name == "STEPPER_BUZZ" || name == "STEPPER_SHAKE") {
        auto it = p.find("STEPPER");
        if (it == p.end()) return "error: " + name + " needs STEPPER=<name>";
        StepperObj* s = plan_.stepper(it->second);
        if (!s) return "error: no stepper '" + it->second + "' in printer.cfg";
        double rpm = 20, secs = 20;
        if (!num(p, "RPM", rpm, 1, 600, err) || !num(p, "SECONDS", secs, 1, 120, err)) return "error: " + err;
        return start_motor_test(size_t(s - plan_.steppers.data()), rpm, secs,
                                name == "STEPPER_BUZZ" ? 1 : name == "STEPPER_SHAKE" ? 2 : 0, now);
    }
    if (name == "FAN_TEST") {
        auto it = p.find("FAN");
        std::string fn = it == p.end() ? "fan" : it->second;
        FanObj* f = plan_.fan(fn);
        if (!f) return "error: no fan '" + fn + "' in printer.cfg";
        return start_fan_test(size_t(f - plan_.fans.data()), now);
    }
    if (name == "LED_TEST") return start_led_test(now);
    bool heating = false;
    for (auto& h : hr_) heating |= h.target > 0;
    if (name == "TMC_CHECK") {
        if (plan_.tmcs.empty()) return "error: no [tmc2209] sections in printer.cfg";
        if (heating) return "error: switch the heaters off first (the TMC check holds the board link for a few seconds)";
        std::string r = tmc_check_all(now);
        return (r.rfind("TMC check passed", 0) == 0 ? "ok " : "error: ") + r;
    }
    if (name == "DUMP_TMC") {
        if (heating) return "error: switch the heaters off first (reading the driver holds the board link)";
        auto it = p.find("STEPPER");
        if (it == p.end()) return "error: DUMP_TMC needs STEPPER=<name>";
        StepperObj* s = plan_.stepper(it->second);
        if (!s) return "error: no stepper '" + it->second + "'";
        return dump_tmc(size_t(s - plan_.steppers.data()));
    }
    if (name == "TEST_ALL") return start_all(now);
    if (name == "PID_CALIBRATE") {
        auto it = p.find("HEATER");
        if (it == p.end()) return "error: PID_CALIBRATE needs HEATER=<extruder|heater_bed> TARGET=<temp>";
        int i = heater_idx(it->second);
        if (i < 0) return "error: no [" + it->second + "] heater in printer.cfg";
        double t = 0;
        if (p.find("TARGET") == p.end()) return "error: PID_CALIBRATE needs TARGET=<temp>";
        if (!num(p, "TARGET", t, 0, 1000, err)) return "error: " + err;
        return start_pid_tune(size_t(i), t, now);
    }
    if (name == "SAVE_CONFIG") return save_config(now);
    if (name == "HEATER_OUTPUT_TEST") {
        auto it = p.find("HEATER");
        std::string hn = it == p.end() ? "extruder" : it->second;
        int i = heater_idx(hn);
        if (i < 0) return "error: no [" + hn + "] heater in printer.cfg";
        return start_output_test(size_t(i), now);
    }
    return "error: " + name + " is not supported yet (try HELP)";
}

// ---------------------------------------------------------------- printing a file

std::string Machine::print_start(double now) {
    if (printing()) return "error: a print is already running";
    if (!th_ || !th_->ok()) return "error: moves are not set up (" + (th_ ? th_->why_not() : std::string("no steppers")) + ")";
    if (!io_.print_open || !io_.print_read) return "error: printing files is not available here";
    if (!th_->idle(now)) return "error: moves are still running";
    for (auto& t : tests_)
        if (t.state == "running") return "error: a test is running (" + t.id + ")";
    std::string name, err;
    if (!io_.print_open(name, &err)) return "error: " + (err.empty() ? std::string("no file uploaded") : err);
    job_ = Job();
    job_.state = "printing";
    job_.file = name;
    job_.started = now;
    if (io_.log) io_.log("print started: " + name);
    return "ok printing " + name;
}

void Machine::print_end(const std::string& state, const std::string& msg, double now) {
    job_.state = state;
    job_.msg = msg;
    job_.ended = now;
    job_.wait = 0;
    if (state != "done") {
        for (size_t i = 0; i < hr_.size(); i++) set_target(i, 0);
        if (FanObj* f = plan_.fan("fan")) fan_set(size_t(f - plan_.fans.data()), 0, now);
    }
    if (th_) th_->flush_lookahead();
    if (io_.log) io_.log("print " + state + (msg.empty() ? "" : ": " + msg));
}

// One line of the file. Returns "" or an error.
std::string Machine::print_exec(const std::string& raw, double now) {
    std::string line = raw.substr(0, raw.find(';'));
    while (!line.empty() && isspace((unsigned char)line.back())) line.pop_back();
    size_t b = line.find_first_not_of(" \t");
    if (b == std::string::npos) return "";
    line = line.substr(b);
    job_.lines++;
    bool classic = (toupper((unsigned char)line[0]) == 'G' || toupper((unsigned char)line[0]) == 'M' ||
                    toupper((unsigned char)line[0]) == 'T' || toupper((unsigned char)line[0]) == 'N') &&
                   line.size() > 1;
    GLine g;
    std::string name;
    std::map<std::string, std::string> p;
    if (classic && gcode_parse(line, g) && g.cmd_letter) {
        name = std::string(1, g.cmd_letter) + std::to_string(g.cmd_num);
    } else if (!classic && gcode_parse_ext(line, name, p)) {
        classic = false;
    } else {
        job_.ignored++;
        return "";
    }
    auto heater = [&](const char* n) -> int {
        for (size_t i = 0; i < plan_.heaters.size(); i++)
            if (plan_.heaters[i].name == n) return int(i);
        return -1;
    };
    if (classic && (g.is('M', 104) || g.is('M', 109) || g.is('M', 140) || g.is('M', 190))) {
        bool bed = g.is('M', 140) || g.is('M', 190);
        int i = heater(bed ? "heater_bed" : "extruder");
        if (i < 0) {   // e.g. a Tina 2 without a heated bed: bed commands are skipped
            job_.ignored++;
            return "";
        }
        double t = g.has('S') ? double(g.get('S')) : g.has('R') ? double(g.get('R')) : 0.0;
        const HeaterObj& o = plan_.heaters[size_t(i)];
        if (t != 0 && (t < o.min_temp || t > o.max_temp)) return o.name + " temperature " + fmt("%.0f", t) + " out of range";
        set_target(size_t(i), t);
        if ((g.is('M', 109) || g.is('M', 190)) && t > 0) {
            if (th_) th_->flush_lookahead();
            job_.wait = 1, job_.wait_heater = i, job_.wait_target = t;
            job_.msg = "waiting for " + o.name + " to reach " + fmt("%.0f C", t);
        }
        return "";
    }
    if (classic && (g.is('M', 106) || g.is('M', 107))) {
        if (FanObj* f = plan_.fan("fan")) {
            double v = g.is('M', 107) ? 0 : g.get('S', 255) / 255.0;
            fan_set(size_t(f - plan_.fans.data()), v < 0 ? 0 : v > 1 ? 1 : v, now);
        }
        return "";
    }
    if (classic && (g.is('M', 84) || g.is('M', 18))) {
        job_.wait = 3;
        return "";
    }
    if (classic && (g.is('M', 117) || g.is('M', 118))) {
        job_.msg = line.size() > 5 ? line.substr(5) : "";
        return "";
    }
    if (name == "M400") {
        job_.wait = 2;
        return "";
    }
    if (name == "G2" || name == "G3")
        return name + " arcs are not supported yet: turn off arc fitting in the slicer and slice again";
    if (name == "G10" || name == "G11")
        return name + " firmware retraction is not supported yet: use slicer retraction and slice again";
    std::string r = th_->gcode(name, classic ? &g : nullptr, p, false, now);
    if (r.empty()) {   // not a motion command we know: skipped (M73, M115, slicer macros, ...)
        job_.ignored++;
        if (job_.ignored_names.find(" " + name) == std::string::npos && job_.ignored_names.size() < 120)
            job_.ignored_names += " " + name;
        return "";
    }
    if (r.rfind("error", 0) == 0) return r.substr(7) + " (line: " + line + ")";
    if (name == "G28" || name == "PROBE") job_.wait = 2;
    return "";
}

void Machine::print_tick(double now) {
    if (job_.state != "printing" && job_.state != "paused") return;
    if (job_.state == "paused") return;
    if (job_.wait == 1) {
        const HeaterRun& h = hr_[size_t(job_.wait_heater)];
        if (h.target != job_.wait_target) {   // changed from the console: stop waiting
            job_.wait = 0;
        } else if (h.have_temp && h.smoothed >= job_.wait_target - 1.0) {
            job_.wait = 0;
            job_.msg.clear();
        } else {
            return;
        }
    }
    if (job_.wait == 2 || job_.wait == 3) {
        if (!th_->idle(now)) return;
        if (!th_->last_error().empty() && job_.wait == 2) {
            std::string e = th_->last_error();
            print_end("error", e, now);
            return;
        }
        if (job_.wait == 3) {
            th_->motors_off(now);
            for (size_t i = 0; i < sr_.size(); i++)
                if (sr_[i].enabled) stepper_enable(i, false, now);
        }
        job_.wait = 0;
    }
    int n = 0;
    while (job_.wait == 0 && th_->buffered(now) < 1.5 && n < 60) {
        std::string line, err;
        bool eof = false;
        if (!io_.print_read(line, eof, &err)) {
            print_end("error", "could not read the file: " + err, now);
            return;
        }
        if (eof) {   // end of file: finish the moves, then it is done
            th_->flush_lookahead();
            job_.wait = 4;
            job_.msg = "finishing the last moves";
            break;
        }
        std::string e = print_exec(line, now);
        if (!e.empty()) {
            print_end("error", e, now);
            return;
        }
        n++;
    }
    if (job_.wait == 4 && th_->idle(now))
        print_end("done", "finished: " + std::to_string(job_.lines) + " lines" + (job_.ignored ? ", skipped:" + job_.ignored_names : ""), now);
}

// ---------------------------------------------------------------- status

std::string Machine::status_json(double now) const {
    bool unsaved = false;
    for (auto& h : hr_) unsaved |= h.unsaved;
    std::string j = "{\"shutdown\":" + std::string(shutdown_ ? "true" : "false") + ",\"reason\":" + json_str(shutdown_reason_) +
                    ",\"unsaved_pid\":" + (unsaved ? "true" : "false") + ",\"power_off_now\":" + (power_off_now_ ? "true" : "false") + ",\"clock_err_ms\":" + fmt("%.1f", clock_err_ms_);
    j += ",\"sensors\":[";
    for (size_t i = 0; i < plan_.sensors.size(); i++) {
        const auto& s = plan_.sensors[i];
        double target = 0, power = 0, temp = s.temp_c;
        if (s.heater >= 0) {
            const HeaterRun& h = hr_[size_t(s.heater)];
            target = h.target, power = h.last_pwm;
            if (h.have_temp) temp = h.smoothed;
        }
        std::string state;
        if (s.state() != 1)
            state = "fault";
        else if (target > 0)
            state = temp < target - 2 ? "heating" : "holding";
        else if (temp > 40 && sensor_rate_[i] < -0.02)
            state = "cooling";
        else if (temp > 40)
            state = "hot";
        else
            state = "cold";
        char b[200];
        snprintf(b, sizeof b, "%s{\"name\":%s,\"temp\":%.1f,\"target\":%.0f,\"power\":%.2f,\"rate\":%.2f,\"heater\":%s,\"state\":\"%s\",\"text\":%s}",
                 i ? "," : "", json_str(s.name).c_str(), temp, target, power, sensor_rate_[i],
                 s.heater >= 0 ? "true" : "false", state.c_str(), json_str(s.describe()).c_str());
        j += b;
    }
    j += "],\"fans\":[";
    for (size_t i = 0; i < plan_.fans.size(); i++) {
        char b[160];
        snprintf(b, sizeof b, "%s{\"name\":%s,\"speed\":%.2f,\"auto\":%s}", i ? "," : "", json_str(plan_.fans[i].name).c_str(),
                 fr_[i].value, plan_.fans[i].heater_fan ? "true" : "false");
        j += b;
    }
    j += "],\"steppers\":[";
    for (size_t i = 0; i < plan_.steppers.size(); i++) {
        j += std::string(i ? "," : "") + "{\"name\":" + json_str(plan_.steppers[i].name) + ",\"enabled\":" +
             (sr_[i].enabled ? "true" : "false") + ",\"busy\":" + (sr_[i].busy ? "true" : "false") +
             ",\"tmc\":" + json_str(plan_.steppers[i].tmc >= 0 ? sr_[i].tmc_status : "no TMC section") +
             ",\"tmc_ok\":" + (sr_[i].tmc_ok ? "true" : "false");
        const StepperObj& st = plan_.steppers[i];
        j += ",\"rd\":" + fmt("%.4f", st.rotation_distance) + ",\"spm\":" + fmt("%.3f", st.steps_per_mm());
        if (st.endstop >= 0 && plan_.endstops[size_t(st.endstop)].tmc >= 0) {
            j += ",\"sgthrs\":" + std::to_string(plan_.tmcs[size_t(plan_.endstops[size_t(st.endstop)].tmc)].sgthrs) +
                 ",\"diag\":" + std::to_string(sr_[i].diag) + ",\"diag_recent\":" +
                 (sr_[i].diag_high_at >= 0 && now - sr_[i].diag_high_at < 3.0 ? "true" : "false");
        }
        j += "}";
    }
    j += "],\"tests\":[";
    for (size_t i = 0; i < tests_.size(); i++) {
        const Test& t = tests_[i];
        double el = t.state == "running" ? now - t.started : 0;
        char b[32];
        snprintf(b, sizeof b, "%.0f", el);
        j += std::string(i ? "," : "") + "{\"id\":" + json_str(t.id) + ",\"label\":" + json_str(t.label) +
             ",\"state\":" + json_str(t.state) + ",\"detail\":" + json_str(t.detail) + ",\"elapsed\":" + b + "}";
    }
    j += "],\"toolhead\":" + (th_ ? th_->status_json(now) : std::string("null"));
    double prog = (printing() || job_.state == "done") && io_.print_progress ? io_.print_progress() : 0;
    char pb[96];
    snprintf(pb, sizeof pb, ",\"progress\":%.3f,\"elapsed\":%.0f,\"lines\":%ld}", prog,
             job_.started > 0 ? (printing() ? now : job_.ended) - job_.started : 0.0, job_.lines);
    j += ",\"print\":{\"state\":" + json_str(job_.state) + ",\"file\":" + json_str(job_.file) + ",\"msg\":" +
         json_str(job_.msg) + pb;
    return j + "}";
}

}  // namespace ph
