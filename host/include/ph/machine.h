// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The printer at run time, on top of a configured board (BoardPlan): heater control with Klipper's
// safety checks, fans, TMC2209 drivers, the test bench (one test per subsystem, each with an abort)
// and the G-code console. Portable: all board traffic goes through MachineIO, so the same code runs
// on the ESP32-S3 and on a PC against fake_mcu.py.
//
// Heater safety, all from Klipper (klippy/extras/heaters.py, verify_heater.py, mcu.py):
//   * the board itself turns a heater off if the host stops refreshing it for 5 s (max_duration on
//     config_digital_out; Klipper MAX_HEAT_TIME), so a crashed or unplugged ESP32 cannot leave it on;
//   * the board itself shuts down if the thermistor reads outside min_temp..max_temp (ADC range check
//     on query_analog_in), e.g. an unplugged or shorted thermistor;
//   * verify_heater: "not heating at expected rate" and "can not hold the target" shut everything down;
//   * M112 / STOP: Klipper's emergency_stop, everything off at once.
#pragma once
#include "ph/boardcfg.h"
#include "ph/msgproto.h"
#include "ph/toolhead.h"

#include <memory>

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

namespace ph {

struct MachineIO {
    // Send commands to the board (waits for the acknowledgement). Several at once are packed.
    std::function<bool(const std::vector<std::string>&, std::string*)> send;
    // Send one command and wait for the response `resp` with this oid.
    std::function<bool(const std::string& cmd, const std::string& resp, int oid, Message& out, int timeout_ms)> query;
    // Board clock (64-bit) expected at a host time in seconds.
    std::function<uint64_t(double)> clock_at;
    // Write and latch NeoPixel bytes (KBoard::neopixel_write).
    std::function<bool(int oid, const std::vector<uint8_t>&, std::string*)> neopixel_write;
    std::function<void(const std::string&)> log;
    // Optional: the host time now (seconds), so commands scheduled after a blocking step (a TMC set-up,
    // a position query) are never late. Without it the time passed in is used.
    std::function<double()> now;
    // Optional: biggest free heap block. The motion planner is not built if memory is short.
    std::function<size_t()> largest_free;
    // Optional: the stored printer.cfg text, and saving a new one (SAVE_CONFIG after PID_CALIBRATE).
    std::function<std::string()> config_text;
    std::function<bool(const std::string&, std::string*)> save_config;
    // Optional: the uploaded G-code file. open() starts reading it from the beginning (fills its name),
    // read() gives the next line (eof at the end), progress() 0..1.
    // name: in = the file asked for ("" = the default / only stored file), out = its display name
    std::function<bool(std::string& name, std::string* err)> print_open;
    // Optional: the G-code files that can be printed (name, bytes), e.g. on an SD card (M20).
    std::function<bool(std::vector<std::pair<std::string, uint64_t>>& files, std::string* err)> list_files;
    std::function<bool(std::string& line, bool& eof, std::string* err)> print_read;
    std::function<double()> print_progress;
    // Optional: most moves of look-ahead wanted (see ToolheadIO::lookahead; 0 = 32).
    int lookahead = 0;
    // Optional: called once when a print ends (state: done, cancelled, error), e.g. for a history.
    std::function<void(const std::string& file, const std::string& state, double seconds, long lines,
                       const std::string& msg)> print_ended;
    // Optional (CAPTAIN / SKIPPER firmware): the accelerometer for the resonance test. mode 1: start
    // recording (axis 0 X, 1 Y); 0: stop and work out the shaper; -1: stop and drop it.
    std::function<bool(int mode, int axis, std::string* err)> accel_record;
    // Optional (2026-10-27): an ADC reading on an ESP32-S3 pin as a fraction of 3.3 V, for
    // [temperature_sensor] with sensor_pin: esp32:gpioN (a chamber thermistor). false = not readable.
    std::function<bool(int gpio, double& ratio)> host_adc;
    // Optional (2026-10-27): performance counters (see ToolheadIO::perf_now), the log of important
    // events (one line each: benchmark results...), and what the board is, for the benchmark report.
    std::function<uint64_t()> perf_now;
    double perf_hz = 0;
    std::function<void(const std::string&)> report;
    std::string board_kind = "printer board, drivers off";
    bool fake_board = false;   // FakeBoard (printbench.h): nothing is connected
    std::string build_id;      // which firmware exactly (version, build, maths, date, binary fingerprint)
};

// TMC UART datagrams (Klipper tmc_uart.py): CRC8-ATM, start/stop bits, read decoding.
uint8_t tmc_crc8(const std::vector<uint8_t>& data);
std::vector<uint8_t> tmc_add_serial_bits(const std::vector<uint8_t>& data);
std::vector<uint8_t> tmc_encode_read(uint8_t sync, uint8_t addr, uint8_t reg);
std::vector<uint8_t> tmc_encode_write(uint8_t sync, uint8_t addr, uint8_t reg, uint32_t val);
bool tmc_decode_read(uint8_t reg, const std::vector<uint8_t>& data, uint32_t& val);

class Machine {
public:
    Machine(BoardPlan& plan, const MsgProto& mp, MachineIO io);

    // After the board is configured: first PWM values (all off), ADC reports, TMC2209 setup.
    // Returns a one-line summary for the log.
    std::string start(double now);
    // Every message from the board (call from the KLink message handler). Never sends anything.
    void on_message(const Message& m, double now);
    // Call often (every 20-100 ms): sends heater updates, runs safety checks, tests, fans.
    void tick(double now);
    // One console line (Klipper G-code subset + test commands). Returns the reply.
    std::string command(const std::string& line, double now);
    // Everything off before the board is configured again (printer.cfg saved, restart).
    void all_off(double now);
    // JSON object for the web pages: heaters, fans, steppers, tests, shutdown.
    std::string status_json(double now) const;
    // Klipper's status objects, as Moonraker reports them (SkiPiPir 2026-10-30, for Pasta Inspector):
    // print_stats, toolhead, motion_report, pause_resume, idle_timeout, virtual_sdcard,
    // display_status, extruder, and exclude_object without its object list (excluded_objects,
    // current_object). The object list (with outlines) separately, and its revision.
    std::string moonraker_status_json(double now) const;
    std::string moonraker_objects_json() const;
    unsigned objects_rev() const { return obj_.rev; }

    // The heaters' last 5 minutes (one line a second) as CSV, for diagnosing heating problems.
    std::string heater_log_csv() const;
    bool is_shutdown() const { return shutdown_; }
    const std::string& shutdown_reason() const { return shutdown_reason_; }
    // FIRMWARE_RESTART was asked for: the caller resets the board and configures it again.
    bool restart_requested() const { return restart_; }
    // The board said it is shut down (get_config is_shutdown) without a shutdown message.
    void note_board_shutdown(const std::string& why);
    Toolhead* toolhead() { return th_ ? th_.get() : nullptr; }
    bool printing() const { return job_.state == "printing" || job_.state == "paused"; }
    // SF_BENCH (2026-10-27): a G-code file run through the whole motion pipeline with the stepper
    // drivers and heaters off, timed stage by stage. bench_report(): the last result ("" if none).
    bool bench_running() const { return bench_; }
    const std::string& bench_report() const { return bench_report_; }

    // Test bench ids: "heat_extruder", "heat_heater_bed", "motor_<stepper>", "fan_<fan>", "led",
    // "tmc", "all". Exposed for tests.
    struct Test {
        Test() = default;
        Test(std::string i, std::string l) : id(std::move(i)), label(std::move(l)) {}
        std::string id, label;
        std::string state = "idle";   // idle, running, passed, failed, aborted
        std::string detail;
        double started = 0;
    };
    const std::vector<Test>& tests() const { return tests_; }

    // What the little OLED on the ESP32 shows (2026-10-23): the hotend and bed, the print state, and
    // what the printer is doing right now. state: standby, printing, paused, complete, shutdown.
    // activity: HEATING (a print or macro waits for a temperature), HOMING, and for moves of the
    // extruder alone outside a print EXTRUDING / RETRACTING (LOADING / UNLOADING from 20 mm), each kept
    // for as long as the move takes plus a second; "" when nothing is going on.
    // CAPTAIN / SKIPPER (machine_pro.cpp): resonance test, bed mesh, screw tilt, exclude object
    bool probing_run() const { return pr_.active; }
    bool resonance_running() const { return rt_.stage != 0; }
    const std::string& screws_text() const { return screws_text_; }

    struct DisplayInfo {
        bool have_ext = false, have_bed = false;
        double ext_temp = 0, ext_target = 0, bed_temp = 0, bed_target = 0;
        std::string state = "standby";
        std::string activity;
    };
    DisplayInfo display_info(double now) const;

private:
    std::string act_text_;     // the last extruder-only move, for display_info()
    double act_until_ = 0, last_feed_ = 25;
    void note_e_move(double de, double feed_mm_s, double now);
    struct HeaterRun {
        double target = 0, last_temp = 0, last_time = 0, smoothed = 0, rate = 0;
        bool have_temp = false;
        double last_pwm = 0, next_pwm_time = 0;
        bool pending = false;
        double pending_value = 0, pending_time = 0;
        // PID
        double prev_temp = 25, prev_time = 0, prev_deriv = 0, prev_integ = 0;
        bool heating = false;   // watermark
        // verify_heater
        bool approaching = false, starting = false;
        double last_target = 0, goal_temp = 0, goal_time = 0, error = 0, next_check = 0;
        double max_power_seen = 0;
        // PID_CALIBRATE (Klipper pid_calibrate.py ControlAutoTune)
        bool tuning = false, tune_heating = false;
        double tune_temp = 0, peak = 0, peak_time = 0, tune_started = 0;
        std::vector<std::pair<double, double>> peaks;   // (temperature, time)
        bool unsaved = false;                           // tuned values not yet in printer.cfg
        // stuck-output check: the output has been off since off_since; lowest temperature since then
        double off_since = -1, off_min = 1e9;
        // heater output check (OUTPUT_TEST): pulses the output so its LED can be watched
        bool out_test = false, out_on = false;
        double out_t0 = 0, out_base = 0, out_after_on = 0, out_peak = -1e9, out_peak_t = 0, out_mid = 0;
        double new_kp = 0, new_ki = 0, new_kd = 0;      // as written in printer.cfg (x255)
        // history for /heater_log.csv: one sample a second, last 5 minutes
        struct Sample { float t, temp, target, power, clock_err_ms; uint16_t raw; };
        std::vector<Sample> hist;
        size_t hist_pos = 0;
        double next_hist = 0;
    };
    struct FanRun {
        double value = 0, requested = 0, kick_until = 0, after_kick = 0;
        bool override_active = false;   // a fan test is driving it
    };
    struct StepperRun {
        bool enabled = false, busy = false;
        double busy_until = 0;
        std::string tmc_status = "not set up";
        bool tmc_ok = false;
        // sensorless axes: the DIAG pin, read four times a second (red box on the Test bench)
        int diag = -1;            // -1 unknown, 0 low, 1 high (stalled)
        double diag_high_at = -1; // last time it read high (the box stays red 3 s, so short triggers show)
    };
    struct HeatTest {
        int heater = -1;
        double target = 0, hold = 60, reached_at = -1, lo = 1e9, hi = -1e9, timeout = 0, max_power = 0;
    };
    struct MotorTest {
        int stepper = -1;
        double t0 = 0, end = 0, cruise_check = 0;
        int64_t start_pos = 0;
        bool have_start_pos = false, checked_drv = false;
        std::string drv_note;
        std::vector<std::pair<double, std::string>> phases;   // (start time, label)
        bool buzz_only = false;
        // STALLGUARD_TEST: sensorless set-up, SG_RESULT and DIAG sampled while cruising at `speed`;
        // half the run with the configured driver_SGTHRS, half with 255 (must trigger: wiring check)
        bool sg = false, sg_switched = false, sg_wiring = false;
        double sg_speed = 0, sg_switch_at = 0, sg_last = 0, sg_tstep = 0;
        int sg_tmc = -1, sg_endstop = -1;
        std::vector<int> sg_vals[2];
        int diag_hi[2] = {0, 0}, diag_n[2] = {0, 0};
        uint32_t sg_err_reads = 0;
    };
    struct FanTest { int fan = -1; double t0 = 0; };
    struct LedTest { double t0 = 0; int step = -1; };

    // helpers
    uint32_t clock32(double t) const;
    double fresh(double fallback) const { return io_.now ? std::max(io_.now(), fallback) : fallback; }
    bool send(const std::vector<std::string>& cmds, std::string* err = nullptr);
    void set_pwm(const PwmPin& p, double value, double when);
    void heater_temp_update(size_t i, double temp, double now);
    void heater_set_pwm(size_t i, double read_time, double value);
    void heater_check(size_t i, double now);
    void set_target(size_t i, double t);
    void fan_set(size_t i, double speed, double now);
    void stepper_enable(size_t i, bool on, double now);
    void shutdown(const std::string& why, double now);   // emergency_stop
    void mark_shutdown(const std::string& why);
    // [homing_override] / [gcode_macro]: lines run one after another; each waits for the moves,
    // homing and (M109 / M190) the temperature before the next.
    struct MacroRun {
        std::string name;
        std::vector<std::string> lines;
        size_t pos = 0;
        bool active = false;
        std::string error;
        int wait_heater = -1;
        double wait_target = 0;
        int assumed = 0;   // axes set by set_position_* (bit 0 = X)
    };
    MacroRun mac_;
    void macro_failed();
    // ---- machine_pro.cpp
    bool pro_command(const std::string& name, const std::map<std::string, std::string>& p, const GLine* g, double now,
                     std::string& reply);
    void pro_tick(double now);
    std::string pro_status_json(double now) const;
    // A print line the extras take care of (excluded objects, BED_MESH_CALIBRATE): true if handled.
    bool pro_print_line(const std::string& name, const std::map<std::string, std::string>& p, const GLine* g, double now,
                        std::string& err);
    std::string start_probe_run(int kind, const std::map<std::string, std::string>& p, double now);
    std::string start_resonance_test(const std::map<std::string, std::string>& p, bool calibrate, double now);
    void stop_pro(const std::string& why, double now);   // TEST_ABORT / M112: probing and the resonance test
    struct ProbeRun {
        bool active = false;
        int kind = 0;   // 1 bed mesh, 2 screws
        std::vector<std::pair<double, double>> pts;   // nozzle X, Y
        std::vector<int> slot;                         // where each result goes
        std::vector<double> z;
        size_t idx = 0;
        int stage = 0;
        double hz = 5, speed = 50;
        std::string direction, error;
        BedMesh mesh;
    } pr_;
    struct ResTest {
        int stage = 0;   // 0 idle, 1 moving to the point, 2 settling, 3 running
        int axis = 0;
        bool calibrate = false;
        std::vector<TestPoint> seq;
        ResonanceMotion m;
        double t = 0, pt[3] = {0, 0, 0};
    } rt_;
    struct Objects {
        std::vector<std::string> names;
        std::vector<std::pair<double, double>> centers;
        std::vector<std::vector<std::pair<float, float>>> polygons;   // POLYGON=[[x,y],...] (SkiPiPir)
        std::vector<std::pair<double, double>> heights;               // PI_Z0 / PI_Z1 (Pasta Inspector), NAN unknown
        unsigned rev = 0;                                             // changes whenever the list does
        std::vector<std::string> excluded;
        std::string current;
        bool is_excluded(const std::string& n) const;
    } obj_;
    std::string screws_text_, screws_json_;
    // TUNING_TOWER (Klipper tuning_tower.py): a command run with a value that follows the height
    struct Tower {
        bool on = false;
        std::string fmt_cmd, param;   // e.g. "SET_PRESSURE_ADVANCE", "ADVANCE"
        double start = 0, factor = 0, band = 0, step_delta = 0, step_height = 0, skip = 0;
        double last_z = -1e9, last_value = NAN;
        double calc(double z) const;
    } tt_;
    void tower_after_move(const double before[4], double now);
    std::string set_shaping_cmd(const std::string& name, const std::map<std::string, std::string>& p, double now);
    // machine_panel.cpp: the buttons on the case (SF_PREHEAT, SF_LOAD_FILAMENT, ...)
    static bool is_panel_command(const std::string& name);
    std::string panel_command(const std::string& name, const std::map<std::string, std::string>& p, double now);
    bool in_override_ = false, in_macro_ = false;
    std::string start_macro(const std::string& name, const std::vector<std::string>& lines, double now);
    void macro_tick(double now);
    void stop_macro(const std::string& why, double now);   // also stops homing it started
    bool override_applies(const GLine* g) const;
    // STALLGUARD_HOME: the real homing move of one axis against its hard stop, then back off
    struct HardStop { bool active = false, aborted = false; size_t st = 0; int axis = 0; double t0 = 0; };
    HardStop hs_;
    std::string start_hard_stop(size_t i, double now);
    void hard_stop_tick(double now);
    bool next_sg_wiring_ = false;
public:
    bool macro_running() const { return mac_.active; }
    const std::string& macro_error() const { return mac_.error; }
private:
    // The last timed commands sent (host time, text), to name the culprit of a "Timer too close".
    struct Sent { double t; std::string cmd; };
    std::vector<Sent> trace_;
    size_t trace_pos_ = 0;
    void trace(const std::vector<std::string>& cmds);
    std::string timing_report(uint32_t shutdown_clock) const;
public:
    const std::string& shutdown_detail() const { return shutdown_detail_; }
private:
    std::string shutdown_detail_;
    Test& test(const std::string& id);
    void finish(Test& t, const std::string& state, const std::string& detail);
    // TMC
    bool tmc_read(const TmcObj& t, uint8_t reg, uint32_t& val, std::string* err);
    bool tmc_write(const TmcObj& t, uint8_t reg, uint32_t val, std::string* err);
    std::string tmc_setup(size_t stepper);
    std::string tmc_status_line(size_t stepper, uint32_t* drv = nullptr);
    std::string dump_tmc(size_t stepper);
    // tests
    std::string start_heat_test(size_t heater, double target, double hold, double now);
    // mode: 0 full test (CW, CCW, shake), 1 Klipper's STEPPER_BUZZ, 2 shake only
    // mode 3: STALLGUARD_TEST (rpm and seconds ignored; sg_speed mm/s, sg_dist mm each way)
    std::string start_motor_test(size_t stepper, double rpm, double seconds, int mode, double now,
                                 double sg_speed = 0, double sg_dist = 0);
    void sg_registers(size_t stepper, bool sensorless, int sgthrs);
    void poll_diag(double now);
    std::string set_current(size_t stepper, double amps, bool save);
    std::string set_stallguard(size_t stepper, int sgthrs, bool save);
    std::string set_rotation_distance(size_t stepper, double rd, double now);
    double next_diag_poll_ = 0;
    void sg_sample(size_t stepper, double now);
    std::string sg_report(size_t stepper, bool& ok);
    std::string start_pid_tune(size_t heater, double target, double now);
    std::string start_output_test(size_t heater, double now);
    void run_output_test(size_t heater, double now);
    void tune_update(size_t heater, double temp, double now);
    void tune_check_peaks(size_t heater);
    void tune_pid(size_t heater, size_t pos, double& kp, double& ki, double& kd) const;
    std::string save_config(double now);
    std::string start_fan_test(size_t fan, double now);
    std::string start_led_test(double now);
    std::string start_all(double now);
    std::string tmc_check_all(double now);
    void abort_test(const std::string& id, double now, const std::string& why = "aborted");
    void run_tests(double now);
    bool stepper_position(size_t i, int64_t& pos);

    BoardPlan& plan_;
    const MsgProto& mp_;
    MachineIO io_;
    std::vector<HeaterRun> hr_;
    std::vector<FanRun> fr_;
    std::vector<StepperRun> sr_;
    std::vector<double> sensor_rate_, sensor_prev_, sensor_prev_time_;
    std::vector<uint32_t> last_clock_;   // per oid: queue_digital_out clocks must increase
    std::vector<Test> tests_;
    std::vector<HeatTest> heat_tests_;     // per heater
    std::vector<MotorTest> motor_tests_;   // per stepper
    std::vector<FanTest> fan_tests_;       // per fan
    LedTest led_test_;
    std::vector<std::string> all_queue_;   // test ids still to start in TEST_ALL
    bool all_running_ = false;
    bool shutdown_ = false, restart_ = false;
    bool power_off_now_ = false;   // a heater heated with its output off: tell the user to cut the power
    std::string shutdown_reason_;
    double last_fan_auto_ = 0;
    // How far our idea of the board clock is from the board's own (from the ADC reports' next_clock),
    // in ms. Heater updates are scheduled with it: a large error means late or early heater changes.
    double clock_err_ms_ = 0;
    // moves, homing, probing
    std::unique_ptr<Toolhead> th_;
    double last_now_ = 0, last_activity_ = 0, next_host_adc_ = 0;
    // the print job (an uploaded G-code file)
    struct Job {
        std::string state = "idle";   // idle, printing, paused, done, cancelled, error
        std::string file, msg;
        double started = 0, ended = 0;
        long lines = 0, ignored = 0;
        std::string ignored_names;
        int wait = 0;                 // 0 none, 1 temperature, 2 motion idle, 3 idle then motors off
        int wait_heater = -1;
        double wait_target = 0;
        // From the slicer's comments, when it writes them (Cura, PrusaSlicer, OrcaSlicer); 0 / -1 unknown.
        int layer = 0, layers = 0;
        double layer_h = 0, layer_z = -1, slicer_s = 0;
        double e_start = 0;           // Toolhead::extruded() when it started (filament_used)
    } job_;
    void print_comment(const std::string& c);
    double print_eta(double now, double progress) const;
    std::string print_start(double now, const std::string& file = "");
    std::string selected_file_;   // M23
    void print_tick(double now);
    void tick_body(double now);
    std::string bench_start(const std::map<std::string, std::string>& p, double now);
    void bench_finish(const std::string& state, double now);
    bool bench_ = false;
    double bench_t0_ = 0;
    uint64_t bench_busy_ = 0;
    std::string bench_report_, bench_file_;
    std::string print_exec(const std::string& line, double now);
    void print_end(const std::string& state, const std::string& msg, double now);
    bool have_clock_err_ = false;
};

}  // namespace ph
