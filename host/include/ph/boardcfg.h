// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// From printer.cfg text to the board's configuration, Klipper-style. Supported so far:
//   [neopixel NAME]  pin, chain_count, color_order, initial_RED/GREEN/BLUE/WHITE
//   [temperature_sensor NAME]  sensor_type (a thermistor), sensor_pin, pullup_resistor,
//                    inline_resistor, min_temp, max_temp  (read only, no heater)
//   [thermistor NAME]  temperature1..3 + resistance1..3, or temperature1 + resistance1 + beta
//   [extruder] / [heater_bed]  heater_pin, sensor_type, sensor_pin, control (pid / watermark),
//                    pid_Kp/Ki/Kd, max_delta, min_temp, max_temp, max_power, smooth_time, pwm_cycle_time
//   [verify_heater NAME]  max_error, check_gain_time, hysteresis, heating_gain
//   [stepper_x] .. and the stepper part of [extruder]  step_pin, dir_pin, enable_pin, microsteps,
//                    rotation_distance, full_steps_per_rotation (motor tests only: no kinematics yet)
//   [tmc2209 NAME]   uart_pin, tx_pin, uart_address, run_current, hold_current, sense_resistor,
//                    stealthchop_threshold, interpolate
//   [printer]        kinematics (cartesian), max_velocity, max_accel, max_z_velocity, max_z_accel,
//                    square_corner_velocity
//   endstop_pin, position_endstop/min/max, homing_speed, homing_retract_dist ... in [stepper_x/y/z];
//                    "tmc2209_stepper_x:virtual_endstop" = sensorless homing (diag_pin, driver_SGTHRS)
//   [probe]          pin (also a TMC virtual endstop), x/y/z_offset, speed, lift_speed
//   [fan] / [heater_fan NAME]  pin, max_power, cycle_time, kick_start_time, off_below
//                    (+ heater, heater_temp, fan_speed)
// Other sections are listed as "not used yet" (so a full Klipper printer.cfg still loads); options
// Klipper knows but this project does not use yet (endstop_pin, position_max, ...) are accepted.
// Also: SET_LED, with Klipper's parameter names. The runtime (heater control, tests, G-code) is
// ph::Machine (machine.h).
#pragma once
#include <cmath>
#include "ph/kboard.h"
#include "ph/msgproto.h"
#include "ph/bedmesh.h"
#include "ph/resonance.h"
#include "ph/shaping.h"
#include "ph/thermistor.h"
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ph {

struct NeopixelObj {
    std::string name;     // "board_led" from [neopixel board_led]
    int oid = 0;
    std::string pin;
    int chain_count = 1;
    std::string order;    // e.g. "GRB"
    std::vector<uint8_t> data;   // chain_count * order.size() bytes, in the LED's own order
};

struct TempSensorObj {
    std::string name;     // "hotend" from [temperature_sensor hotend]
    int oid = 0;
    std::string pin;
    std::string type;     // sensor_type
    Thermistor th;
    int heater = -1;      // index into BoardPlan::heaters if this is a heater's sensor
    // sensor_pin: esp32:gpioN (2026-10-27): read by the ESP32-S3 itself (a chamber thermistor on a spare
    // ESP32 pin), not by the printer board: no config_analog_in, no board range check, never a shutdown.
    int host_gpio = -1;
    // ADC range check done by the board itself (query_analog_in min_value/max_value): outside it
    // for range_check_count reports in a row, the board shuts down ("ADC out of range"). Set from
    // min_temp/max_temp for heater sensors, like Klipper; 0..65535 (no check) for plain sensors.
    uint16_t min_value = 0, max_value = 65535;
    bool has_min = false, has_max = false;
    double min_temp = 0, max_temp = 0;
    // latest reading (filled by apply_adc_message)
    uint32_t updates = 0;
    double adc = -1;      // 0..1 of full scale; -1 = nothing received yet
    double temp_c = 0;
    // "waiting for the first reading", "18.2 °C", "no sensor? (reads open circuit)", ...
    std::string describe() const;
    // 0 waiting, 1 ok, 2 open circuit, 3 short circuit
    int state() const;
};

// A soft-PWM output (heater, fan): config_digital_out + set_digital_out_pwm_cycle, updated with
// queue_digital_out, as Klipper's mcu.py does for pins without hardware PWM.
struct PwmPin {
    int oid = -1;
    std::string pin;
    bool invert = false;
    double cycle_time = 0.1;
    double max_duration = 0;     // seconds; 0 = no watchdog. Heaters: 5 s (Klipper MAX_HEAT_TIME)
    uint32_t cycle_ticks = 0;
};

struct HeaterObj {
    std::string name;            // "extruder" or "heater_bed"
    int sensor = -1;             // index into BoardPlan::sensors
    PwmPin out;
    double min_temp = 0, max_temp = 0, max_power = 1, smooth_time = 1;
    bool pid = true;
    double kp = 0, ki = 0, kd = 0;   // already divided by Klipper's PID_PARAM_BASE (255)
    double max_delta = 2;            // watermark control
    // verify_heater (Klipper defaults; check_gain_time 60 s for the bed)
    double hysteresis = 5, max_error = 120, heating_gain = 2, check_gain_time = 20;
};

struct FanObj {
    std::string name;            // "fan" or the NAME of [heater_fan NAME]
    bool heater_fan = false;
    PwmPin out;
    double max_power = 1, kick_start_time = 0.1, off_below = 0;
    std::string heater;          // heater_fan: which heater (default extruder)
    double heater_temp = 50, fan_speed = 1;
};

struct TmcObj {
    std::string stepper;         // "stepper_x" from [tmc2209 stepper_x]
    int uart_oid = -1;           // shared by drivers on the same pins (config_tmcuart)
    int addr = 0;
    double run_current = 0, hold_current = 0;
    int irun = 0, ihold = 0;
    bool vsense = false, stealthchop = false;
    double rsense = 0.110;       // sense resistor, ohms
    // Register writes in the order Klipper's TMC2209 code sends them (name, address, value)
    struct Reg { std::string name; int addr; uint32_t value; };
    std::vector<Reg> init_regs;
    // sensorless homing (Klipper tmc.py TMCVirtualPinHelper)
    std::string diag_pin;        // the DIAG output, as wired to a board pin
    bool diag_pullup = false;
    int sgthrs = 0;              // driver_SGTHRS: StallGuard threshold (higher = more sensitive on a TMC2209)
    uint32_t gconf = 0, tpwmthrs = 0xfffff, tcoolthrs = 0;   // as initialised, restored after homing
};

// An endstop input on the board (config_endstop). Sensorless: the pin is the TMC2209's DIAG output.
struct EndstopObj {
    int oid = -1;
    std::string pin;
    bool pullup = false, invert = false;
    int tmc = -1;                // sensorless: index into BoardPlan::tmcs
};

struct StepperObj {
    std::string name;            // "stepper_x", ..., "extruder"
    int oid = -1, enable_oid = -1, trsync_oid = -1;
    std::string step_pin, dir_pin, enable_pin;
    bool invert_step = false, invert_dir = false, invert_enable = false;
    int microsteps = 16, full_steps = 200;
    double rotation_distance = 0;
    uint32_t pulse_ticks = 0;
    int tmc = -1;                // index into BoardPlan::tmcs
    int steps_per_rev() const { return full_steps * microsteps; }
    double steps_per_mm() const { return rotation_distance > 0 ? steps_per_rev() / rotation_distance : 0; }
    // axis limits and homing (stepper_x/y/z), Klipper's [stepper_*] options
    int endstop = -1;            // index into BoardPlan::endstops
    std::string endstop_spec;    // endstop_pin as written
    int endstop_line = 0;
    bool has_rail = false;
    double position_min = 0, position_max = 0, position_endstop = 0;
    double homing_speed = 5, second_homing_speed = 2.5, homing_retract_dist = 5;
    bool homing_positive_dir = false;
    double arm_length = 0, angle = 0;   // delta towers (stepper_a / b / c): arm and tower angle (degrees)
};

struct PrinterLimits {           // [printer]
    bool present = false;
    bool corexy = false;         // kinematics: corexy (2026-10-27): stepper_x = motor A, stepper_y = motor B
    bool delta = false;          // kinematics: delta (2026-10-31): stepper_a / b / c are the towers
    double delta_radius = 0, print_radius = 0, minimum_z_position = 0, arm_length = 0;   // delta
    double max_velocity = 0, max_accel = 0, max_z_velocity = 0, max_z_accel = 0;
    double square_corner_velocity = 5;
    double min_extrude_temp = 170, max_extrude_only_velocity = 0;   // from [extruder]
};

struct ProbeObj {                // [probe]: here usually sensorless on Z
    bool present = false;
    int endstop = -1;
    double x_offset = 0, y_offset = 0, z_offset = 0, speed = 5, lift_speed = 5;
};

// [gcode_macro NAME] / [homing_override]: plain G-code lines (no Jinja templates)
struct MacroObj {
    std::string name;                    // upper case, as typed in the console
    std::vector<std::string> lines;
};

struct BoardPlan {
    std::vector<std::string> cmds;       // config commands (without allocate_oids / finalize_config)
    int oid_count = 0;
    std::vector<NeopixelObj> neopixels;
    std::vector<TempSensorObj> sensors;
    std::vector<HeaterObj> heaters;
    std::vector<FanObj> fans;
    std::vector<StepperObj> steppers;
    std::vector<TmcObj> tmcs;
    std::vector<EndstopObj> endstops;
    PrinterLimits printer;
    ProbeObj probe;
    ShapingConfig shaping;               // [input_shaper] + [extruder] pressure_advance (CAPTAIN / SKIPPER)
    MeshConfig mesh;                     // [bed_mesh]
    BedMesh mesh_default;                // [bed_mesh default]: the saved mesh, used from the start
    ScrewsConfig screws;                 // [screws_tilt_adjust]
    RetractConfig retract;               // [firmware_retraction]
    double arc_resolution = 1.0;         // [gcode_arcs] resolution (mm)
    ResonanceParams resonance;           // [resonance_tester]
    bool resonance_point_set = false;
    double resonance_point[3] = {0, 0, 0};
    std::vector<MacroObj> macros;        // [gcode_macro ...]
    std::vector<std::string> homing_override;   // [homing_override] gcode: G28 runs these instead
    std::string homing_override_axes = "xyz";   // only when homing one of these (Klipper's "axes")
    // Klipper's set_position_x/y/z: the axis is assumed to be here before the override runs (so it
    // can lift Z before homing X and Y). NAN: not set.
    double homing_override_set[3] = {NAN, NAN, NAN};
    const MacroObj* macro(const std::string& name) const;
    int move_count = 0;          // the board's move queue size (get_config), set after configuring
    double clock_freq = 0;
    double adc_full_scale = 0;           // sample_count * ADC_MAX: what analog_in_state reports at 100 %
    std::vector<std::string> errors;     // with line numbers; the plan is not used if non-empty
    std::vector<std::string> notes;      // e.g. sections not supported yet
    std::set<std::string> skipped_sections;   // sections of a supported kind we chose not to use
    NeopixelObj* neopixel(const std::string& name);
    TempSensorObj* sensor(const std::string& name);
    HeaterObj* heater(const std::string& name);
    FanObj* fan(const std::string& name);
    StepperObj* stepper(const std::string& name);
};

// Build the plan. `mp` is the board's data dictionary (pin names and supported commands).
BoardPlan build_board_plan(const std::string& cfg_text, const MsgProto& mp, double clock_freq);

// Commands to send after the board is configured (Klipper's "init" commands, not part of the
// config CRC): query_analog_in for every sensor, starting at board clock `start_clock`.
std::vector<std::string> adc_query_cmds(const BoardPlan& plan, uint32_t start_clock, double clock_freq);
// Feed a message from the board: analog_in_state updates the matching sensor. True if used.
bool apply_adc_message(BoardPlan& plan, const Message& m);

// TMC2209 register values for a driver, as Klipper's tmc2209.py would send them (exposed for tests).
// microsteps 1..256, rotation_distance and full_steps for the stealthChop threshold.
// Klipper's TMCCurrentHelper: the CS value (0-31) for a current in amps RMS, and back.
int tmc2209_cs(double current, bool vsense, double rsense);
double tmc2209_current(int cs, bool vsense, double rsense);
bool tmc2209_registers(TmcObj& t, int microsteps, double rotation_distance, int full_steps, double sense_resistor,
                       bool has_stealth, double stealth_velocity, bool interpolate, std::string* err);

// SET_LED LED=<name> [RED=] [GREEN=] [BLUE=] [WHITE=] [INDEX=]  (values 0.0-1.0, INDEX 1-based,
// absent INDEX = every LED in the chain). Updates the object's data; the caller then writes it with
// KBoard::neopixel_write. Returns the object, or nullptr with a message.
NeopixelObj* apply_set_led(BoardPlan& plan, const std::map<std::string, std::string>& params, std::string& msg);

// Run one console command against the plan and the board. Returns the reply text.
// Supported: SET_LED, M105 (temperatures), HELP. Anything else is reported as not supported yet.
// (The ESP32 and klink_probe use ph::Machine::command, which adds heaters, fans, tests, ...)
std::string run_console_command(BoardPlan& plan, KBoard& board, const std::string& line);

}  // namespace ph
