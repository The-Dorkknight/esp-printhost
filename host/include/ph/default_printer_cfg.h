// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The printer.cfg the ESP32-S3 starts with (BTT SKR Pico). Shared with the PC tests, which check it
// against the shipped SKR Pico firmware's real data dictionary. Must stay under 4 KB (NVS limit).
#pragma once

namespace ph {

// Pins and PID values from Klipper's config/generic-bigtreetech-skr-pico-v1.0.cfg (v0.13.0).
// sensor_type is Generic 3950 for both: that is what read sensibly on the first hardware test
// (Klipper's sample says EPCOS 100K for the hotend and ATC Semitec 104GT-2 for the bed).
inline const char* default_printer_cfg() {
    return "# printer.cfg for Skipper Flipper (Klipper format), BTT SKR Pico.\n"
           "# Pins from Klipper's SKR Pico sample config. Check sensor_type, run_current and PID values\n"
           "# for YOUR printer. Used so far: heaters, fans, LED, TMC2209 and motor tests.\n"
           "# Not yet: moves, homing, printing. Max 4 KB.\n"
           "# A heater whose thermistor is missing shuts the board down (Klipper's range check):\n"
           "# delete [heater_bed] if no bed thermistor is fitted.\n"
           "[mcu]\nserial: uart\n\n"
           "[printer]\nkinematics: cartesian\nmax_velocity: 300\nmax_accel: 3000\n\n"
           "[extruder]\nstep_pin: gpio14\ndir_pin: !gpio13\nenable_pin: !gpio15\nmicrosteps: 16\n"
           "rotation_distance: 33.500\nheater_pin: gpio23\n# TH0\nsensor_type: Generic 3950\nsensor_pin: gpio27\n"
           "control: pid\npid_Kp: 21.527\npid_Ki: 1.063\npid_Kd: 108.982\nmin_temp: 0\nmax_temp: 250\n\n"
           "[heater_bed]\nheater_pin: gpio21\n# TB\nsensor_type: Generic 3950\nsensor_pin: gpio26\ncontrol: pid\n"
           "pid_Kp: 54.027\npid_Ki: 0.770\npid_Kd: 948.182\nmin_temp: 0\nmax_temp: 130\n\n"
           "[stepper_x]\nstep_pin: gpio11\ndir_pin: !gpio10\nenable_pin: !gpio12\nmicrosteps: 16\n"
           "rotation_distance: 40\nendstop_pin: ^gpio4\nposition_endstop: 0\nposition_max: 235\n\n"
           "[tmc2209 stepper_x]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 0\nrun_current: 0.580\n"
           "stealthchop_threshold: 999999\n\n"
           "[stepper_y]\nstep_pin: gpio6\ndir_pin: !gpio5\nenable_pin: !gpio7\nmicrosteps: 16\n"
           "rotation_distance: 40\nendstop_pin: ^gpio3\nposition_endstop: 0\nposition_max: 235\n\n"
           "[tmc2209 stepper_y]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 2\nrun_current: 0.580\n"
           "stealthchop_threshold: 999999\n\n"
           "[stepper_z]\nstep_pin: gpio19\ndir_pin: gpio28\nenable_pin: !gpio2\nmicrosteps: 16\n"
           "rotation_distance: 8\nendstop_pin: ^gpio25\nposition_endstop: 0.0\nposition_max: 250\n\n"
           "[tmc2209 stepper_z]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 1\nrun_current: 0.580\n"
           "stealthchop_threshold: 999999\n\n"
           "[tmc2209 extruder]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 3\nrun_current: 0.650\n"
           "stealthchop_threshold: 999999\n\n"
           "# FAN0 part cooling (M106/M107); FAN1/FAN2 come on with the hotend above 50 C\n"
           "[fan]\npin: gpio17\n\n"
           "[heater_fan heatbreak_cooling_fan]\npin: gpio18\n\n"
           "[heater_fan controller_fan]\npin: gpio20\n\n"
           "[neopixel board_led]\npin: gpio24\nchain_count: 1\ncolor_order: GRB\ninitial_GREEN: 0.15\n";
}

// WEEDO Tina 2 with a BTT SKR Pico fitted, sensorless homing on every axis. Mechanics (rotation
// distances, homing ends, extruder, PID, speeds) from the community Tina 2 Klipper config
// (github.com/gistogisto/Tina2Klipper, written for the stock board); pins from the SKR Pico preset
// above. Build volume reduced (85 x 105 x 80 instead of 97 x 120 x 91) for a larger custom printhead.
inline const char* default_printer_cfg_tina2() {
    return "# printer.cfg: WEEDO Tina 2 with a BTT SKR Pico + Skipper Flipper. UNTESTED on a Tina 2.\n"
           "# Sensorless homing on X, Y, Z: needs the SKR Pico DIAG jumpers for X, Y, Z fitted.\n"
           "# Before homing, run each motor test: if an axis moves the wrong way, add or remove the\n"
           "# '!' on its dir_pin. driver_SGTHRS: higher = stops more easily (try 60-150).\n"
           "# Z homes UP (to the top, like the stock Tina 2). Then measure: jog down to a sheet of\n"
           "# paper and set position_endstop = 80 - the Z shown. Limits are reduced for a bigger head.\n"
           "[mcu]\nserial: uart\n\n"
           "[printer]\nkinematics: cartesian\nmax_velocity: 97\nmax_accel: 1000\nmax_z_velocity: 10\n"
           "max_z_accel: 250\n\n"
           "[stepper_x]\nstep_pin: gpio11\ndir_pin: !gpio10\nenable_pin: !gpio12\nmicrosteps: 16\n"
           "rotation_distance: 32\nendstop_pin: tmc2209_stepper_x:virtual_endstop\nposition_endstop: 85\n"
           "position_min: 5\nposition_max: 85\nhoming_speed: 16\nhoming_retract_dist: 0\n\n"
           "[tmc2209 stepper_x]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 0\nrun_current: 0.500\n"
           "stealthchop_threshold: 999999\ndiag_pin: ^gpio4\ndriver_SGTHRS: 100\n\n"
           "[stepper_y]\nstep_pin: gpio6\ndir_pin: !gpio5\nenable_pin: !gpio7\nmicrosteps: 16\n"
           "rotation_distance: 32\nendstop_pin: tmc2209_stepper_y:virtual_endstop\nposition_endstop: 0\n"
           "position_max: 105\nhoming_speed: 16\nhoming_retract_dist: 0\n\n"
           "[tmc2209 stepper_y]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 2\nrun_current: 0.500\n"
           "stealthchop_threshold: 999999\ndiag_pin: ^gpio3\ndriver_SGTHRS: 100\n\n"
           "[stepper_z]\nstep_pin: gpio19\ndir_pin: gpio28\nenable_pin: !gpio2\nmicrosteps: 16\n"
           "rotation_distance: 8\nendstop_pin: tmc2209_stepper_z:virtual_endstop\nposition_endstop: 80\n"
           "position_min: -2\nposition_max: 80\nhoming_speed: 4\nhoming_retract_dist: 0\n\n"
           "[tmc2209 stepper_z]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 1\nrun_current: 0.500\n"
           "stealthchop_threshold: 999999\ndiag_pin: ^gpio25\ndriver_SGTHRS: 100\n\n"
           "[extruder]\nstep_pin: gpio14\ndir_pin: !gpio13\nenable_pin: !gpio15\nmicrosteps: 16\n"
           "rotation_distance: 33.725\nnozzle_diameter: 0.4\nfilament_diameter: 1.750\nheater_pin: gpio23\n"
           "sensor_type: EPCOS 100K B57560G104F\nsensor_pin: gpio27\ncontrol: pid\npid_Kp: 41.141\n"
           "pid_Ki: 2.716\npid_Kd: 155.822\nmin_temp: 0\nmax_temp: 240\n\n"
           "[tmc2209 extruder]\nuart_pin: gpio9\ntx_pin: gpio8\nuart_address: 3\nrun_current: 0.500\n"
           "stealthchop_threshold: 999999\n\n"
           "# Sensorless probe: the nozzle is pushed onto the bed until the Z motor stalls. Watch it.\n"
           "[probe]\npin: tmc2209_stepper_z:virtual_endstop\nz_offset: 0\nspeed: 2\n\n"
           "# The SKR Pico's three fan outputs. Test bench: run each Fan test to see which port your fans\n"
           "# are on (the board's LED by that port lights), then swap the pins here if needed.\n"
           "# [fan] = part cooling (M106/M107); heater fans run while the hotend is above 50 C.\n"
           "[fan]\npin: gpio17\n\n"
           "[heater_fan hotend_fan]\npin: gpio18\n\n"
           "[heater_fan fan3]\npin: gpio20\n\n"
           "[neopixel board_led]\npin: gpio24\nchain_count: 1\ncolor_order: GRB\ninitial_GREEN: 0.15\n";
}

}  // namespace ph
