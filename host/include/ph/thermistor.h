// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Thermistor ADC reading to degrees C, the same maths as Klipper's klippy/extras/thermistor.py
// (Steinhart-Hart from three points, or a beta value), with the named sensors from Klipper's
// temperature_sensors.cfg that this project supports.
#pragma once
#include <string>

namespace ph {

struct Thermistor {
    double pullup = 4700.0;        // Klipper's default pullup_resistor
    double inline_resistor = 0.0;
    double c1 = 0, c2 = 0, c3 = 0; // Steinhart-Hart coefficients: 1/T = c1 + c2*ln(R) + c3*ln(R)^3

    // Three (temperature C, resistance ohm) points. Falls back to a beta model if the fit is
    // not physical (c3 <= 0), as Klipper does. Returns false for impossible input.
    bool setup_points(double t1, double r1, double t2, double r2, double t3, double r3);
    // One point and beta.
    bool setup_beta(double t1, double r1, double beta);
    // adc = voltage / full scale (0..1) with the thermistor from the pin to ground and the pullup to
    // the supply. Clamped to 0.00001..0.99999 like Klipper.
    double calc_temp(double adc) const;
    // The reverse: ADC fraction a given temperature would read (used for tests and range limits).
    double calc_adc(double temp_c) const;
};

// Named sensors: "EPCOS 100K B57560G104F", "ATC Semitec 104GT-2", "Generic 3950",
// "NTC 100K MGB18-104F39050L32". Returns false for any other name.
bool thermistor_preset(const std::string& name, Thermistor& out);

}  // namespace ph
