// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Tiny shared test macros; counters live in test_main.cpp.
#pragma once
#include <cmath>
#include <cstdio>

extern int g_fail, g_pass;
#define CHECK(c) do { if (c) g_pass++; else { g_fail++; \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, eps) CHECK(std::fabs((a) - (b)) <= (eps))

void test_segq_core();   // test_segq_core.cpp
void test_engine();      // test_engine.cpp
void test_kboard();      // test_kboard.cpp
void test_clocksync();   // test_kboard.cpp
void test_boardcfg();    // test_kboard.cpp
void test_thermistor();  // test_kboard.cpp
void test_temperature_sensors();  // test_kboard.cpp
void test_machine();     // test_machine.cpp
void test_toolhead();    // test_toolhead.cpp
void test_multipart();   // test_multipart.cpp
void test_shaping();     // test_shaping.cpp
void test_resonance();   // test_resonance.cpp
