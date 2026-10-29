// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Writes the SF_BENCH benchmark file to stdout: build/bench_gcode > docs/bench/sf-bench.gcode
#include "ph/printbench.h"

#include <cstdio>

int main() {
    ph::BenchGcode g;
    std::string l;
    while (g.next(l)) printf("%s\n", l.c_str());
    return 0;
}
