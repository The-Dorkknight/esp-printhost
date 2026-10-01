// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Minimal G-code line parser.
#pragma once
#include <string>

namespace ph {

struct GLine {
    char cmd_letter = 0;      // 'G', 'M', 'T' or 0 for empty/comment-only
    int  cmd_num = -1;        // 1 for G1, 104 for M104
    int  cmd_sub = -1;        // G29.1 -> 1 (rare)
    long line_no = -1;        // N word, if present
    bool checksum_ok = true;  // '*' checksum present and wrong -> false
    std::string text_arg;     // raw text after cmd for M117/M118/M23/M28 etc.

    bool has(char c) const { return seen_[idx(c)]; }
    float get(char c, float dflt = 0.f) const { return has(c) ? val_[idx(c)] : dflt; }
    bool is(char l, int n) const { return cmd_letter == l && cmd_num == n; }

    // internal
    static int idx(char c) { return (c & ~0x20) - 'A'; }
    bool  seen_[26] = {};
    float val_[26] = {};
};

// Returns false only for malformed words (e.g. "G1 X1.2.3").
bool gcode_parse(const std::string& line, GLine& out);

}  // namespace ph
