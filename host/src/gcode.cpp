// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/gcode.h"
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace ph {
namespace {

bool takes_text(char l, int n) {
    // Commands whose argument is free text, not letter/number words.
    return l == 'M' && (n == 23 || n == 28 || n == 30 || n == 32 || n == 117 ||
                        n == 118 || n == 550 || n == 587);
}

}  // namespace

bool gcode_parse(const std::string& raw, GLine& g) {
    g = GLine{};
    std::string s = raw;

    // Checksum: "N12 G1 X5*57" -> xor of all bytes before '*'.
    size_t star = s.find('*');
    if (star != std::string::npos) {
        uint8_t cs = 0;
        for (size_t i = 0; i < star; i++) cs ^= uint8_t(s[i]);
        g.checksum_ok = (atoi(s.c_str() + star + 1) == cs);
        s.resize(star);
    }

    const char* p = s.c_str();
    const char* e = p + s.size();
    bool first_cmd = true;
    while (p < e) {
        char c = *p;
        if (c == ';') break;
        if (c == '(') { while (p < e && *p != ')') p++; if (p < e) p++; continue; }
        if (isspace((unsigned char)c)) { p++; continue; }
        if (!isalpha((unsigned char)c)) return false;
        char L = char(toupper((unsigned char)c));
        p++;
        while (p < e && *p == ' ') p++;  // allow "G 1"
        char* end = nullptr;
        double v = strtod(p, &end);
        bool has_num = end != p;
        if (has_num) {
            // reject "1.2.3"
            if (end < e && *end == '.') return false;
            p = end;
        }
        if (L == 'N' && g.cmd_letter == 0) { g.line_no = long(v); continue; }
        if (first_cmd && (L == 'G' || L == 'M' || L == 'T') && has_num) {
            g.cmd_letter = L;
            g.cmd_num = int(v);
            double frac = v - int(v);
            if (frac > 1e-6) g.cmd_sub = int(frac * 10 + 0.5);
            first_cmd = false;
            if (takes_text(L, g.cmd_num)) {
                while (p < e && isspace((unsigned char)*p)) p++;
                g.text_arg.assign(p, e);
                // strip trailing comment/space
                size_t sc = g.text_arg.find(';');
                if (sc != std::string::npos) g.text_arg.resize(sc);
                while (!g.text_arg.empty() && isspace((unsigned char)g.text_arg.back()))
                    g.text_arg.pop_back();
                return true;
            }
            continue;
        }
        int i = GLine::idx(L);
        g.seen_[i] = true;
        g.val_[i] = has_num ? float(v) : 0.f;
    }
    return true;
}

}  // namespace ph
