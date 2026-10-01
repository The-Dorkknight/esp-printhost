// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/json_mini.h"
#include <cstdlib>
#include <cstring>

namespace ph {
namespace {

struct P {
    const char* s; const char* e; std::string err;
    void ws() { while (s < e && strchr(" \t\r\n", *s)) s++; }
    bool fail(const char* m) { if (err.empty()) err = m; return false; }

    bool lit(const char* w) {
        size_t n = strlen(w);
        if (size_t(e - s) < n || strncmp(s, w, n)) return fail("bad literal");
        s += n; return true;
    }
    static void put_utf8(std::string& o, uint32_t cp) {
        if (cp < 0x80) o += char(cp);
        else if (cp < 0x800) { o += char(0xC0 | cp >> 6); o += char(0x80 | (cp & 0x3F)); }
        else { o += char(0xE0 | cp >> 12); o += char(0x80 | ((cp >> 6) & 0x3F));
               o += char(0x80 | (cp & 0x3F)); }
    }
    bool string(std::string& o) {
        if (s >= e || *s != '"') return fail("expected string");
        s++;
        while (s < e && *s != '"') {
            char c = *s++;
            if (c != '\\') { o += c; continue; }
            if (s >= e) return fail("bad escape");
            char x = *s++;
            switch (x) {
                case 'n': o += '\n'; break; case 't': o += '\t'; break;
                case 'r': o += '\r'; break; case 'b': o += '\b'; break;
                case 'f': o += '\f'; break;
                case 'u': {
                    if (e - s < 4) return fail("bad \\u");
                    char h[5] = {s[0], s[1], s[2], s[3], 0};
                    put_utf8(o, uint32_t(strtoul(h, nullptr, 16)));
                    s += 4; break;
                }
                default: o += x;
            }
        }
        if (s >= e) return fail("unterminated string");
        s++; return true;
    }
    bool value(JVal& v, int depth) {
        if (depth > 32) return fail("too deep");
        ws();
        if (s >= e) return fail("unexpected end");
        char c = *s;
        if (c == '{') {
            v.type = JVal::Obj; s++; ws();
            if (s < e && *s == '}') { s++; return true; }
            for (;;) {
                ws(); std::string k;
                if (!string(k)) return false;
                ws(); if (s >= e || *s++ != ':') return fail("expected ':'");
                if (!value(v.obj[k], depth + 1)) return false;
                ws(); if (s >= e) return fail("unexpected end");
                if (*s == ',') { s++; continue; }
                if (*s == '}') { s++; return true; }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            v.type = JVal::Arr; s++; ws();
            if (s < e && *s == ']') { s++; return true; }
            for (;;) {
                v.arr.emplace_back();
                if (!value(v.arr.back(), depth + 1)) return false;
                ws(); if (s >= e) return fail("unexpected end");
                if (*s == ',') { s++; continue; }
                if (*s == ']') { s++; return true; }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') { v.type = JVal::Str; return string(v.str); }
        if (c == 't') { v.type = JVal::Bool; v.b = true; return lit("true"); }
        if (c == 'f') { v.type = JVal::Bool; v.b = false; return lit("false"); }
        if (c == 'n') { v.type = JVal::Null; return lit("null"); }
        std::string num;
        while (s < e && strchr("+-0123456789.eE", *s)) num += *s++;
        if (num.empty()) return fail("unexpected character");
        v.type = JVal::Num; v.num = strtod(num.c_str(), nullptr);
        return true;
    }
};

}  // namespace

bool json_parse(const std::string& text, JVal& out, std::string* err) {
    P p{text.data(), text.data() + text.size(), {}};
    out = JVal{};
    bool ok = p.value(out, 0);
    if (ok) { p.ws(); if (p.s != p.e) ok = p.fail("trailing data"); }
    if (!ok && err) *err = p.err;
    return ok;
}

}  // namespace ph
