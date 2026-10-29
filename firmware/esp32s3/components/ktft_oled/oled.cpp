// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// From KlipperTFT-Pico 0.6.1 (same author; published there under PolyForm Noncommercial 1.0.0).
#include "oled.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ktft {

void OledFrame::clear() { memset(buf_, 0, sizeof buf_); }

void OledFrame::pixel(int x, int y, bool on) {
    if (x < 0 || x >= 128 || y < 0 || y >= h_) return;
    uint8_t& b = buf_[(y >> 3) * 128 + x];
    uint8_t m = (uint8_t)(1u << (y & 7));
    b = on ? (uint8_t)(b | m) : (uint8_t)(b & ~m);
}

bool OledFrame::get(int x, int y) const {
    if (x < 0 || x >= 128 || y < 0 || y >= h_) return false;
    return buf_[(y >> 3) * 128 + x] & (1u << (y & 7));
}

void OledFrame::fill(int x, int y, int w, int h, bool on) {
    for (int yy = y; yy < y + h; yy++)
        for (int xx = x; xx < x + w; xx++) pixel(xx, yy, on);
}

void OledFrame::frame(int x, int y, int w, int h) {
    fill(x, y, w, 1);
    fill(x, y + h - 1, w, 1);
    fill(x, y, 1, h);
    fill(x + w - 1, y, 1, h);
}

static int glyph_index(const OledFont& f, char c) {
    const char* p = strchr(f.chars, c);
    if ((!p || !c) && c >= 'a' && c <= 'z') p = strchr(f.chars, c - 'a' + 'A');   // big fonts: capitals only
    return p && c ? (int)(p - f.chars) : -1;
}

int OledFrame::text_width(const OledFont& f, const std::string& s) {
    int w = 0;
    for (char c : s) {
        int i = glyph_index(f, c);
        w += (i < 0 ? f.height / 4 : f.w[i]) + 1;
    }
    return w > 0 ? w - 1 : 0;
}

int OledFrame::text(int x, int y, const OledFont& f, const std::string& s, bool inverse) {
    int nb = (f.height + 7) / 8, x0 = x;
    for (char c : s) {
        int i = glyph_index(f, c);
        int gw = i < 0 ? f.height / 4 : f.w[i];
        for (int col = 0; col < gw; col++) {
            uint64_t bits = 0;
            if (i >= 0)
                for (int b = 0; b < nb; b++) bits |= (uint64_t)f.data[f.off[i] + col * nb + b] << (8 * b);
            for (int row = 0; row < f.height; row++) {
                bool on = (bits >> row) & 1;
                if (inverse) pixel(x + col, y + row, !on);
                else if (on) pixel(x + col, y + row, true);
            }
        }
        if (inverse) fill(x + gw, y, 1, f.height, true);
        x += gw + 1;
    }
    return x - x0 - 1;
}

void OledFrame::text_centered(int y, const OledFont& f, const std::string& s, int x0, int w) {
    text(x0 + (w - text_width(f, s)) / 2, y, f, s);
}

void OledFrame::text_right(int right, int y, const OledFont& f, const std::string& s) {
    text(right - text_width(f, s), y, f, s);
}

void OledFrame::degree(int x, int y, int r) {
    for (int a = 0; a < 360; a += 10) {
        double t = a * 3.14159265 / 180;
        pixel(x + (int)std::lround(r * std::cos(t)), y + (int)std::lround(r * std::sin(t)));
        if (r > 2) pixel(x + (int)std::lround((r - 1) * std::cos(t)), y + (int)std::lround((r - 1) * std::sin(t)));
    }
}

void OledFrame::rotate180() {
    for (int y = 0; y < h_ / 2; y++)
        for (int x = 0; x < 128; x++) {
            bool a = get(x, y), b = get(127 - x, h_ - 1 - y);
            pixel(x, y, b);
            pixel(127 - x, h_ - 1 - y, a);
        }
}

void ActivityTracker::set(const std::string& text, uint32_t now, uint32_t ms) {
    text_ = text;
    until_ = now + ms;
}

// ---- layouts -------------------------------------------------------------------------------
static std::string num(double v) {
    char b[16];
    snprintf(b, sizeof b, "%d", (int)std::lround(v));
    return b;
}

// One temperature: label + target in a narrow column, the number as big as fits.
static void temp_block(OledFrame& f, int x, int y, int w, int h, const char* label, double temp, double target) {
    std::string t = num(temp);
    if (h >= 30 && w >= 100) {                          // 128x64: label + target on the left
        int ty = y + (h - kFontBig.height) / 2 + 1;
        int right = x + w - 9;
        f.text_right(right, ty, kFontBig, t);
        f.degree(right + 5, ty + 3, 3);
        f.text(x + 1, y + 1, kFontSmall, label);
        std::string tg = target > 0 ? num(target) : "OFF";
        f.text(x + 1, y + h - kFontSmall.height, kFontSmall, tg);
        if (target > 0 && std::fabs(temp - target) > 2) {   // heating / cooling arrow
            int ax = x + OledFrame::text_width(kFontSmall, tg) + 3, ay = y + h - 6;
            bool up = temp < target;
            for (int i = 0; i < 4; i++) f.fill(ax + 3 - i, up ? ay + i : ay + 3 - i, 1 + 2 * i, 1);
        }
        return;
    }
    // 128x32 half: number fills the block, degree ring top right, label letter bottom right
    int ty = y + (h - kFontBig.height) / 2 + 1;
    int right = x + w - 9;
    f.text_right(right, ty, kFontBig, t);
    f.degree(right + 5, ty + 3, 2);
    f.text(right + 2, y + h - kFontSmall.height, kFontSmall, label);
}

static void temps(OledFrame& f, const DisplayState& s) {
    if (f.height() >= 64) {
        temp_block(f, 0, 0, 128, 32, "HOTEND", s.st.ext_temp, s.st.ext_target);
        temp_block(f, 0, 32, 128, 32, "BED", s.st.bed_temp, s.st.bed_target);
        f.fill(0, 31, 128, 1);                         // divider
    } else {
        temp_block(f, 0, 0, 63, 32, "H", s.st.ext_temp, s.st.ext_target);
        temp_block(f, 65, 0, 63, 32, "B", s.st.bed_temp, s.st.bed_target);
        f.fill(63, 2, 1, 28);
    }
}

static void big_message(OledFrame& f, const std::string& msg) {
    const OledFont* font = &kFontBig;
    if (f.height() < 64 || OledFrame::text_width(*font, msg) > 124) font = &kFontMedium;
    std::string m = msg;
    while (OledFrame::text_width(*font, m) > 126 && m.size() > 1) m.pop_back();
    f.fill(0, 0, 128, f.height());                      // inverted panel: easy to see at a glance
    int y = (f.height() - font->height) / 2;
    int x = (128 - OledFrame::text_width(*font, m)) / 2;
    f.text(x, y, *font, m, true);
}

static void progress(OledFrame& f, const DisplayState& s, uint32_t now) {
    double p = s.st.progress;
    if (p < 0) p = 0;
    if (p > 1) p = 1;
    int pct = (int)std::floor(p * 100);
    bool paused = s.st.state == "paused";
    bool show_paused = paused && (now / 2000) % 2 == 1;
    if (f.height() >= 64) {
        if (show_paused) f.text_centered(2, kFontBig, "PAUSED");
        else f.text_centered(0, kFontHuge, std::to_string(pct) + "%");
        f.frame(0, 44, 128, 20);
        f.fill(2, 46, (int)std::lround(124 * p), 16);
    } else {
        if (show_paused) f.text(0, 8, kFontMedium, "PAUSE");
        else f.text(0, 3, kFontBig, std::to_string(pct) + "%");
        f.frame(70, 6, 58, 20);
        f.fill(72, 8, (int)std::lround(54 * p), 16);
    }
}

static void lines(OledFrame& f, const std::string& built_in, const DisplayState& s) {
    const std::string& title = s.title.empty() ? built_in : s.title;
    if (f.height() >= 64) {
        f.text_centered(0, kFontMedium, title);
        int y = 22;
        for (const std::string* l : {&s.line1, &s.line2, &s.line3}) {
            if (!l->empty()) f.text_centered(y, kFontSmall, *l);
            y += 14;
        }
    } else {
        int y = 0;
        for (const std::string* l : {&title, &s.line1, &s.line2}) {
            if (!l->empty()) f.text_centered(y, kFontSmall, *l);
            y += 11;
        }
    }
}

void render_display(OledFrame& f, const DisplayState& s, uint32_t now) {
    f.clear();
    switch (s.link) {
        case DisplayState::Link::Setup: lines(f, "WIFI SETUP", s); return;
        case DisplayState::Link::NoKlipper: lines(f, "NO KLIPPER", s); return;
        case DisplayState::Link::Connecting:
            if (!s.have_status) { lines(f, "CONNECTING", s); return; }
            break;
        default: break;
    }
    if (!s.have_status) { lines(f, "WAITING", s); return; }
    const std::string& st = s.st.state;
    if (st == "error" || st == "shutdown") {
        DisplayState e = s;
        e.title.clear();
        e.line1 = "Klipper stopped";
        e.line2 = "FIRMWARE_RESTART";
        e.line3.clear();
        lines(f, "ERROR", e);
        return;
    }
    bool flash = !s.activity.empty() && (now / 2000) % 2 == 1;   // 2 s on, 2 s off
    if (flash) { big_message(f, s.activity); return; }
    if (st == "printing" || st == "paused") { progress(f, s, now); return; }
    temps(f, s);
}

void wrap_small(const std::string& msg, std::string& l1, std::string& l2, std::string& l3) {
    std::string* out[3] = {&l1, &l2, &l3};
    for (auto* o : out) o->clear();
    int line = 0;
    size_t i = 0;
    while (i < msg.size() && line < 3) {
        while (i < msg.size() && msg[i] == ' ') i++;
        size_t j = msg.find(' ', i);
        std::string word = msg.substr(i, j == std::string::npos ? std::string::npos : j - i);
        std::string trial = out[line]->empty() ? word : *out[line] + " " + word;
        if (OledFrame::text_width(kFontSmall, trial) <= 126 || out[line]->empty()) {
            *out[line] = trial;
            i = j == std::string::npos ? msg.size() : j;
        } else {
            line++;
        }
    }
    for (auto* o : out)   // a single over-long word: cut it
        while (o->size() > 1 && OledFrame::text_width(kFontSmall, *o) > 126) o->pop_back();
}

int oled_init_sequence(bool sh1106, int height, bool rotate, uint8_t* out, int max) {
    const uint8_t mux = height == 32 ? 0x1F : 0x3F;
    const uint8_t com = height == 32 ? 0x02 : 0x12;
    uint8_t seq[] = {
        0xAE,                         // display off
        0xD5, 0x80,                   // clock
        0xA8, mux,                    // multiplex
        0xD3, 0x00,                   // offset
        0x40,                         // start line 0
        0x8D, 0x14,                   // charge pump on (SSD1306/SSD1315; ignored by SH1106)
        0x20, 0x02,                   // page addressing (works on both controllers)
        0xA1,                         // segment remap (upside-down mounting is done in software)
        0xC8,                         // COM scan direction
        0xDA, com,                    // COM pins
        0x81, 0xCF,                   // contrast
        0xD9, 0xF1,                   // pre-charge
        0xDB, 0x40,                   // VCOM detect
        0xA4,                         // follow RAM
        0xA6,                         // normal (not inverted)
        0xAF,                         // display on
    };
    (void)sh1106;
    (void)rotate;
    int n = (int)sizeof seq;
    if (n > max) n = max;
    memcpy(out, seq, (size_t)n);
    return n;
}

}  // namespace ktft
