// Tiny monochrome OLED support (SSD1306 / SSD1315 / SH1106, 128x64 or 128x32): a framebuffer,
// bitmap fonts and the screen layouts. Hardware drivers only push the buffer out over I2C.
// Stand-alone: needs only the C++ standard library (oled.cpp, oled_fonts.cpp, this header).
// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// From KlipperTFT-Pico 0.6.1 (same author; published there under PolyForm Noncommercial 1.0.0).
#pragma once
#include <cstdint>
#include <string>


namespace ktft {

struct OledFont {
    const char* chars;      // characters present, in table order
    int count;
    int height;             // cell height in pixels (glyph tops at row 0)
    const uint8_t* data;    // columns, ceil(height/8) bytes each, bit 0 = top
    const uint16_t* off;    // byte offset of each glyph
    const uint8_t* w;       // width of each glyph in columns
};
extern const OledFont kFontSmall, kFontMedium, kFontBig, kFontHuge;

class OledFrame {
public:
    explicit OledFrame(int height = 64) { set_height(height); }
    void set_height(int h) { h_ = (h == 32) ? 32 : 64; clear(); }
    int width() const { return 128; }
    int height() const { return h_; }
    void clear();
    void pixel(int x, int y, bool on = true);
    bool get(int x, int y) const;
    void fill(int x, int y, int w, int h, bool on = true);
    void frame(int x, int y, int w, int h);
    // Draws text with its top-left at (x, y); returns the width drawn. inverse: dark on light.
    int text(int x, int y, const OledFont& f, const std::string& s, bool inverse = false);
    static int text_width(const OledFont& f, const std::string& s);
    void text_centered(int y, const OledFont& f, const std::string& s, int x0 = 0, int w = 128);
    void text_right(int right, int y, const OledFont& f, const std::string& s);
    void degree(int x, int y, int r);                  // small ring for "°"
    const uint8_t* page(int p) const { return buf_ + p * 128; }   // SSD1306 page order
    int pages() const { return h_ / 8; }
    void rotate180();                                   // for upside-down mounting

private:
    int h_ = 64;
    uint8_t buf_[128 * 8];
};

// "What is going on" for the screen: load/unload/extrude/heating, shown flashing.
class ActivityTracker {
public:
    void set(const std::string& text, uint32_t now_ms, uint32_t duration_ms);
    void clear() { text_.clear(); }
    bool active(uint32_t now_ms) const { return !text_.empty() && (int32_t)(until_ - now_ms) > 0; }
    const std::string& text() const { return text_; }

private:
    std::string text_;
    uint32_t until_ = 0;
};

// What the screen needs to know about the printer.
struct OledStatus {
    double ext_temp = 0, ext_target = 0, bed_temp = 0, bed_target = 0;
    std::string state;                 // standby / printing / paused / complete / error / shutdown
    double progress = 0;               // 0..1, used while printing or paused
};

struct DisplayState {
    enum class Link { Ok, Connecting, Setup, NoKlipper };
    Link link = Link::Connecting;
    std::string line1, line2, line3;   // detail lines for Setup / NoKlipper (small font)
    std::string title;                 // optional heading for those screens instead of the built-in one
                                       // (e.g. "NO BOARD" on a firmware that is not talking to Klipper)
    bool have_status = false;
    OledStatus st;
    std::string activity;              // empty: none
    bool chamber_valid = false;
    double chamber = 0;
};

// Lays out one frame:
//   idle      hotend and bed temperatures, as big as they fit
//   printing  big percentage and a big progress bar (paused: "PAUSED" alternates with it)
//   activity  the temperatures, with the activity (LOADING, HEATING...) shown 2 s in every 4 s
//   no link   what is wrong and how to fix it (WiFi setup details on the ESP32)
void render_display(OledFrame& f, const DisplayState& s, uint32_t now_ms);

// Splits a message over up to three small-font lines of the screen width.
void wrap_small(const std::string& msg, std::string& l1, std::string& l2, std::string& l3);

// Init commands for the controller (sent with control byte 0x00). Returns the count.
int oled_init_sequence(bool sh1106, int height, bool rotate, uint8_t* out, int max);

}  // namespace ktft
