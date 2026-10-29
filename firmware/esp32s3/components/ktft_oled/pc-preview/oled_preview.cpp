// Renders the OLED layouts to PBM files so they can be checked (and shown in the docs).
//   oled_preview <out_dir>
// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// From KlipperTFT-Pico 0.6.1 (same author; published there under PolyForm Noncommercial 1.0.0).
#include <cstdio>
#include <string>

#include "oled.hpp"

using namespace ktft;

static void save(const OledFrame& f, const std::string& path) {
    FILE* fp = fopen(path.c_str(), "w");
    if (!fp) return;
    fprintf(fp, "P1\n%d %d\n", f.width(), f.height());
    for (int y = 0; y < f.height(); y++) {
        for (int x = 0; x < f.width(); x++) fputs(f.get(x, y) ? "1 " : "0 ", fp);
        fputc('\n', fp);
    }
    fclose(fp);
}

int main(int argc, char** argv) {
    std::string dir = argc > 1 ? argv[1] : ".";
    for (int h : {64, 32}) {
        OledFrame f(h);
        std::string sfx = h == 64 ? "_128x64" : "_128x32";
        DisplayState s;
        s.link = DisplayState::Link::Ok;
        s.have_status = true;
        s.st.state = "standby";
        s.st.ext_temp = 214.6; s.st.ext_target = 215;
        s.st.bed_temp = 48.2; s.st.bed_target = 60;
        render_display(f, s, 0); save(f, dir + "/idle" + sfx + ".pbm");

        s.st.ext_temp = 23; s.st.ext_target = 0; s.st.bed_temp = 22; s.st.bed_target = 0;
        render_display(f, s, 0); save(f, dir + "/cold" + sfx + ".pbm");

        s.st.state = "printing"; s.st.progress = 0.47;
        render_display(f, s, 0); save(f, dir + "/printing" + sfx + ".pbm");
        s.st.state = "paused";
        render_display(f, s, 2500); save(f, dir + "/paused" + sfx + ".pbm");

        s.st.state = "standby"; s.st.ext_temp = 180; s.st.ext_target = 200;
        s.activity = "LOADING";
        render_display(f, s, 0); save(f, dir + "/loading_temps" + sfx + ".pbm");
        render_display(f, s, 2500); save(f, dir + "/loading_flash" + sfx + ".pbm");
        s.activity = "HEATING";
        render_display(f, s, 2500); save(f, dir + "/heating_flash" + sfx + ".pbm");
        s.activity.clear();

        DisplayState w;
        w.link = DisplayState::Link::Setup;
        w.line1 = "KlipperTFT-1A2B";
        w.line2 = "pass k7mxq2rtv9";
        w.line3 = "then open 192.168.4.1";
        render_display(f, w, 0); save(f, dir + "/setup" + sfx + ".pbm");
        DisplayState n;
        n.link = DisplayState::Link::NoKlipper;
        n.line1 = "Klipper address not set";
        n.line2 = "open the settings page";
        render_display(f, n, 0); save(f, dir + "/noklipper" + sfx + ".pbm");
        s.st.state = "error";
        render_display(f, s, 0); save(f, dir + "/error" + sfx + ".pbm");

        // Skipper Flipper's own screens (main/oled.cpp builds these from the link state)
        DisplayState nb;
        nb.link = DisplayState::Link::NoKlipper;
        nb.title = "NO BOARD";
        nb.line1 = "board not answering";
        nb.line2 = "SkipperFlipper-1A2B";
        nb.line3 = "pass skipperflipper";
        render_display(f, nb, 0); save(f, dir + "/sf_noboard" + sfx + ".pbm");
        nb.line3 = "192.168.4.1";
        render_display(f, nb, 0); save(f, dir + "/sf_noboard_addr" + sfx + ".pbm");
        DisplayState cc = nb;
        cc.title = "CHECK CFG";
        cc.line1 = "printer.cfg has errors";
        render_display(f, cc, 0); save(f, dir + "/sf_checkcfg" + sfx + ".pbm");
        DisplayState hm = s;
        hm.st.state = "standby";
        hm.activity = "HOMING";
        render_display(f, hm, 2500); save(f, dir + "/sf_homing" + sfx + ".pbm");
        hm.st.state = "shutdown";
        hm.activity.clear();
        render_display(f, hm, 0); save(f, dir + "/sf_shutdown" + sfx + ".pbm");
    }
    return 0;
}
