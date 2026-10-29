// Unit tests for the OLED module (from KlipperTFT-Pico). Build and run on a PC:
//   g++ -std=c++17 -I components/ktft_oled/include components/ktft_oled/oled.cpp
//       components/ktft_oled/oled_fonts.cpp tests/oled_test.cpp -o oled_test && ./oled_test
#include <cstdio>
#include <string>

#include "oled.hpp"

using namespace ktft;
static int failures = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static int lit(const OledFrame& f, int x0, int y0, int w, int h) {
    int n = 0;
    for (int y = y0; y < y0 + h; y++)
        for (int x = x0; x < x0 + w; x++) n += f.get(x, y);
    return n;
}

static void test_oled() {
    OledFrame f(64);
    CHECK(f.pages() == 8 && f.height() == 64);
    f.pixel(3, 9);
    CHECK(f.get(3, 9) && f.page(1)[3] == 0x02);
    f.rotate180();
    CHECK(f.get(124, 54) && !f.get(3, 9));
    CHECK(OledFrame::text_width(kFontBig, "215") > OledFrame::text_width(kFontBig, "21"));
    CHECK(OledFrame::text_width(kFontHuge, "100%") <= 128);           // fits the screen
    CHECK(OledFrame::text_width(kFontBig, "UNLOADING") <= 126 || OledFrame::text_width(kFontMedium, "UNLOADING") <= 126);

    DisplayState s;
    s.link = DisplayState::Link::Ok;
    s.have_status = true;
    s.st.state = "standby";
    s.st.ext_temp = 215; s.st.ext_target = 215; s.st.bed_temp = 60; s.st.bed_target = 60;
    render_display(f, s, 0);
    int idle = lit(f, 0, 0, 128, 64);
    CHECK(idle > 300 && idle < 4000);                                 // numbers, not a full panel

    s.activity = "LOADING";                                           // 2 s temps, 2 s message
    render_display(f, s, 500);
    CHECK(lit(f, 0, 0, 128, 64) == idle);
    render_display(f, s, 2500);
    CHECK(lit(f, 0, 0, 128, 64) > 6000);                              // inverted message panel
    render_display(f, s, 4500);
    CHECK(lit(f, 0, 0, 128, 64) == idle);
    s.activity.clear();

    s.st.state = "printing";                                          // bar fill follows progress
    s.st.progress = 0.25;
    render_display(f, s, 0);
    int quarter = lit(f, 2, 46, 124, 16);
    s.st.progress = 0.75;
    render_display(f, s, 0);
    int three_q = lit(f, 2, 46, 124, 16);
    CHECK(quarter > 400 && quarter < 560 && three_q > 1400 && three_q < 1560);

    OledFrame g(32);                                                  // 128x32 also renders
    s.st.state = "standby";
    render_display(g, s, 0);
    CHECK(lit(g, 0, 0, 128, 32) > 200);

    std::string l1, l2, l3;
    wrap_small("no answer from Moonraker (check address, port and that it is running)", l1, l2, l3);
    CHECK(!l1.empty() && !l2.empty() && OledFrame::text_width(kFontSmall, l1) <= 126);

    DisplayState nb;                                                  // custom heading
    nb.link = DisplayState::Link::NoKlipper;
    nb.title = "NO BOARD";
    OledFrame a1(64), a2(64);
    render_display(a1, nb, 0);
    nb.title.clear();
    render_display(a2, nb, 0);
    CHECK(lit(a1, 0, 0, 128, 18) != lit(a2, 0, 0, 128, 18));

    uint8_t seq[64];
    CHECK(oled_init_sequence(false, 32, false, seq, 64) > 20 && seq[0] == 0xAE);

    ActivityTracker a;
    a.set("X", 1000, 500);
    CHECK(a.active(1200) && !a.active(1600));
}

int main() {
    test_oled();
    if (failures == 0) printf("all OLED tests passed\n");
    return failures ? 1 : 0;
}
