// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// PC test driver for ph::KLink (the same code the ESP32-S3 runs over UART).
//   klink_probe /dev/pts/N [baud] [--neopixel PIN ORDER R,G,B] [--clock SECONDS]
//               [--config printer.cfg [--gcode "SET_LED LED=x RED=1"]]
//               [--machine printer.cfg [--at SECONDS "G-CODE"]... [--for SECONDS] [--no-off]]
//   klink_probe /dev/pts/N [baud] --session printer.cfg [--for SECONDS] [--at SECONDS "G-CODE"]...
// --session runs ph::BoardSession, the ESP32's whole connection logic (search, reset, configure, run,
// recover), for --for seconds and prints every state change. Its TX "release" drops writes, and it
// counts any write made while released (there should be none).
// --machine runs ph::Machine (heaters, fans, TMC, tests) like the ESP32 does, sends each --at command
// at its time, runs for --for seconds, prints the replies, test results and status JSON, then switches
// everything off (unless --no-off: then it just exits, like an ESP32 that died).
#include "ph/board_session.h"
#include "ph/boardcfg.h"
#include "ph/clocksync.h"
#include "ph/kboard.h"
#include "ph/klink.h"
#include "ph/machine.h"

#include <asm/termbits.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <zlib.h>

using namespace ph;

static int open_serial(const char* dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(dev); return -1; }
    struct termios2 t{};
    if (ioctl(fd, TCGETS2, &t) < 0) { perror("TCGETS2"); return -1; }
    t.c_cflag = CS8 | CREAD | CLOCAL | BOTHER;
    t.c_iflag = t.c_oflag = t.c_lflag = 0;
    t.c_ispeed = t.c_ospeed = baud;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (ioctl(fd, TCSETS2, &t) < 0) { perror("TCSETS2"); return -1; }
    return fd;
}

static bool inflate_all(const Bytes& in, std::string& out) {
    z_stream z{};
    if (inflateInit(&z) != Z_OK) return false;
    z.next_in = const_cast<Bytef*>(in.data());
    z.avail_in = uInt(in.size());
    char buf[4096];
    int r;
    do {
        z.next_out = reinterpret_cast<Bytef*>(buf);
        z.avail_out = sizeof buf;
        r = inflate(&z, Z_NO_FLUSH);
        if (r != Z_OK && r != Z_STREAM_END) { inflateEnd(&z); return false; }
        out.append(buf, sizeof buf - z.avail_out);
    } while (r != Z_STREAM_END);
    inflateEnd(&z);
    return true;
}

static bool g_tx_released = false;
static unsigned g_writes_released = 0;

static int run_session(KLink& link, int fd, int argc, char** argv, int i);

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: klink_probe DEVICE [baud]\n"); return 2; }
    int fd = open_serial(argv[1], argc > 2 && argv[2][0] != '-' ? atoi(argv[2]) : 250000);
    if (fd < 0) return 1;
    auto t0 = std::chrono::steady_clock::now();
    KLinkIO io;
    io.write = [fd](const uint8_t* p, size_t n) {
        if (g_tx_released) {   // like the ESP32 with its TX pin released: nothing goes out
            g_writes_released++;
            return true;
        }
        return write(fd, p, n) == ssize_t(n);
    };
    io.read = [fd](uint8_t* p, size_t n, int ms) -> size_t {
        pollfd pf{fd, POLLIN, 0};
        if (poll(&pf, 1, ms) <= 0) return 0;
        ssize_t r = read(fd, p, n);
        return r > 0 ? size_t(r) : 0;
    };
    io.now_ms = [t0]() {
        return uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count());
    };
    io.inflate = inflate_all;
    io.now_us = [t0]() {
        return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - t0).count());
    };
    io.log = [](const std::string& s) { fprintf(stderr, "%s\n", s.c_str()); };
    KLink link(io);
    for (int i = 2; i + 1 < argc; i++)
        if (std::string(argv[i]) == "--session") return run_session(link, fd, argc, argv, i);
    KInfo info;
    std::string err;
    if (!link.identify(info, &err)) { printf("IDENTIFY FAILED: %s\n", err.c_str()); return 1; }
    printf("version:    %s\nMCU:        %s\nCLOCK_FREQ: %.0f\ndictionary: %zu bytes compressed, %zu JSON\npins:       %s\n",
           info.version.c_str(), info.mcu.c_str(), info.clock_freq, info.dict_compressed, info.dict_json,
           info.pins.c_str());
    for (int i = 3; i < argc; i++) {
        if (std::string(argv[i]) != "--blind-reset") continue;
        printf("blind reset: %s\n", link.blind_reset() ? "sent" : "FAILED");
        usleep(300000);
        if (!link.identify(info, &err)) { printf("re-identify failed: %s\n", err.c_str()); return 1; }
        printf("identified again after the reset\n");
    }
    bool cfg = false, sd = false;
    if (!link.get_config(cfg, sd, &err)) { printf("GET_CONFIG FAILED: %s\n", err.c_str()); return 1; }
    printf("configured: %s  shutdown: %s\ncrc errors: %u\n", cfg ? "yes" : "no", sd ? "yes" : "no",
           link.crc_errors());
    // Optional: configure the board's NeoPixel and set a colour, like Klipper's [neopixel] + SET_LED.
    for (int i = 3; i + 3 < argc; i++) {
        if (std::string(argv[i]) != "--neopixel") continue;
        std::string pin = argv[i + 1], order = argv[i + 2];
        int r = 0, g = 0, b = 0;
        if (sscanf(argv[i + 3], "%d,%d,%d", &r, &g, &b) != 3) { printf("bad colour\n"); return 2; }
        KBoard board(link);
        for (int round = 0; round < 3; round++) {
            std::string msg;
            auto res = board.configure({neopixel_config_cmd(0, pin, int(order.size()), info.clock_freq)}, 1, &msg);
            printf("configure: %s (crc %u)\n", msg.c_str(), board.crc());
            if (res == KBoard::Result::Error) return 1;
            if (res != KBoard::Result::ResetSent) {
                if (!board.neopixel_write(0, led_bytes(order, uint8_t(r), uint8_t(g), uint8_t(b)), &err)) {
                    printf("NEOPIXEL FAILED: %s\n", err.c_str());
                    return 1;
                }
                printf("neopixel set %d,%d,%d\n", r, g, b);
                break;
            }
            usleep(300000);   // the board reboots after reset
            if (!link.identify(info, &err)) { printf("re-identify failed: %s\n", err.c_str()); return 1; }
        }
    }
    for (int i = 3; i + 1 < argc; i++) {
        if (std::string(argv[i]) != "--clock") continue;
        double secs = atof(argv[i + 1]);
        uint64_t c64;
        double at;
        if (!link.get_uptime(c64, at, &err)) { printf("UPTIME FAILED: %s\n", err.c_str()); return 1; }
        ClockSync cs;
        cs.reset(info.clock_freq, c64, at);
        double end = link.host_time_s() + secs;
        while (link.host_time_s() < end) {
            uint32_t c;
            double s0, s1;
            if (!link.get_clock(c, s0, s1, &err)) { printf("GET_CLOCK FAILED: %s\n", err.c_str()); return 1; }
            cs.add_sample(s0, s1, c);
            usleep(50000);
        }
        printf("clock: %.0f Hz measured, drift %.1f ppm, min rtt %.2f ms, %u samples, %u rejected\n",
               cs.frequency(), cs.drift_ppm(), cs.min_rtt() * 1e3, cs.samples(), cs.rejected());
    }
    for (int i = 3; i + 1 < argc; i++) {
        if (std::string(argv[i]) != "--config") continue;
        std::ifstream f(argv[i + 1]);
        std::stringstream ss;
        ss << f.rdbuf();
        BoardPlan plan = build_board_plan(ss.str(), link.proto(), info.clock_freq);
        for (auto& e : plan.errors) printf("config error: %s\n", e.c_str());
        for (auto& n : plan.notes) printf("config note: %s\n", n.c_str());
        if (!plan.errors.empty()) return 1;
        KBoard board(link);
        for (int round = 0; round < 3; round++) {
            std::string msg;
            auto res = board.configure(plan.cmds, plan.oid_count, &msg);
            printf("configure: %s\n", msg.c_str());
            if (res == KBoard::Result::Error) return 1;
            if (res != KBoard::Result::ResetSent) break;
            usleep(300000);
            if (!link.identify(info, &err)) { printf("re-identify failed: %s\n", err.c_str()); return 1; }
        }
        for (auto& n : plan.neopixels)   // initial colours, as Klipper sends at connect
            if (!board.neopixel_write(n.oid, n.data, &err)) { printf("initial LED: %s\n", err.c_str()); return 1; }
        if (!plan.sensors.empty()) {   // start the ADC reports (Klipper's init commands) and listen
            uint64_t c64;
            double at;
            if (!link.get_uptime(c64, at, &err)) { printf("GET_UPTIME FAILED: %s\n", err.c_str()); return 1; }
            uint32_t start = uint32_t(c64 + uint64_t(0.2 * info.clock_freq));
            link.set_message_handler([&](const Message& m) { apply_adc_message(plan, m); });
            for (auto& q : adc_query_cmds(plan, start, info.clock_freq))
                if (!link.command(q, nullptr, 300, &err)) { printf("query_analog_in: %s\n", err.c_str()); return 1; }
            auto t0 = std::chrono::steady_clock::now();
            auto waiting = [&]() {
                for (auto& s : plan.sensors) if (s.updates == 0) return true;
                return false;
            };
            while (waiting() && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(3)) link.poll(50);
            for (auto& s : plan.sensors) printf("temp %s: %s (%u reports)\n", s.name.c_str(), s.describe().c_str(), s.updates);
        }
        for (int j = 3; j + 1 < argc; j++)
            if (std::string(argv[j]) == "--gcode")
                printf("gcode: %s -> %s\n", argv[j + 1], run_console_command(plan, board, argv[j + 1]).c_str());
    }
    for (int i = 3; i + 1 < argc; i++) {
        if (std::string(argv[i]) != "--machine") continue;
        std::ifstream f(argv[i + 1]);
        std::stringstream ss;
        ss << f.rdbuf();
        BoardPlan plan = build_board_plan(ss.str(), link.proto(), info.clock_freq);
        for (auto& e : plan.errors) printf("config error: %s\n", e.c_str());
        for (auto& n : plan.notes) printf("config note: %s\n", n.c_str());
        if (!plan.errors.empty()) return 1;
        KBoard board(link);
        for (int round = 0; round < 3; round++) {
            std::string msg;
            auto res = board.configure(plan.cmds, plan.oid_count, &msg);
            printf("configure: %s\n", msg.c_str());
            if (res == KBoard::Result::Error) return 1;
            if (res != KBoard::Result::ResetSent) break;
            usleep(300000);
            if (!link.identify(info, &err)) { printf("re-identify failed: %s\n", err.c_str()); return 1; }
        }
        uint64_t c64;
        double at;
        if (!link.get_uptime(c64, at, &err)) { printf("GET_UPTIME FAILED: %s\n", err.c_str()); return 1; }
        ClockSync clk;
        clk.reset(info.clock_freq, c64, at);
        MachineIO mio;
        mio.send = [&](const std::vector<std::string>& c, std::string* e) { return link.send_batch(c, e); };
        mio.query = [&](const std::string& c, const std::string& r, int oid, Message& out, int ms) {
            return link.query(c, r, oid, out, ms);
        };
        mio.clock_at = [&](double t) { return clk.clock_at(t); };
        mio.neopixel_write = [&](int oid, const std::vector<uint8_t>& d, std::string* e) { return board.neopixel_write(oid, d, e); };
        mio.log = [](const std::string& s) { printf("machine: %s\n", s.c_str()); };
        mio.now = [&]() { return link.host_time_s(); };
        {   // the board's move queue size, for flow control
            bool c1 = false, s1 = false;
            uint32_t crc = 0, mc = 0;
            if (link.get_config(c1, s1, &err, &crc, &mc)) plan.move_count = int(mc ? mc : 1024);
        }
        std::string print_path;
        for (int j = 3; j + 1 < argc; j++)
            if (std::string(argv[j]) == "--print") print_path = argv[j + 1];
        std::ifstream pf;
        long pf_size = 1, pf_pos = 0;
        mio.print_open = [&](std::string& name, std::string* e) {
            if (print_path.empty()) { if (e) *e = "no --print file"; return false; }
            pf.close();
            pf.clear();
            pf.open(print_path);
            pf.seekg(0, std::ios::end);
            pf_size = long(pf.tellg());
            pf.seekg(0);
            pf_pos = 0;
            name = print_path.substr(print_path.find_last_of('/') + 1);
            return bool(pf);
        };
        mio.print_read = [&](std::string& line, bool& eof, std::string*) {
            eof = !std::getline(pf, line);
            if (!eof) pf_pos += long(line.size()) + 1;
            return true;
        };
        mio.print_progress = [&]() { return pf_size > 0 ? double(pf_pos) / double(pf_size) : 0.0; };
        Machine m(plan, link.proto(), mio);
        link.set_message_handler([&](const Message& msg) {
            if (getenv("PROBE_TRACE") && msg.name != "analog_in_state") printf("msg: %s oid=%lld\n", msg.name.c_str(), (long long)msg.get("oid"));
            m.on_message(msg, link.host_time_s());
        });
        printf("machine: %s\n", m.start(link.host_time_s()).c_str());
        double run_for = 5;
        bool no_off = false;
        std::vector<std::pair<double, std::string>> at_cmds;
        for (int j = 3; j < argc; j++) {
            std::string a = argv[j];
            if (a == "--for" && j + 1 < argc) run_for = atof(argv[j + 1]);
            if (a == "--no-off") no_off = true;
            if (a == "--at" && j + 2 < argc) at_cmds.push_back({atof(argv[j + 1]), argv[j + 2]});
        }
        double t_start = link.host_time_s(), last_clock = 0;
        size_t next = 0;
        while (link.host_time_s() - t_start < run_for) {
            double now = link.host_time_s();
            if (now - last_clock >= 1.0) {   // clock sync, as the ESP32 does
                last_clock = now;
                uint32_t c;
                double s0, s1;
                if (link.get_clock(c, s0, s1, &err)) clk.add_sample(s0, s1, c);
            }
            while (next < at_cmds.size() && now - t_start >= at_cmds[next].first) {
                printf("gcode: %s -> %s\n", at_cmds[next].second.c_str(), m.command(at_cmds[next].second, now).c_str());
                next++;
            }
            link.poll(20);
            m.tick(link.host_time_s());
        }
        for (auto& t : m.tests()) printf("test %s: %s %s\n", t.id.c_str(), t.state.c_str(), t.detail.c_str());
        printf("status: %s\n", m.status_json(link.host_time_s()).c_str());
        printf("shutdown: %s\n", m.is_shutdown() ? m.shutdown_reason().c_str() : "no");
        if (no_off) {
            printf("exiting without switching anything off\n");
            return 0;
        }
        m.all_off(link.host_time_s());
    }
    printf("KLINK OK\n");
    close(fd);
    return 0;
}

static int run_session(KLink& link, int fd, int argc, char** argv, int i) {
    std::ifstream f(argv[i + 1]);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string cfg = ss.str();
    double run_for = 10;
    std::vector<std::pair<double, std::string>> at_cmds;
    for (int j = 2; j < argc; j++) {
        std::string a = argv[j];
        if (a == "--for" && j + 1 < argc) run_for = atof(argv[j + 1]);
        if (a == "--at" && j + 2 < argc) at_cmds.push_back({atof(argv[j + 1]), argv[j + 2]});
    }
    SessionIO sio;
    sio.sleep_ms = [](int ms) { usleep(useconds_t(ms) * 1000); };
    sio.config_text = [&cfg]() { return cfg; };
    sio.log = [&link](const std::string& s) { printf("[%7.2f] session: %s\n", link.host_time_s(), s.c_str()); fflush(stdout); };
    sio.tx_drive = [&link](bool on) {
        g_tx_released = !on;
        printf("[%7.2f] tx %s\n", link.host_time_s(), on ? "driven" : "released");
        fflush(stdout);
    };
    // The PC cannot read the line level; a line held low arrives as 00 bytes (the fake board sends
    // them while "off" or hung), an idle line as nothing.
    sio.rx_line = [fd]() {
        uint8_t b[256];
        size_t n = 0, nz = 0;
        for (int k = 0; k < 6; k++) {
            pollfd pf{fd, POLLIN, 0};
            if (poll(&pf, 1, 10) <= 0) continue;
            ssize_t r = read(fd, b, sizeof b);
            for (ssize_t q = 0; q < r; q++) nz += b[q] != 0;
            if (r > 0) n += size_t(r);
        }
        return n > 0 && nz == 0 ? 0 : 1;
    };
    sio.flush_input = [fd]() {
        uint8_t b[256];
        while (read(fd, b, sizeof b) > 0) {}
    };
    MachineIO base;
    base.log = [&link](const std::string& s) { printf("[%7.2f] machine: %s\n", link.host_time_s(), s.c_str()); };
    base.config_text = [&cfg]() { return cfg; };
    BoardSession bs(link, sio, base);
    double t_start = link.host_time_s();
    size_t next = 0;
    bool was_running = false;
    while (link.host_time_s() - t_start < run_for) {
        bs.step();
        double now = link.host_time_s();
        if (bs.running() != was_running) {
            was_running = bs.running();
            printf("[%7.2f] STATE %s%s\n", now, was_running ? "running" : "searching: ",
                   was_running ? (bs.cfg_ok() ? " (configured)" : " (not configured)") : bs.last_error().c_str());
            fflush(stdout);
        }
        while (next < at_cmds.size() && now - t_start >= at_cmds[next].first) {
            std::string r = bs.machine() ? bs.machine()->command(at_cmds[next].second, now) : "error: not running";
            printf("[%7.2f] gcode: %s -> %s\n", now, at_cmds[next].second.c_str(), r.c_str());
            next++;
        }
    }
    printf("connects: %u\nfinal state: %s\nlast error: %s\nwrites while TX released: %u\n", bs.connects(),
           bs.running() ? "running" : "searching", bs.last_error().c_str(), g_writes_released);
    if (bs.machine()) {
        const BoardPlan* p = bs.plan();
        for (auto& s : p->sensors) printf("temp %s: %s (%u reports)\n", s.name.c_str(), s.describe().c_str(), s.updates);
        bs.machine()->all_off(link.host_time_s());
    }
    close(fd);
    return 0;
}
