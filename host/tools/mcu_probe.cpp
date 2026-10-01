// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// mcu_probe — PoC step 1 on a Linux PC, before any ESP32 work.
// Talks to a board flashed with stock Klipper MCU firmware using our own
// protocol code (ph_core), not Klipper's Python.
//
//   ./mcu_probe /dev/ttyUSB0 250000               # identify + uptime
//   ./mcu_probe /dev/ttyUSB0 250000 --blink PC13  # also toggle a pin 10x
//
// Stop Klipper/any other program using the port first. Power-cycle the board
// if it reports it is already configured.
#include "ph/msgproto.h"
#include "ph/wire.h"
#include <asm/termbits.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>
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
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    if (ioctl(fd, TCSETS2, &t) < 0) { perror("TCSETS2"); return -1; }
    ioctl(fd, TCFLSH, TCIOFLUSH);
    return fd;
}

struct Link {
    int fd;
    uint8_t seq = 0;
    FrameDecoder dec;
    MsgProto mp;

    // Send one block, return decoded response messages (waits up to ms).
    bool transact(const Bytes& content, std::vector<Message>& msgs, int ms = 300) {
        for (int attempt = 0; attempt < 4; attempt++) {
            Bytes f = frame_encode(content, seq);
            if (write(fd, f.data(), f.size()) != ssize_t(f.size())) return false;
            bool acked = false;
            auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
            while (std::chrono::steady_clock::now() < end) {
                pollfd p{fd, POLLIN, 0};
                if (poll(&p, 1, 20) <= 0) { if (acked) break; continue; }  // 20 ms idle after ack
                uint8_t buf[256];
                ssize_t n = read(fd, buf, sizeof buf);
                if (n <= 0) continue;
                std::vector<FrameDecoder::Block> blocks;
                dec.feed(buf, size_t(n), blocks);
                for (auto& b : blocks) {
                    uint8_t want = (seq + 1) & MSG_SEQ_MASK;
                    if (b.seq == want) acked = true;
                    else if (b.content.empty()) {
                        // NAK: MCU tells us which seq it expects. Adopt it.
                        seq = b.seq;
                        goto retry;
                    }
                    std::string err;
                    if (!b.content.empty() && !mp.decode(b.content, msgs, &err))
                        fprintf(stderr, "decode: %s\n", err.c_str());
                }
            }
            if (acked) { seq = (seq + 1) & MSG_SEQ_MASK; return true; }
        retry:;
        }
        return false;
    }
    bool cmd(const std::string& line, std::vector<Message>* out = nullptr) {
        Bytes c; std::string err;
        if (!mp.encode(line, c, &err)) { fprintf(stderr, "encode: %s\n", err.c_str()); return false; }
        std::vector<Message> tmp;
        std::vector<Message>& got = out ? *out : tmp;
        size_t before = got.size();
        bool ok = transact(c, got);
        if (!ok) fprintf(stderr, "no ack for: %s\n", line.c_str());
        for (size_t i = before; i < got.size(); i++) {
            auto& m = got[i];
            if (m.name == "shutdown" || m.name == "is_shutdown") {
                std::string why = mp.enum_name_of("static_string_id", m.get("static_string_id"));
                printf("MCU SHUTDOWN: %s\n", why.empty() ? "(unknown reason)" : why.c_str());
                shutdown = true;
            }
        }
        return ok && !shutdown;
    }
    bool shutdown = false;
};

static bool inflate_all(const Bytes& in, std::string& out) {
    z_stream z{};
    if (inflateInit(&z) != Z_OK) return false;
    z.next_in = const_cast<Bytef*>(in.data());
    z.avail_in = uInt(in.size());
    char buf[4096]; int r;
    do {
        z.next_out = reinterpret_cast<Bytef*>(buf); z.avail_out = sizeof buf;
        r = inflate(&z, Z_NO_FLUSH);
        if (r != Z_OK && r != Z_STREAM_END) { inflateEnd(&z); return false; }
        out.append(buf, sizeof buf - z.avail_out);
    } while (r != Z_STREAM_END);
    inflateEnd(&z);
    return true;
}

static void close_fd(int& fd) { if (fd >= 0) close(fd); fd = -1; }

// Open port, read the data dictionary. Returns false with a message on failure.
static bool connect(Link& L, const char* dev, int baud, bool quiet = false) {
    close_fd(L.fd);
    L.fd = open_serial(dev, baud);
    if (L.fd < 0) return false;
    L.seq = 0; L.dec = FrameDecoder(); L.shutdown = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    Bytes dict;
    for (int tries = 0; tries < 400; tries++) {
        std::vector<Message> msgs;
        L.cmd("identify offset=" + std::to_string(dict.size()) + " count=40", &msgs);
        bool got = false, done = false;
        for (auto& m : msgs) {
            if (m.name != "identify_response") continue;
            if (size_t(m.get("offset")) != dict.size()) continue;
            auto& d = m.params["data"].b;
            dict.insert(dict.end(), d.begin(), d.end());
            got = true; done = d.size() < 40;
        }
        if (done) break;
        if (!got && tries > 20 && dict.empty()) { fprintf(stderr, "no identify_response\n"); return false; }
    }
    std::string json, err;
    if (!inflate_all(dict, json)) { fprintf(stderr, "inflate failed\n"); return false; }
    L.mp = MsgProto();
    if (!L.mp.load_dictionary(json, &err)) { fprintf(stderr, "dict: %s\n", err.c_str()); return false; }
    if (!quiet) {
        printf("MCU:        %s\n", L.mp.config_str("MCU").c_str());
        printf("CLOCK_FREQ: %.0f\n", L.mp.config_num("CLOCK_FREQ"));
        printf("version:    %s\n", L.mp.version().c_str());
        printf("dictionary: %zu bytes compressed, %zu bytes JSON\n", dict.size(), json.size());
        printf("pins:       %s\n", L.mp.enum_summary("pin").c_str());
    }
    return true;
}

// Returns is_config; also reports shutdown state.
static bool query_config(Link& L, bool& is_shutdown) {
    std::vector<Message> msgs;
    bool configured = false;
    is_shutdown = false;
    L.shutdown = false;
    L.cmd("get_config", &msgs);
    for (auto& m : msgs)
        if (m.name == "config") {
            configured = m.get("is_config");
            is_shutdown = m.get("is_shutdown");
            printf("configured: %s  shutdown: %s\n", configured ? "yes" : "no",
                   is_shutdown ? "yes" : "no");
        }
    L.shutdown = false;
    return configured;
}

int main(int argc, char** argv) {
    setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffer so GUIs see output live
    if (argc < 3) {
        fprintf(stderr, "usage: %s DEV BAUD [--blink PIN] [--pins]\n", argv[0]);
        return 2;
    }
    const char* dev = argv[1];
    int baud = atoi(argv[2]);
    std::string blink_pin;
    bool list_pins = false;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--blink") && i + 1 < argc) blink_pin = argv[++i];
        else if (!strcmp(argv[i], "--pins")) list_pins = true;
        else { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
    }

    Link L;
    L.fd = -1;
    if (!connect(L, dev, baud)) return 1;

    if (list_pins) {
        auto keys = L.mp.enum_keys("pin");
        printf("all %zu pin names:\n", keys.size());
        std::string line;
        for (auto& k : keys) {
            if (line.size() + k.size() > 70) { printf("  %s\n", line.c_str()); line.clear(); }
            line += k + " ";
        }
        if (!line.empty()) printf("  %s\n", line.c_str());
    }

    // ---- uptime ----
    std::vector<Message> msgs;
    if (L.cmd("get_uptime", &msgs))
        for (auto& m : msgs)
            if (m.name == "uptime")
                printf("uptime:     %.3f s\n",
                       double((uint64_t(m.get("high")) << 32) | uint64_t(m.get("clock")))
                       / L.mp.config_num("CLOCK_FREQ", 1));

    bool is_shutdown = false;
    bool configured = query_config(L, is_shutdown);

    if (!blink_pin.empty()) {
        // 1. Check the pin name BEFORE touching the MCU.
        int64_t pin_id;
        if (!L.mp.enum_value("pin", blink_pin, pin_id)) {
            printf("ERROR: '%s' is not a pin on this MCU (%s).\n", blink_pin.c_str(),
                   L.mp.config_str("MCU").c_str());
            printf("valid pins: %s\n", L.mp.enum_summary("pin").c_str());
            return 1;
        }
        // 2. A configured or shut-down MCU must be reset before we can configure it.
        if (configured || is_shutdown) {
            if (!L.mp.has_command("reset")) {
                printf("ERROR: MCU is already configured/shut down and has no 'reset' "
                       "command. Power-cycle the board, then run again.\n");
                return 1;
            }
            printf("resetting MCU (it was %s)...\n", is_shutdown ? "shut down" : "already configured");
            Bytes c; L.mp.encode("reset", c);
            Bytes f = frame_encode(c, L.seq);
            if (write(L.fd, f.data(), f.size()) < 0) perror("write");
            close_fd(L.fd);
            bool back = false;
            for (int t = 0; t < 30 && !back; t++) {      // USB re-enumerates: ~1-3 s
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                if (access(dev, F_OK) != 0) continue;
                back = connect(L, dev, baud, true);
            }
            if (!back) { printf("ERROR: MCU did not come back after reset\n"); return 1; }
            printf("MCU back after reset\n");
            configured = query_config(L, is_shutdown);
            if (configured || is_shutdown) { printf("ERROR: still configured after reset\n"); return 1; }
        }
        // 3. Configure one digital output and toggle it.
        bool ok = L.cmd("allocate_oids count=1")
               && L.cmd("config_digital_out oid=0 pin=" + blink_pin +
                        " value=0 default_value=0 max_duration=0")
               && L.cmd("finalize_config crc=12345");
        if (!ok) { printf("ERROR: MCU rejected the pin configuration\n"); return 1; }
        printf("blinking %s (pin id %lld) 10x...\n", blink_pin.c_str(), (long long)pin_id);
        for (int i = 0; i < 10 && ok; i++) {
            ok = L.cmd(std::string("update_digital_out oid=0 value=") + (i & 1 ? "0" : "1"));
            if (ok) printf("%s %s\n", blink_pin.c_str(), i & 1 ? "off" : "on");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        if (!ok) { printf("ERROR: blink stopped (see above)\n"); return 1; }
        query_config(L, is_shutdown);
        printf("note: MCU is now configured; the next --blink run will reset it automatically\n");
    }
    printf("ok (crc errors: %u)\n", L.dec.crc_errors());
    return 0;
}
