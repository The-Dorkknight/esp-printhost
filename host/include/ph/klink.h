// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Talk to a Klipper MCU over any byte stream (PC serial port, ESP32 UART).
// Portable: all I/O, time and zlib inflate come in through KLinkIO callbacks.
#pragma once
#include "ph/msgproto.h"
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ph {

struct KLinkIO {
    // Write all bytes; false on error.
    std::function<bool(const uint8_t*, size_t)> write;
    // Read up to len bytes, waiting at most timeout_ms; returns bytes read (0 = timeout).
    std::function<size_t(uint8_t*, size_t, int)> read;
    // Monotonic milliseconds.
    std::function<uint32_t()> now_ms;
    // zlib-inflate the compressed data dictionary into JSON text.
    std::function<bool(const Bytes&, std::string&)> inflate;
    // Optional diagnostic log.
    std::function<void(const std::string&)> log;
    // Optional microsecond clock for clock sync (falls back to now_ms * 1000).
    std::function<uint64_t()> now_us;
};

struct KInfo {
    std::string version, build_versions, mcu, pins;
    double clock_freq = 0;
    size_t dict_compressed = 0, dict_json = 0, commands = 0;
};

class KLink {
public:
    explicit KLink(KLinkIO io) : io_(std::move(io)) {}

    // Read the MCU's data dictionary (Klipper "identify"). Fills info on success. Gives up after
    // `max_silent` identify commands without any answer (each waits up to 4 x 300 ms).
    bool identify(KInfo& info, std::string* err = nullptr, int max_silent = 15);
    // Send one text command (needs identify first) and collect the responses.
    bool command(const std::string& line, std::vector<Message>* out = nullptr,
                 int timeout_ms = 300, std::string* err = nullptr);
    // Klipper get_config: is the MCU configured / shut down?
    bool get_config(bool& configured, bool& shutdown, std::string* err = nullptr, uint32_t* crc = nullptr,
                    uint32_t* move_count = nullptr);
    // Board clock for clock sync: get_clock, timed from just before sending to the moment the
    // reply arrived (not the end of the quiet wait). Times in seconds of the host clock.
    bool get_clock(uint32_t& clock, double& sent_s, double& recv_s, std::string* err = nullptr);
    // Full 64-bit board clock (get_uptime).
    bool get_uptime(uint64_t& clock64, double& at_s, std::string* err = nullptr);
    double host_time_s() const;
    // Send several commands, packed into as few frames as fit (Klipper blocks are at most 64 bytes),
    // returning as soon as each frame is acknowledged. For scheduled commands (queue_step, ...) where
    // a 20 ms quiet wait per command would be too slow.
    bool send_batch(const std::vector<std::string>& lines, std::string* err = nullptr);
    // Send a command and wait up to timeout_ms for the response `resp` carrying this oid
    // (e.g. tmcuart_send -> tmcuart_response). The response is also passed to the handler.
    bool query(const std::string& line, const std::string& resp, int oid, Message& out, int timeout_ms = 200,
               std::string* err = nullptr);
    // Reset a board whose message sequence we may have lost: "reset" sent once with each of the 16
    // sequence numbers (the board acts on the one it expects and drops the rest). Klipper boards
    // answer "reset" even when shut down. Returns false only if the UART write failed.
    bool blind_reset();
    // Send a command that gets no answer (e.g. "reset": the board reboots at once).
    bool send_only(const std::string& line, std::string* err = nullptr);
    // A few SYNC bytes (0x7e). A Klipper board that saw line noise discards bytes until it sees one
    // (src/command.c "need_sync"); extra leading SYNC bytes are ignored without a NAK.
    bool send_sync();

    // Received-byte statistics since clear_rx_stats(): all bytes, bytes that were not 0x00, and the
    // current run of 0x00 bytes. A UART line held low (board switched off, or hung with its TX low)
    // reads as a stream of 0x00 bytes; valid Klipper traffic never has 64 in a row (every block is at
    // most 64 bytes and ends in 0x7e).
    void clear_rx_stats() { rx_bytes_ = rx_nonzero_ = zero_run_ = 0; }
    uint32_t rx_bytes() const { return rx_bytes_; }
    uint32_t rx_nonzero() const { return rx_nonzero_; }
    bool line_held_low() const { return zero_run_ >= 64; }

    // Messages the board sends on its own (e.g. analog_in_state): every decoded message, including
    // those that answer a command, is passed to this handler. It must outlive the calls, or be
    // cleared with set_message_handler(nullptr).
    void set_message_handler(std::function<void(const Message&)> h) { on_message_ = std::move(h); }
    // Wait up to timeout_ms for bytes and dispatch any messages in them. True if bytes arrived.
    bool poll(int timeout_ms);

    uint32_t crc_errors() const { return dec_.crc_errors(); }
    uint32_t resyncs() const { return dec_.resyncs(); }
    const MsgProto& proto() const { return mp_; }

private:
    bool identify_fresh(KInfo& info, std::string* err, int max_silent);
    void note_rx(const uint8_t* p, size_t n) {
        rx_bytes_ += uint32_t(n);
        for (size_t i = 0; i < n; i++) {
            if (p[i]) { rx_nonzero_++; zero_run_ = 0; }
            else zero_run_++;
        }
    }
    bool transact(const Bytes& content, std::vector<Message>& msgs, int timeout_ms, bool return_on_ack = false);
    void log(const std::string& s) const { if (io_.log) io_.log(s); }

    KLinkIO io_;
    std::function<void(const Message&)> on_message_;
    std::vector<Message>* capture_ = nullptr;   // query(): also keep messages that arrive in poll()
    uint8_t seq_ = 0;
    double last_rx_s_ = 0;   // host time the last block with content arrived
    uint32_t rx_bytes_ = 0, rx_nonzero_ = 0, zero_run_ = 0;
    FrameDecoder dec_;
    MsgProto mp_;
};

}  // namespace ph
