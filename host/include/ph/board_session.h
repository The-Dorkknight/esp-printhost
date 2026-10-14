// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// The whole life of the connection to the printer board, shared by the ESP32-S3 firmware and the PC
// test tool (klink_probe --session), so the PC tests run exactly what the ESP32 runs:
//
//   searching: listen first (the board's TX line must be idle-high, i.e. the board is powered and its
//              firmware is running), then identify. A board that still has a configuration from
//              before, or is shut down, is reset so it starts clean. Then printer.cfg is sent and the
//              printer objects (ph::Machine) start.
//   running:   machine ticks, clock samples and a get_config health check every second.
//              The board restarted by itself (power cycle, reset button)  -> configure it again.
//              The line went low / no answers                              -> searching.
//              printer.cfg changed, FIRMWARE_RESTART                       -> reset, configure again.
//
// While searching the ESP32 lets go of its TX wire (when the platform can, SessionIO::tx_drive): a
// board whose own supply is switched off is otherwise half-powered through its RX pin, and may then
// not restart properly when its supply comes back (2026-10-14: the SKR Pico sent only 00 bytes after
// being power-cycled with the ESP32 running).
#pragma once
#include "ph/boardcfg.h"
#include "ph/clocksync.h"
#include "ph/kboard.h"
#include "ph/klink.h"
#include "ph/machine.h"

#include <functional>
#include <memory>
#include <string>

namespace ph {

struct SessionIO {
    std::function<void(int ms)> sleep_ms;                    // required
    std::function<std::string()> config_text;                // required: the current printer.cfg
    std::function<void(const std::string&)> log;             // optional
    std::function<void(bool drive)> tx_drive;                // optional: false = release the TX wire
    // Optional: the board's TX line (our RX) while idle: 1 high (board up), 0 held low (board off or
    // hung), -1 unknown. Without it the session always tries to talk.
    std::function<int()> rx_line;
    std::function<void()> flush_input;                       // optional: drop buffered UART input
};

class BoardSession {
public:
    enum class State { Searching, Running };

    // `base` supplies the platform parts of MachineIO (log, config_text, save_config, print_*,
    // largest_free); the session fills in send, query, clock_at, neopixel_write and now.
    BoardSession(KLink& link, SessionIO io, MachineIO base);
    ~BoardSession();

    // One unit of work: a connection attempt (may take a few seconds) or one run step (~20 ms).
    void step();

    State state() const { return state_; }
    bool running() const { return state_ == State::Running; }
    Machine* machine() { return machine_.get(); }
    const BoardPlan* plan() const { return plan_.get(); }
    const KInfo& info() const { return info_; }
    const ClockSync& clock() const { return clk_; }
    bool cfg_ok() const { return cfg_ok_; }
    const std::string& cfg_msg() const { return cfg_msg_; }
    const std::string& last_error() const { return last_error_; }   // why it is not running
    uint32_t attempts() const { return attempts_; }
    uint32_t config_crc() const { return board_.crc(); }
    bool board_configured() const { return b_cfg_; }
    bool board_shutdown() const { return b_sd_; }
    uint32_t connects() const { return connects_; }

    void reload() { reload_ = true; }                         // printer.cfg changed
    // Called just before the machine is destroyed (e.g. to keep its heater log).
    std::function<void(Machine&)> on_stop;
    // Called after a successful connection.
    std::function<void(BoardSession&)> on_start;

private:
    void connect();
    void run_step();
    void stop_running(const std::string& why);
    void lost(const std::string& why, bool line_low);
    void tx(bool drive);
    void set_error(const std::string& e);
    void log(const std::string& s) { if (io_.log) io_.log(s); }

    KLink& link_;
    SessionIO io_;
    MachineIO base_;
    KBoard board_;
    ClockSync clk_;
    KInfo info_;
    std::unique_ptr<BoardPlan> plan_;
    std::unique_ptr<Machine> machine_;
    State state_ = State::Searching;
    bool cfg_ok_ = false, ever_ok_ = false, reload_ = false, tx_on_ = true;
    bool b_cfg_ = false, b_sd_ = false;
    std::string cfg_msg_, last_error_ = "searching";
    uint32_t attempts_ = 0, connects_ = 0;
    int resets_ = 0, misses_ = 0;
    double last_check_ = 0;
};

}  // namespace ph
