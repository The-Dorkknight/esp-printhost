// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/board_session.h"

namespace ph {

namespace {
const int SILENT_IDENTIFIES = 5;   // ~6 s of no answer before giving up one attempt
const int HEALTH_MISSES = 2;       // get_config failing twice in a row (~3 s): the board is gone
const int MAX_RESETS = 3;          // resets in a row that did not take effect
}  // namespace

BoardSession::BoardSession(KLink& link, SessionIO io, MachineIO base)
    : link_(link), io_(std::move(io)), base_(std::move(base)), board_(link) {}

BoardSession::~BoardSession() {
    link_.set_message_handler(nullptr);
}

void BoardSession::set_error(const std::string& e) {
    if (e != last_error_) log(e);   // log each new problem once, not every attempt
    last_error_ = e;
}

void BoardSession::tx(bool drive) {
    if (drive == tx_on_) return;
    tx_on_ = drive;
    if (io_.tx_drive) io_.tx_drive(drive);
}

void BoardSession::step() {
    if (state_ == State::Running) run_step();
    else connect();
}

void BoardSession::connect() {
    attempts_++;
    link_.set_message_handler(nullptr);
    // Listen before talking: a board that is off (or hung with its TX low) holds our RX line low.
    // Driving our TX high into it would half-power it through its RX pin.
    if (io_.rx_line && io_.rx_line() == 0) {
        tx(false);
        set_error("the board's TX line is low: the board is switched off, still starting, or hung. "
                  "If it stays like this, switch the printer's supply off and unplug the board's USB "
                  "for 10 seconds (the ESP32 does not drive its TX wire while waiting).");
        io_.sleep_ms(1000);
        return;
    }
    tx(true);
    if (io_.flush_input) io_.flush_input();
    link_.clear_rx_stats();
    link_.send_sync();
    KInfo ki;
    std::string err;
    if (!link_.identify(ki, &err, SILENT_IDENTIFIES)) {
        if (link_.rx_bytes() > 0 && link_.rx_nonzero() == 0)
            err = "only 00 bytes from the board: its TX line is held low (board off, browning out or hung)";
        else if (link_.rx_bytes() > 0)
            err += " (bytes arrive but are not valid replies: check baud rate and GND)";
        if (ever_ok_) link_.blind_reset();   // alive but stuck: this brings it back; harmless otherwise
        tx(false);
        set_error("no printer board: " + err);
        io_.sleep_ms(3000);
        return;
    }
    ever_ok_ = true;
    info_ = ki;
    log("printer board found: Klipper " + ki.version + " on " + ki.mcu);
    std::string text = io_.config_text();
    reload_ = false;
    plan_.reset(new BoardPlan(build_board_plan(text, link_.proto(), ki.clock_freq)));
    for (auto& e : plan_->errors) log("printer.cfg: " + e);

    bool c = false, s = false;
    uint32_t crc = 0, mc = 0;
    if (!link_.get_config(c, s, &err, &crc, &mc)) {
        last_error_ = "get_config: " + err;
        io_.sleep_ms(1000);
        return;
    }
    if (c || s) {
        // Configured from before (the ESP32 restarted, printer.cfg changed, FIRMWARE_RESTART) or shut
        // down: Klipper boards take a new configuration only after a restart. Always restart it, so
        // the printer starts from a known state with every output off.
        if (resets_ >= MAX_RESETS) {
            set_error("the printer board does not restart when told to (it ignores \"reset\"). "
                      "Switch the printer's supply off and unplug the board's USB for 10 seconds.");
            resets_ = MAX_RESETS - 1;   // try once more after the pause
            tx(false);
            io_.sleep_ms(5000);
            return;
        }
        log(s ? "the board is shut down from before: restarting it"
              : "the board still has a configuration from before: restarting it for a clean start");
        link_.blind_reset();
        resets_++;
        last_error_ = "restarting the printer board";
        io_.sleep_ms(1500);   // an RP2040 is back in well under a second
        return;
    }
    resets_ = 0;
    b_cfg_ = b_sd_ = false;
    cfg_ok_ = false;
    if (!plan_->errors.empty()) {
        cfg_msg_ = "printer.cfg has errors (nothing sent to the board)";
    } else if (plan_->cmds.empty()) {
        cfg_msg_ = "printer.cfg has nothing for the board yet";
    } else {
        auto res = board_.configure(plan_->cmds, plan_->oid_count, &cfg_msg_);
        log("board config: " + cfg_msg_);
        if (res != KBoard::Result::Configured && res != KBoard::Result::AlreadyConfigured) {
            last_error_ = "configuring the board: " + cfg_msg_;
            io_.sleep_ms(1000);
            return;
        }
        cfg_ok_ = true;
        b_cfg_ = true;
        for (auto& n : plan_->neopixels)   // initial colours, as Klipper sends at connect
            if (!board_.neopixel_write(n.oid, n.data, &err)) log("LED " + n.name + ": " + err);
    }
    uint64_t c64;
    double at;
    if (link_.get_uptime(c64, at, &err)) clk_.reset(ki.clock_freq, c64, at);
    else clk_.reset(ki.clock_freq, 0, link_.host_time_s());
    // Klipper boards report their move queue size only once configured (it is set up by
    // finalize_config; before that get_config says 0), as Klipper's own host reads it: ask again now.
    // 2026-10-28, Tina 2: read only before configuring, it was 0, the planner assumed a 200-command
    // queue (the RP2040 has 1024) and held moves back until they were too late to send.
    if (cfg_ok_) {
        bool c2 = false, s2 = false;
        uint32_t crc2 = 0, mc2 = 0;
        if (link_.get_config(c2, s2, &err, &crc2, &mc2) && mc2) mc = mc2;
    }
    if (!mc) log("the board did not report its move queue size: assuming 200 commands");
    else log("board move queue: " + std::to_string(mc) + " commands");
    plan_->move_count = int(mc);
    if (cfg_ok_) {
        MachineIO mio = base_;
        mio.send = [this](const std::vector<std::string>& c, std::string* e) { return link_.send_batch(c, e); };
        mio.query = [this](const std::string& c, const std::string& r, int oid, Message& out, int ms) {
            return link_.query(c, r, oid, out, ms);
        };
        mio.clock_at = [this](double t) { return clk_.clock_at(t); };
        mio.neopixel_write = [this](int oid, const std::vector<uint8_t>& d, std::string* e) {
            return board_.neopixel_write(oid, d, e);
        };
        mio.now = [this]() { return link_.host_time_s(); };
        machine_.reset(new Machine(*plan_, link_.proto(), mio));
        link_.set_message_handler([this](const Message& m) {
            if (machine_) machine_->on_message(m, link_.host_time_s());
        });
        log("machine: " + machine_->start(link_.host_time_s()));
    }
    state_ = State::Running;
    connects_++;
    misses_ = 0;
    last_check_ = link_.host_time_s();
    last_error_.clear();
    link_.clear_rx_stats();
    if (on_start) on_start(*this);
}

void BoardSession::run_step() {
    double now = link_.host_time_s();
    if (machine_) machine_->tick(now);
    link_.poll(20);
    if (link_.line_held_low()) {
        lost("the board's TX line went low (board switched off or restarting)", true);
        return;
    }
    if (machine_ && machine_->restart_requested()) {   // FIRMWARE_RESTART
        stop_running("FIRMWARE_RESTART");
        link_.blind_reset();
        io_.sleep_ms(1500);
        return;
    }
    if (reload_) {
        if (machine_) machine_->all_off(link_.host_time_s());   // nothing stays on across a reconfigure
        stop_running("printer.cfg changed: configuring the board again");
        return;
    }
    now = link_.host_time_s();
    if (now - last_check_ < 1.0) return;
    last_check_ = now;
    std::string err;
    uint32_t c32;
    double s0, s1;
    if (link_.get_clock(c32, s0, s1, &err)) clk_.add_sample(s0, s1, c32);
    bool c = false, s = false;
    uint32_t crc = 0;
    if (!link_.get_config(c, s, &err, &crc)) {
        if (link_.line_held_low()) {
            lost("the board's TX line went low (board switched off or restarting)", true);
        } else if (++misses_ >= HEALTH_MISSES) {
            lost("the printer board stopped answering", false);
        }
        return;
    }
    misses_ = 0;
    b_cfg_ = c;
    b_sd_ = s;
    if (cfg_ok_ && (!c || crc != board_.crc())) {
        // It forgot its configuration: it restarted (power cycle, reset button, watchdog). Every
        // output on it is off now; configure it again from scratch.
        stop_running("the printer board restarted by itself (power cycle or reset): configuring it again");
        return;
    }
    if (s && machine_ && !machine_->is_shutdown()) machine_->note_board_shutdown("the board reports it is shut down");
}

void BoardSession::stop_running(const std::string& why) {
    log(why);
    link_.set_message_handler(nullptr);
    if (machine_ && on_stop) on_stop(*machine_);
    machine_.reset();
    state_ = State::Searching;
    cfg_ok_ = false;
    last_error_ = why;
}

void BoardSession::lost(const std::string& why, bool line_low) {
    std::string w = why;
    if (machine_ && machine_->is_shutdown())
        w += " (after it shut down: " + machine_->shutdown_reason().substr(0, 80) + ")";
    stop_running(w);
    // Still powered but out of step: a reset brings it back. Not into a line that is held low: the
    // board is off, and driving our TX would only feed it through its RX pin.
    if (!line_low) link_.blind_reset();
    tx(false);
    io_.sleep_ms(2000);
}

}  // namespace ph
