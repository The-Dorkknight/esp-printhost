// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/klink.h"

namespace ph {

bool KLink::transact(const Bytes& content, std::vector<Message>& msgs, int timeout_ms, bool return_on_ack) {
    for (int attempt = 0; attempt < 4; attempt++) {
        Bytes f = frame_encode(content, seq_);
        if (attempt > 0) f.insert(f.begin(), MSG_SYNC);   // a retry: resynchronise a board that saw noise
        if (!io_.write(f.data(), f.size())) return false;
        bool acked = false, nak = false;
        uint32_t start = io_.now_ms();
        while (uint32_t(io_.now_ms() - start) < uint32_t(timeout_ms)) {
            uint8_t buf[256];
            size_t n = io_.read(buf, sizeof buf, 20);
            note_rx(buf, n);
            if (n == 0) {
                if (acked) break;  // 20 ms quiet after the ack: responses are in
                continue;
            }
            std::vector<FrameDecoder::Block> blocks;
            dec_.feed(buf, n, blocks);
            for (auto& b : blocks) {
                uint8_t want = uint8_t((seq_ + 1) & MSG_SEQ_MASK);
                if (b.seq == want) {
                    acked = true;
                } else if (b.content.empty()) {
                    // NAK: the MCU says which sequence number it expects. Adopt it, resend.
                    seq_ = b.seq;
                    nak = true;
                    break;
                }
                std::string err;
                if (b.content.empty()) continue;
                last_rx_s_ = host_time_s();
                std::vector<Message> got_msgs;
                if (!mp_.decode(b.content, got_msgs, &err)) log("decode: " + err);
                for (auto& m : got_msgs) {
                    if (on_message_) on_message_(m);
                    msgs.push_back(std::move(m));
                }
            }
            if (nak || (acked && return_on_ack)) break;
        }
        if (acked) {
            seq_ = uint8_t((seq_ + 1) & MSG_SEQ_MASK);
            return true;
        }
    }
    return false;
}

bool KLink::poll(int timeout_ms) {
    uint8_t buf[256];
    size_t n = io_.read(buf, sizeof buf, timeout_ms);
    note_rx(buf, n);
    if (n == 0) return false;
    std::vector<FrameDecoder::Block> blocks;
    dec_.feed(buf, n, blocks);
    for (auto& b : blocks) {
        if (b.content.empty()) continue;   // acks and NAKs are only meaningful inside a transaction
        last_rx_s_ = host_time_s();
        std::vector<Message> got_msgs;
        std::string err;
        if (!mp_.decode(b.content, got_msgs, &err)) log("decode: " + err);
        for (auto& m : got_msgs) {
            if (on_message_) on_message_(m);
            if (capture_) capture_->push_back(m);
        }
    }
    return true;
}

bool KLink::send_batch(const std::vector<std::string>& lines, std::string* err) {
    const size_t MAX_CONTENT = 64 - 5;   // Klipper MESSAGE_MAX minus header and trailer
    Bytes block;
    std::vector<Message> msgs;
    auto flush = [&]() {
        if (block.empty()) return true;
        bool ok = transact(block, msgs, 300, true);
        block.clear();
        if (!ok && err) *err = "no acknowledgement from the board";
        return ok;
    };
    for (auto& l : lines) {
        Bytes c;
        std::string e;
        if (!mp_.encode(l, c, &e)) {
            if (err) *err = "encode: " + e;
            return false;
        }
        if (block.size() + c.size() > MAX_CONTENT && !flush()) return false;
        block.insert(block.end(), c.begin(), c.end());
    }
    return flush();
}

bool KLink::query(const std::string& line, const std::string& resp, int oid, Message& out, int timeout_ms,
                  std::string* err) {
    std::vector<Message> msgs;
    auto find = [&](std::vector<Message>& v) {
        for (auto& m : v)
            if (m.name == resp && (oid < 0 || m.get("oid") == oid)) {
                out = m;
                return true;
            }
        return false;
    };
    if (!command(line, &msgs, 50, err)) return false;
    if (find(msgs)) return true;
    uint32_t start = io_.now_ms();
    std::vector<Message> late;
    capture_ = &late;
    bool got = false;
    while (!got && uint32_t(io_.now_ms() - start) < uint32_t(timeout_ms)) {
        poll(10);
        got = find(late);
    }
    capture_ = nullptr;
    if (!got && err) *err = "no " + resp + " from the board";
    return got;
}

bool KLink::command(const std::string& line, std::vector<Message>* out, int timeout_ms, std::string* err) {
    Bytes c;
    std::string e;
    if (!mp_.encode(line, c, &e)) {
        if (err) *err = "encode: " + e;
        return false;
    }
    std::vector<Message> tmp;
    if (!transact(c, out ? *out : tmp, timeout_ms)) {
        if (err) *err = "no answer to: " + line;
        return false;
    }
    return true;
}

bool KLink::identify(KInfo& info, std::string* err, int max_silent) {
    MsgProto keep = mp_;   // the last good dictionary stays usable (blind_reset) if this fails
    bool ok = identify_fresh(info, err, max_silent);
    if (!ok) mp_ = keep;
    return ok;
}

bool KLink::identify_fresh(KInfo& info, std::string* err, int max_silent) {
    mp_ = MsgProto();
    Bytes dict;
    int empty_tries = 0;
    for (int tries = 0; tries < 600; tries++) {
        std::vector<Message> msgs;
        command("identify offset=" + std::to_string(dict.size()) + " count=40", &msgs, 300);
        bool got = false, done = false;
        for (auto& m : msgs) {
            if (m.name != "identify_response" || size_t(m.get("offset")) != dict.size()) continue;
            auto it = m.params.find("data");
            if (it == m.params.end()) continue;
            const Bytes& d = it->second.b;
            dict.insert(dict.end(), d.begin(), d.end());
            got = true;
            done = d.size() < 40;
        }
        if (done) break;
        if (!got && dict.empty() && ++empty_tries >= max_silent) {
            if (err) *err = "no identify_response (wrong wiring, baud, or the board is not running "
                            "Klipper serial firmware)";
            return false;
        }
    }
    if (dict.empty()) {
        if (err) *err = "no data dictionary received";
        return false;
    }
    std::string json, e;
    if (!io_.inflate(dict, json)) {
        if (err) *err = "could not unpack the data dictionary (" + std::to_string(dict.size()) + " bytes)";
        return false;
    }
    if (!mp_.load_dictionary(json, &e)) {
        if (err) *err = "bad data dictionary: " + e;
        return false;
    }
    info.version = mp_.version();
    info.mcu = mp_.config_str("MCU");
    info.clock_freq = mp_.config_num("CLOCK_FREQ");
    info.pins = mp_.enum_summary("pin");
    info.dict_compressed = dict.size();
    info.dict_json = json.size();
    info.commands = 0;
    return true;
}

double KLink::host_time_s() const {
    return io_.now_us ? double(io_.now_us()) / 1e6 : double(io_.now_ms()) / 1e3;
}

bool KLink::get_clock(uint32_t& clock, double& sent_s, double& recv_s, std::string* err) {
    std::vector<Message> msgs;
    last_rx_s_ = 0;
    sent_s = host_time_s();
    if (!command("get_clock", &msgs, 300, err)) return false;
    for (auto& m : msgs) {
        if (m.name == "clock") {
            clock = uint32_t(m.get("clock"));
            recv_s = last_rx_s_ > 0 ? last_rx_s_ : host_time_s();
            return true;
        }
    }
    if (err) *err = "no clock response";
    return false;
}

bool KLink::get_uptime(uint64_t& clock64, double& at_s, std::string* err) {
    std::vector<Message> msgs;
    double sent = host_time_s();
    if (!command("get_uptime", &msgs, 300, err)) return false;
    for (auto& m : msgs) {
        if (m.name == "uptime") {
            clock64 = (uint64_t(uint32_t(m.get("high"))) << 32) | uint32_t(m.get("clock"));
            at_s = (sent + (last_rx_s_ > 0 ? last_rx_s_ : host_time_s())) / 2;
            return true;
        }
    }
    if (err) *err = "no uptime response";
    return false;
}

bool KLink::send_only(const std::string& line, std::string* err) {
    Bytes c;
    std::string e;
    if (!mp_.encode(line, c, &e)) {
        if (err) *err = "encode: " + e;
        return false;
    }
    Bytes f = frame_encode(c, seq_);
    seq_ = uint8_t((seq_ + 1) & MSG_SEQ_MASK);
    return io_.write(f.data(), f.size());
}

bool KLink::send_sync() {
    const uint8_t s[4] = {MSG_SYNC, MSG_SYNC, MSG_SYNC, MSG_SYNC};
    return io_.write(s, sizeof s);
}

bool KLink::blind_reset() {
    Bytes c;
    if (!mp_.encode("reset", c, nullptr)) {
        // no dictionary (identify never worked this time): Klipper's "reset" has no fixed id, so
        // nothing can be sent
        return false;
    }
    bool ok = send_sync();
    for (uint8_t s = 0; s <= MSG_SEQ_MASK; s++) {
        Bytes f = frame_encode(c, uint8_t((seq_ + s) & MSG_SEQ_MASK));
        ok = io_.write(f.data(), f.size()) && ok;
    }
    seq_ = 0;   // a freshly started board expects sequence 0
    return ok;
}

bool KLink::get_config(bool& configured, bool& shutdown, std::string* err, uint32_t* crc, uint32_t* move_count) {
    std::vector<Message> msgs;
    if (!command("get_config", &msgs, 300, err)) return false;
    for (auto& m : msgs) {
        if (m.name == "config") {
            configured = m.get("is_config") != 0;
            shutdown = m.get("is_shutdown") != 0;
            if (crc) *crc = uint32_t(m.get("crc"));
            if (move_count) *move_count = uint32_t(m.get("move_count"));
            return true;
        }
    }
    if (err) *err = "no config response";
    return false;
}

}  // namespace ph
