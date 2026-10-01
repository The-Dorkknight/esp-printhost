// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Klipper-compatible wire format: CRC16, VLQ ints, message blocks.
#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace ph {

using Bytes = std::vector<uint8_t>;

// ---- CRC16-CCITT, Klipper variant (init 0xFFFF). ----
uint16_t crc16_ccitt(const uint8_t* buf, size_t len);

// ---- VLQ integers (Klipper msgproto encoding). ----
void vlq_encode(Bytes& out, int64_t v);           // v is treated as 32-bit
// Returns false on truncation. `is_signed` = format is %i / %hi.
bool vlq_decode(const uint8_t* buf, size_t len, size_t& pos, bool is_signed,
                int64_t& out);

// ---- Message blocks. ----
constexpr size_t MSG_HEADER = 2, MSG_TRAILER = 3;
constexpr size_t MSG_MIN = 5, MSG_MAX = 64;
constexpr size_t MSG_MAX_PAYLOAD = MSG_MAX - MSG_MIN;  // 59
constexpr uint8_t MSG_SYNC = 0x7E, MSG_DEST = 0x10, MSG_SEQ_MASK = 0x0F;

// Wrap content (<= 59 bytes) into a block. seq is 0..15.
Bytes frame_encode(const Bytes& content, uint8_t seq);

// Streaming decoder: feed raw serial bytes, get complete blocks.
class FrameDecoder {
public:
    struct Block { uint8_t seq; Bytes content; };
    // Returns number of blocks appended to `out`.
    size_t feed(const uint8_t* data, size_t len, std::vector<Block>& out);
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t resyncs() const { return resyncs_; }
private:
    Bytes buf_;
    bool need_sync_ = false;
    uint32_t crc_errors_ = 0, resyncs_ = 0;
};

}  // namespace ph
