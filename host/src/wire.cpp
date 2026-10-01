// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/wire.h"
#include <algorithm>

namespace ph {

uint16_t crc16_ccitt(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t data = buf[i];
        data ^= crc & 0xFF;
        data ^= (data & 0x0F) << 4;
        crc = ((uint16_t(data) << 8) | (crc >> 8)) ^ (data >> 4)
              ^ (uint16_t(data) << 3);
    }
    return crc;
}

void vlq_encode(Bytes& out, int64_t v64) {
    int32_t v = int32_t(uint32_t(v64));  // wire values are 32-bit
    if (v >= 0xC000000 || v < -0x4000000) out.push_back(((v >> 28) & 0x7F) | 0x80);
    if (v >= 0x180000  || v < -0x80000)   out.push_back(((v >> 21) & 0x7F) | 0x80);
    if (v >= 0x3000    || v < -0x1000)    out.push_back(((v >> 14) & 0x7F) | 0x80);
    if (v >= 0x60      || v < -0x20)      out.push_back(((v >> 7)  & 0x7F) | 0x80);
    out.push_back(v & 0x7F);
}

bool vlq_decode(const uint8_t* buf, size_t len, size_t& pos, bool is_signed,
                int64_t& out) {
    if (pos >= len) return false;
    uint8_t c = buf[pos++];
    uint32_t v = c & 0x7F;
    if ((c & 0x60) == 0x60) v |= uint32_t(-0x20);
    while (c & 0x80) {
        if (pos >= len) return false;
        c = buf[pos++];
        v = (v << 7) | (c & 0x7F);
    }
    out = is_signed ? int64_t(int32_t(v)) : int64_t(v);
    return true;
}

Bytes frame_encode(const Bytes& content, uint8_t seq) {
    Bytes b;
    b.reserve(content.size() + MSG_MIN);
    b.push_back(uint8_t(content.size() + MSG_MIN));
    b.push_back(MSG_DEST | (seq & MSG_SEQ_MASK));
    b.insert(b.end(), content.begin(), content.end());
    uint16_t crc = crc16_ccitt(b.data(), b.size());
    b.push_back(crc >> 8);
    b.push_back(crc & 0xFF);
    b.push_back(MSG_SYNC);
    return b;
}

size_t FrameDecoder::feed(const uint8_t* data, size_t len,
                          std::vector<Block>& out) {
    buf_.insert(buf_.end(), data, data + len);
    size_t found = 0;
    for (;;) {
        if (need_sync_) {
            auto it = std::find(buf_.begin(), buf_.end(), MSG_SYNC);
            if (it == buf_.end()) { buf_.clear(); break; }
            buf_.erase(buf_.begin(), it + 1);
            need_sync_ = false;
            resyncs_++;
        }
        if (buf_.empty()) break;
        uint8_t mlen = buf_[0];
        if (mlen < MSG_MIN || mlen > MSG_MAX) { need_sync_ = true; continue; }
        // Check seq byte early so junk can't stall us waiting for a fake length.
        if (buf_.size() >= 2 && (buf_[1] & ~MSG_SEQ_MASK) != MSG_DEST) {
            need_sync_ = true; continue;
        }
        if (buf_.size() < mlen) break;
        bool ok = buf_[mlen - 1] == MSG_SYNC;
        if (ok) {
            uint16_t crc = crc16_ccitt(buf_.data(), mlen - MSG_TRAILER);
            if (buf_[mlen - 3] != (crc >> 8) || buf_[mlen - 2] != (crc & 0xFF)) {
                crc_errors_++;
                ok = false;
            }
        }
        if (!ok) { need_sync_ = true; continue; }
        Block blk;
        blk.seq = buf_[1] & MSG_SEQ_MASK;
        blk.content.assign(buf_.begin() + MSG_HEADER,
                           buf_.begin() + mlen - MSG_TRAILER);
        out.push_back(std::move(blk));
        buf_.erase(buf_.begin(), buf_.begin() + mlen);
        found++;
    }
    return found;
}

}  // namespace ph
