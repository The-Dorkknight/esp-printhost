// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/multipart.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace ph {

namespace {
std::string lower(std::string s) {
    for (auto& c : s) c = char(tolower((unsigned char)c));
    return s;
}

// value of key="..." or key=token in a header parameter list; "" if absent
std::string param(const std::string& h, const std::string& key) {
    std::string lh = lower(h);
    size_t pos = 0;
    while ((pos = lh.find(key + "=", pos)) != std::string::npos) {
        // must start a parameter: preceded by ';' or whitespace
        if (pos > 0 && lh[pos - 1] != ';' && lh[pos - 1] != ' ' && lh[pos - 1] != '\t') {
            pos++;
            continue;
        }
        size_t v = pos + key.size() + 1;
        if (v < h.size() && h[v] == '"') {
            std::string out;
            for (size_t i = v + 1; i < h.size() && h[i] != '"'; i++) {
                if (h[i] == '\\' && i + 1 < h.size()) i++;
                out += h[i];
            }
            return out;
        }
        size_t e = h.find(';', v);
        std::string out = h.substr(v, e == std::string::npos ? std::string::npos : e - v);
        while (!out.empty() && isspace((unsigned char)out.back())) out.pop_back();
        return out;
    }
    return "";
}
}  // namespace

Multipart::Multipart(const std::string& boundary) : delim_("\r\n--" + boundary), buf_("\r\n") {
    if (boundary.empty() || boundary.size() > 70) state_ = State::Failed, err_ = "bad boundary";
}

bool Multipart::boundary_from_content_type(const std::string& ct, std::string& b) {
    if (lower(ct).find("multipart/form-data") == std::string::npos) return false;
    b = param(ct, "boundary");
    return !b.empty() && b.size() <= 70;
}

bool Multipart::fail(const std::string& e) {
    state_ = State::Failed;
    err_ = e;
    return false;
}

bool Multipart::parse_headers(const std::string& block, Part& p) {
    size_t start = 0;
    bool disp = false;
    while (start < block.size()) {
        size_t e = block.find("\r\n", start);
        std::string line = block.substr(start, e == std::string::npos ? std::string::npos : e - start);
        start = e == std::string::npos ? block.size() : e + 2;
        size_t c = line.find(':');
        if (c == std::string::npos) continue;
        std::string k = lower(line.substr(0, c)), v = line.substr(c + 1);
        while (!v.empty() && isspace((unsigned char)v.front())) v.erase(0, 1);
        if (k == "content-disposition") {
            if (lower(v).compare(0, 9, "form-data") != 0) return false;
            disp = true;
            p.name = param(v, "name");
            p.filename = param(v, "filename");
        } else if (k == "content-type") {
            p.type = v;
        }
    }
    return disp;
}

bool Multipart::feed(const char* data, size_t n) {
    if (state_ == State::Failed) return false;
    if (state_ == State::Done) return true;   // epilogue: ignored
    buf_.append(data, n);
    for (;;) {
        switch (state_) {
        case State::Preamble: {
            size_t i = buf_.find(delim_);
            if (i == std::string::npos) {
                if (buf_.size() > delim_.size()) buf_.erase(0, buf_.size() - delim_.size());
                return true;
            }
            buf_.erase(0, i + delim_.size());
            state_ = State::AfterBoundary;
            break;
        }
        case State::AfterBoundary: {
            size_t i = 0;
            while (i < buf_.size() && (buf_[i] == ' ' || buf_[i] == '\t')) i++;   // transport padding
            if (buf_.size() < i + 2) return true;
            if (buf_.compare(i, 2, "--") == 0) {
                state_ = State::Done;
                buf_.clear();
                return true;
            }
            if (buf_.compare(i, 2, "\r\n") != 0) return fail("malformed boundary line");
            buf_.erase(0, i + 2);
            state_ = State::Headers;
            break;
        }
        case State::Headers: {
            size_t i = buf_.find("\r\n\r\n");
            if (i == std::string::npos) {
                if (buf_.size() > 8192) return fail("part headers too long");
                return true;
            }
            Part p;
            if (!parse_headers(buf_.substr(0, i), p)) return fail("part without Content-Disposition: form-data");
            buf_.erase(0, i + 4);
            if (on_part && !on_part(p)) return fail("stopped");
            state_ = State::Body;
            break;
        }
        case State::Body: {
            size_t i = buf_.find(delim_);
            if (i == std::string::npos) {
                // everything except a possible start of the delimiter can go out now
                size_t keep = delim_.size() - 1;
                if (buf_.size() > keep) {
                    size_t out = buf_.size() - keep;
                    if (on_data && !on_data(buf_.data(), out)) return fail("stopped");
                    buf_.erase(0, out);
                }
                return true;
            }
            if (i && on_data && !on_data(buf_.data(), i)) return fail("stopped");
            buf_.erase(0, i + delim_.size());
            if (on_part_end && !on_part_end()) return fail("stopped");
            state_ = State::AfterBoundary;
            break;
        }
        case State::Done:
            return true;
        case State::Failed:
            return false;
        }
    }
}

// ---------------------------------------------------------------- G-code thumbnails

bool find_gcode_thumbnail(const std::string& head, int max_px, GcodeThumbnail& out) {
    bool found = false;
    size_t pos = 0;
    while ((pos = head.find("thumbnail", pos)) != std::string::npos) {
        size_t ls = head.rfind('\n', pos);
        ls = ls == std::string::npos ? 0 : ls + 1;
        size_t le = head.find('\n', pos);
        if (le == std::string::npos) break;   // incomplete line at the end of what was read
        std::string line = head.substr(ls, le - ls);
        pos = le;
        // "; thumbnail begin WxH len", "; thumbnail_JPG begin WxH len", "; thumbnail_QOI begin ..."
        size_t t = line.find("thumbnail");
        if (line.find_first_not_of("; \t") != t) continue;
        std::string rest = line.substr(t + 9);
        std::string fmt = "png";
        if (!rest.empty() && rest[0] == '_') {
            size_t sp = rest.find(' ');
            if (sp == std::string::npos) continue;
            fmt = lower(rest.substr(1, sp - 1));
            rest = rest.substr(sp);
        }
        if (fmt == "jpeg") fmt = "jpg";
        size_t b = rest.find("begin");
        if (b == std::string::npos || (fmt != "png" && fmt != "jpg")) continue;
        int w = 0, h = 0;
        long len = 0;
        if (sscanf(rest.c_str() + b + 5, " %dx%d %ld", &w, &h, &len) < 2 || w <= 0 || h <= 0) continue;
        if (w > max_px || (found && w <= out.w)) continue;
        // collect base64 lines up to the matching end line
        std::string data;
        size_t p = le + 1;
        bool ended = false;
        while (p < head.size()) {
            size_t e = head.find('\n', p);
            if (e == std::string::npos) break;
            std::string l = head.substr(p, e - p);
            p = e + 1;
            if (!l.empty() && l.back() == '\r') l.pop_back();
            if (l.find("thumbnail") != std::string::npos && l.find("end") != std::string::npos) {
                ended = true;
                break;
            }
            size_t s = l.find_first_not_of("; \t");
            if (s != std::string::npos) data += l.substr(s);
        }
        if (!ended || data.empty() || (len > 0 && long(data.size()) != len)) continue;
        out.w = w;
        out.h = h;
        out.format = fmt;
        out.base64 = data;
        found = true;
        pos = p;
    }
    return found;
}

}  // namespace ph
