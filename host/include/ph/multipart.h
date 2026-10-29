// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Streaming multipart/form-data parser (RFC 7578), for slicer uploads ("send to printer" from
// PrusaSlicer / OrcaSlicer, which post the G-code like OctoPrint's POST /api/files/local).
// Feed the request body in any chunk sizes; a file part's data is handed on as it arrives, so a
// 50 MB file needs no more memory than one chunk plus the boundary.
#pragma once
#include <cstddef>
#include <functional>
#include <string>

namespace ph {

class Multipart {
public:
    struct Part {
        std::string name;       // form field name ("file", "print", "select", "path")
        std::string filename;   // empty for a plain field
        std::string type;       // Content-Type of the part, if given
    };

    explicit Multipart(const std::string& boundary);

    // Boundary from a Content-Type header value: multipart/form-data; boundary=----abc
    static bool boundary_from_content_type(const std::string& content_type, std::string& boundary);

    // Callbacks; each may return false to stop (feed() then returns false, error() is "stopped").
    std::function<bool(const Part&)> on_part;                 // headers of a new part read
    std::function<bool(const char* data, size_t n)> on_data;  // part body bytes, in order
    std::function<bool()> on_part_end;                        // the current part is complete

    // Returns false on malformed input or when a callback stopped it.
    bool feed(const char* data, size_t n);
    bool done() const { return state_ == State::Done; }       // the closing boundary was seen
    const std::string& error() const { return err_; }

private:
    enum class State { Preamble, AfterBoundary, Headers, Body, Done, Failed };
    bool fail(const std::string& e);
    bool parse_headers(const std::string& block, Part& p);
    std::string delim_;   // "\r\n--" + boundary
    std::string buf_;
    State state_ = State::Preamble;
    std::string err_;
};

// Embedded preview pictures in sliced G-code (PrusaSlicer, OrcaSlicer, Cura with a thumbnail
// plug-in, SuperSlicer): "; thumbnail begin 300x300 12345" ... base64 lines ... "; thumbnail end",
// also "; thumbnail_JPG begin". Picks the largest PNG / JPG no wider than max_px (QOI is skipped:
// browsers cannot show it). Returns false if none is complete in `head` (the start of the file).
struct GcodeThumbnail {
    int w = 0, h = 0;
    std::string format;   // "png" or "jpg"
    std::string base64;   // without line breaks or "; "
};
bool find_gcode_thumbnail(const std::string& head, int max_px, GcodeThumbnail& out);

}  // namespace ph
