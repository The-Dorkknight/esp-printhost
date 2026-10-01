// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Klipper message protocol: data dictionary, command encode, response decode.
#pragma once
#include "ph/json_mini.h"
#include "ph/wire.h"
#include <map>
#include <string>
#include <vector>

namespace ph {

enum class PType { U32, I32, U16, I16, Byte, Str, Buf };

struct Param {
    std::string name;
    PType type;
    std::string enum_name;  // non-empty if values are enumeration names
};

struct MsgFormat {
    int id = -1;
    std::string name;          // e.g. "config_digital_out"
    std::string format;        // full Klipper format string
    std::vector<Param> params;
};

// A decoded response parameter: integer or raw bytes (strings/buffers).
struct PVal {
    int64_t i = 0;
    Bytes b;
    bool is_bytes = false;
};

struct Message {
    std::string name;
    std::map<std::string, PVal> params;
    int64_t get(const std::string& k, int64_t dflt = 0) const {
        auto it = params.find(k);
        return it == params.end() ? dflt : it->second.i;
    }
};

// Parse "name a=%u b=%.*s" into a MsgFormat (id left -1).
bool parse_format(const std::string& fmt, MsgFormat& out);

class MsgProto {
public:
    MsgProto();  // preloads identify / identify_response (fixed ids 1 / 0)

    // Load the (already inflated) JSON data dictionary from the MCU.
    bool load_dictionary(const std::string& json, std::string* err = nullptr);

    // Encode a text command: "config_digital_out oid=3 pin=PA5 value=0".
    // Pin names etc. are resolved through enumerations.
    bool encode(const std::string& cmdline, Bytes& out,
                std::string* err = nullptr) const;

    // Decode all messages in one block's content.
    bool decode(const Bytes& content, std::vector<Message>& out,
                std::string* err = nullptr) const;

    const MsgFormat* command(const std::string& name) const;
    const MsgFormat* response(const std::string& name) const;
    // Values from dictionary "config" (e.g. CLOCK_FREQ, MCU).
    std::string config_str(const std::string& k) const;
    double config_num(const std::string& k, double dflt = 0) const;
    bool enum_value(const std::string& enum_name, const std::string& key,
                    int64_t& v) const;
    // All names in an enumeration (e.g. every pin name), sorted naturally.
    std::vector<std::string> enum_keys(const std::string& enum_name) const;
    // Reverse lookup: value -> name ("" if unknown). Used for static_string_id.
    std::string enum_name_of(const std::string& enum_name, int64_t v) const;
    // Compact summary like "gpio0..gpio29" or "PA0..PA15, PB0..PB15".
    std::string enum_summary(const std::string& enum_name, size_t max_len = 200) const;
    bool has_command(const std::string& name) const { return command(name) != nullptr; }
    const std::string& version() const { return version_; }

private:
    void add(const std::string& fmt, int id, bool is_response);
    void bind_enums(MsgFormat& f) const;

    std::map<std::string, MsgFormat> cmds_by_name_;
    std::map<int, MsgFormat> resp_by_id_;
    std::map<std::string, int> resp_ids_by_name_;
    std::map<std::string, std::map<std::string, int64_t>> enums_;
    std::map<std::string, JVal> config_;
    std::string version_;
};

}  // namespace ph
