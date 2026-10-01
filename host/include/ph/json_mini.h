// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Tiny JSON reader — enough for Klipper's data dictionary. Not a general lib.
#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ph {

struct JVal {
    enum Type { Null, Bool, Num, Str, Arr, Obj } type = Null;
    bool b = false;
    double num = 0;
    std::string str;
    std::vector<JVal> arr;
    std::map<std::string, JVal> obj;

    const JVal* get(const std::string& k) const {
        auto it = obj.find(k);
        return it == obj.end() ? nullptr : &it->second;
    }
    int64_t as_int() const { return int64_t(num); }
};

// Returns false on syntax error (err gets a message).
bool json_parse(const std::string& text, JVal& out, std::string* err = nullptr);

}  // namespace ph
