// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// printer.cfg-style config parser (Klipper-compatible subset).
//   [section name]
//   key: value        (or key = value)
//     indented continuation lines append to the previous value
//   # and ; start comments
#pragma once
#include <map>
#include <string>
#include <vector>

namespace ph {

class ConfigSection;

class Config {
public:
    bool parse(const std::string& text);  // false if syntax errors
    const ConfigSection* section(const std::string& name) const;
    std::vector<const ConfigSection*> sections_prefixed(const std::string& p) const;
    bool has(const std::string& name) const { return section(name) != nullptr; }

    // Collected problems (syntax + typed-getter failures). Show in web UI.
    std::vector<std::string>& errors() const { return errors_; }
    bool ok() const { return errors_.empty(); }
    // Keys that were never read — likely typos. Call after loading everything.
    std::vector<std::string> unused_keys() const;

private:
    std::vector<ConfigSection> sections_;
    mutable std::vector<std::string> errors_;
};

class ConfigSection {
public:
    const std::string& name() const { return name_; }
    bool has(const std::string& k) const { return vals_.count(k) != 0; }

    // Required getters record an error in the parent Config if missing/invalid.
    std::string get(const std::string& k) const;
    std::string get(const std::string& k, const std::string& dflt) const;
    double getf(const std::string& k) const;
    double getf(const std::string& k, double dflt, double minv = -1e300,
                double maxv = 1e300) const;
    long geti(const std::string& k) const;
    long geti(const std::string& k, long dflt, long minv = -2147483647L,
              long maxv = 2147483647L) const;
    bool getb(const std::string& k, bool dflt) const;

private:
    friend class Config;
    struct Val { std::string v; int line; mutable bool used = false; };
    const Val* find(const std::string& k) const;
    double num(const std::string& k, const Val* v, double minv, double maxv) const;
    void error(const std::string& m) const;

    std::string name_;
    std::map<std::string, Val> vals_;
    const Config* owner_ = nullptr;
};

}  // namespace ph
