// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/config_ini.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace ph {
namespace {

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && isspace((unsigned char)s[a])) a++;
    while (b > a && isspace((unsigned char)s[b - 1])) b--;
    return s.substr(a, b - a);
}
std::string lower(std::string s) {
    for (auto& c : s) c = char(tolower((unsigned char)c));
    return s;
}
std::string strip_comment(const std::string& s) {
    // Klipper: '#' or ';' begins a comment (only at start or after whitespace).
    for (size_t i = 0; i < s.size(); i++)
        if ((s[i] == '#' || s[i] == ';') && (i == 0 || isspace((unsigned char)s[i - 1])))
            return s.substr(0, i);
    return s;
}

}  // namespace

bool Config::parse(const std::string& text) {
    sections_.clear(); errors_.clear();
    std::istringstream in(text);
    std::string raw; int lineno = 0;
    ConfigSection* cur = nullptr;
    std::string last_key;
    bool had_syntax_error = false;
    while (std::getline(in, raw)) {
        lineno++;
        if (!raw.empty() && raw.back() == '\r') raw.pop_back();
        // Klipper's auto-saved block marker: stop there (we don't use SAVE_CONFIG).
        if (raw.rfind("#*#", 0) == 0) break;
        std::string line = strip_comment(raw);
        if (trim(line).empty()) continue;
        bool indented = isspace((unsigned char)line[0]);
        std::string t = trim(line);
        if (indented && cur && !last_key.empty()) {
            auto& v = cur->vals_[last_key].v;
            v += (v.empty() ? "" : "\n") + t;
            continue;
        }
        if (t.front() == '[') {
            if (t.back() != ']') {
                errors_.push_back("line " + std::to_string(lineno) + ": bad section header");
                had_syntax_error = true; cur = nullptr; continue;
            }
            std::string name = lower(trim(t.substr(1, t.size() - 2)));
            // collapse inner whitespace: "[stepper   x]" -> "stepper x"
            std::istringstream ns(name); std::string w, n2;
            while (ns >> w) n2 += (n2.empty() ? "" : " ") + w;
            if (name.rfind("include", 0) == 0) {
                errors_.push_back("line " + std::to_string(lineno) + ": [include] not supported");
                had_syntax_error = true; cur = nullptr; continue;
            }
            auto it = std::find_if(sections_.begin(), sections_.end(),
                                   [&](const ConfigSection& s) { return s.name_ == n2; });
            if (it == sections_.end()) {
                sections_.emplace_back();
                cur = &sections_.back();
                cur->name_ = n2;
            } else {
                cur = &*it;  // Klipper merges repeated sections
            }
            last_key.clear();
            continue;
        }
        size_t sep = t.find_first_of(":=");
        if (!cur || sep == std::string::npos) {
            errors_.push_back("line " + std::to_string(lineno) +
                              (cur ? ": expected 'key: value'" : ": value outside section"));
            had_syntax_error = true; continue;
        }
        last_key = lower(trim(t.substr(0, sep)));
        cur->vals_[last_key] = {trim(t.substr(sep + 1)), lineno};
    }
    for (auto& s : sections_) s.owner_ = this;  // after vector stops growing
    return !had_syntax_error;
}

const ConfigSection* Config::section(const std::string& name) const {
    for (auto& s : sections_) if (s.name_ == name) return &s;
    return nullptr;
}

std::vector<const ConfigSection*> Config::sections_prefixed(const std::string& p) const {
    std::vector<const ConfigSection*> v;
    for (auto& s : sections_) if (s.name_.rfind(p, 0) == 0) v.push_back(&s);
    return v;
}

std::vector<std::string> Config::unused_keys() const {
    std::vector<std::string> v;
    for (auto& s : sections_)
        for (auto& kv : s.vals_)
            if (!kv.second.used)
                v.push_back("[" + s.name_ + "] " + kv.first + " (line " +
                            std::to_string(kv.second.line) + ")");
    return v;
}

// ---- ConfigSection ----

const ConfigSection::Val* ConfigSection::find(const std::string& k) const {
    auto it = vals_.find(k);
    if (it == vals_.end()) return nullptr;
    it->second.used = true;
    return &it->second;
}
void ConfigSection::error(const std::string& m) const {
    if (owner_) owner_->errors().push_back("[" + name_ + "] " + m);
}

std::string ConfigSection::get(const std::string& k) const {
    auto v = find(k);
    if (!v) { error("missing required option '" + k + "'"); return ""; }
    return v->v;
}
std::string ConfigSection::get(const std::string& k, const std::string& d) const {
    auto v = find(k);
    return v ? v->v : d;
}

double ConfigSection::num(const std::string& k, const Val* v, double minv,
                          double maxv) const {
    char* end = nullptr;
    double x = strtod(v->v.c_str(), &end);
    if (end == v->v.c_str() || *end) {
        error("'" + k + "' is not a number (line " + std::to_string(v->line) + ")");
        return 0;
    }
    if (x < minv || x > maxv) {
        std::ostringstream o;
        o << "'" << k << "' = " << x << " out of range [" << minv << ", " << maxv
          << "] (line " << v->line << ")";
        error(o.str());
    }
    return x;
}
double ConfigSection::getf(const std::string& k) const {
    auto v = find(k);
    if (!v) { error("missing required option '" + k + "'"); return 0; }
    return num(k, v, -1e300, 1e300);
}
double ConfigSection::getf(const std::string& k, double d, double minv,
                           double maxv) const {
    auto v = find(k);
    return v ? num(k, v, minv, maxv) : d;
}
long ConfigSection::geti(const std::string& k) const {
    auto v = find(k);
    if (!v) { error("missing required option '" + k + "'"); return 0; }
    return long(num(k, v, -2147483647.0, 2147483647.0));
}
long ConfigSection::geti(const std::string& k, long d, long minv, long maxv) const {
    auto v = find(k);
    return v ? long(num(k, v, double(minv), double(maxv))) : d;
}
bool ConfigSection::getb(const std::string& k, bool d) const {
    auto v = find(k);
    if (!v) return d;
    std::string s = lower(v->v);
    if (s == "1" || s == "true" || s == "yes" || s == "on") return true;
    if (s == "0" || s == "false" || s == "no" || s == "off") return false;
    error("'" + k + "' must be true/false (line " + std::to_string(v->line) + ")");
    return d;
}

std::string config_set_option(const std::string& text, const std::string& section, const std::string& key,
                              const std::string& value) {
    std::vector<std::string> lines;
    {
        std::istringstream in(text);
        std::string l;
        while (std::getline(in, l)) lines.push_back(l);
    }
    auto norm_section = [](const std::string& raw) -> std::string {
        std::string t = trim(strip_comment(raw));
        if (t.size() < 2 || t.front() != '[' || t.back() != ']') return "";
        std::istringstream ns(lower(trim(t.substr(1, t.size() - 2))));
        std::string w, n;
        while (ns >> w) n += (n.empty() ? "" : " ") + w;
        return n;
    };
    std::string want = lower(section), wkey = lower(key), newline = key + ": " + value;
    int start = -1, end = int(lines.size());
    for (size_t i = 0; i < lines.size(); i++) {
        if (lines[i].rfind("#*#", 0) == 0) {
            if (start >= 0 && end == int(lines.size())) end = int(i);
            break;
        }
        std::string n = norm_section(lines[i]);
        if (n.empty()) continue;
        if (start >= 0) {
            end = int(i);
            break;
        }
        if (n == want) start = int(i);
    }
    if (start < 0) {
        std::string out = text;
        if (!out.empty() && out.back() != '\n') out += "\n";
        return out + "\n[" + section + "]\n" + newline + "\n";
    }
    int last_content = start;
    for (int i = start + 1; i < end; i++) {
        std::string t = trim(strip_comment(lines[size_t(i)]));
        if (t.empty()) continue;
        last_content = i;
        if (isspace((unsigned char)lines[size_t(i)][0])) continue;   // continuation line
        size_t sep = t.find_first_of(":=");
        if (sep != std::string::npos && lower(trim(t.substr(0, sep))) == wkey) {
            lines[size_t(i)] = newline;
            // drop the old value's continuation lines (a multi-line value such as bed mesh points)
            size_t j = size_t(i) + 1;
            while (j < size_t(end) && !lines[j].empty() && isspace((unsigned char)lines[j][0]) &&
                   !trim(strip_comment(lines[j])).empty())
                lines.erase(lines.begin() + long(j)), end--;
            std::string out;
            for (auto& l : lines) out += l + "\n";
            return out;
        }
    }
    lines.insert(lines.begin() + last_content + 1, newline);
    std::string out;
    for (auto& l : lines) out += l + "\n";
    return out;
}

}  // namespace ph
