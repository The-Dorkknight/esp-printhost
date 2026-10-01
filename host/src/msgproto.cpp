// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/msgproto.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <sstream>

namespace ph {
namespace {

bool type_from(const std::string& f, PType& t) {
    if (f == "%u") t = PType::U32;
    else if (f == "%i") t = PType::I32;
    else if (f == "%hu") t = PType::U16;
    else if (f == "%hi") t = PType::I16;
    else if (f == "%c") t = PType::Byte;
    else if (f == "%s") t = PType::Str;
    else if (f == "%.*s" || f == "%*s") t = PType::Buf;
    else return false;
    return true;
}
bool is_signed(PType t) { return t == PType::I32 || t == PType::I16; }
bool is_bytes(PType t) { return t == PType::Str || t == PType::Buf; }

std::vector<std::string> split_ws(const std::string& s) {
    std::vector<std::string> v; std::istringstream is(s); std::string w;
    while (is >> w) v.push_back(w);
    return v;
}
bool set_err(std::string* err, const std::string& m) { if (err) *err = m; return false; }

}  // namespace

bool parse_format(const std::string& fmt, MsgFormat& out) {
    auto parts = split_ws(fmt);
    if (parts.empty()) return false;
    out = MsgFormat{};
    out.format = fmt;
    out.name = parts[0];
    for (size_t i = 1; i < parts.size(); i++) {
        auto eq = parts[i].find('=');
        if (eq == std::string::npos) return false;
        Param p;
        p.name = parts[i].substr(0, eq);
        if (!type_from(parts[i].substr(eq + 1), p.type)) return false;
        out.params.push_back(p);
    }
    return true;
}

MsgProto::MsgProto() {
    add("identify_response offset=%u data=%.*s", 0, true);
    add("identify offset=%u count=%c", 1, false);
}

void MsgProto::bind_enums(MsgFormat& f) const {
    for (auto& p : f.params) {
        if (is_bytes(p.type)) continue;
        for (auto& e : enums_) {
            const std::string& en = e.first;
            bool suffix = p.name.size() > en.size() + 1
                && p.name.compare(p.name.size() - en.size() - 1, std::string::npos,
                                  "_" + en) == 0;
            if (p.name == en || suffix) { p.enum_name = en; break; }
        }
    }
}

void MsgProto::add(const std::string& fmt, int id, bool is_response) {
    MsgFormat f;
    if (!parse_format(fmt, f)) return;
    f.id = id;
    bind_enums(f);
    if (is_response) { resp_ids_by_name_[f.name] = id; resp_by_id_[id] = f; }
    else cmds_by_name_[f.name] = f;
}

bool MsgProto::load_dictionary(const std::string& json, std::string* err) {
    JVal root;
    if (!json_parse(json, root, err)) return false;
    if (root.type != JVal::Obj) return set_err(err, "dictionary not an object");

    enums_.clear();
    if (auto en = root.get("enumerations")) {
        for (auto& kv : en->obj) {
            auto& dst = enums_[kv.first];
            for (auto& item : kv.second.obj) {
                const std::string& key = item.first;
                const JVal& v = item.second;
                if (v.type == JVal::Num) { dst[key] = v.as_int(); continue; }
                if (v.type != JVal::Arr || v.arr.size() != 2) continue;
                // Range: "PA0": [start_value, count] -> PA0..PA{count-1}
                size_t d = key.size();
                while (d > 0 && isdigit((unsigned char)key[d - 1])) d--;
                std::string root_name = key.substr(0, d);
                long start_idx = d < key.size() ? strtol(key.c_str() + d, nullptr, 10) : 0;
                int64_t start_val = v.arr[0].as_int(), count = v.arr[1].as_int();
                for (int64_t i = 0; i < count; i++)
                    dst[root_name + std::to_string(start_idx + i)] = start_val + i;
            }
        }
    }
    cmds_by_name_.clear(); resp_by_id_.clear(); resp_ids_by_name_.clear();
    add("identify_response offset=%u data=%.*s", 0, true);
    add("identify offset=%u count=%c", 1, false);
    if (auto c = root.get("commands"))
        for (auto& kv : c->obj) add(kv.first, int(kv.second.as_int()), false);
    if (auto r = root.get("responses"))
        for (auto& kv : r->obj) add(kv.first, int(kv.second.as_int()), true);
    // Not yet implemented (task G1): "output" (printf-style debug) messages are not decoded yet.
    config_.clear();
    if (auto cf = root.get("config")) config_ = cf->obj;
    if (auto v = root.get("version")) version_ = v->str;
    return true;
}

const MsgFormat* MsgProto::command(const std::string& name) const {
    auto it = cmds_by_name_.find(name);
    return it == cmds_by_name_.end() ? nullptr : &it->second;
}
const MsgFormat* MsgProto::response(const std::string& name) const {
    auto it = resp_ids_by_name_.find(name);
    return it == resp_ids_by_name_.end() ? nullptr : &resp_by_id_.at(it->second);
}
std::string MsgProto::config_str(const std::string& k) const {
    auto it = config_.find(k);
    if (it == config_.end()) return "";
    if (it->second.type == JVal::Str) return it->second.str;
    std::ostringstream o; o << it->second.num; return o.str();
}
double MsgProto::config_num(const std::string& k, double dflt) const {
    auto it = config_.find(k);
    if (it == config_.end()) return dflt;
    if (it->second.type == JVal::Num) return it->second.num;
    return strtod(it->second.str.c_str(), nullptr);
}
bool MsgProto::enum_value(const std::string& en, const std::string& key,
                          int64_t& v) const {
    auto e = enums_.find(en);
    if (e == enums_.end()) return false;
    auto it = e->second.find(key);
    if (it == e->second.end()) return false;
    v = it->second; return true;
}

namespace {
// Split "PA12" -> ("PA", 12); no trailing digits -> (name, -1).
std::pair<std::string, long> split_num(const std::string& k) {
    size_t d = k.size();
    while (d > 0 && isdigit((unsigned char)k[d - 1])) d--;
    if (d == k.size()) return {k, -1};
    return {k.substr(0, d), strtol(k.c_str() + d, nullptr, 10)};
}
}  // namespace

std::vector<std::string> MsgProto::enum_keys(const std::string& en) const {
    std::vector<std::string> v;
    auto e = enums_.find(en);
    if (e == enums_.end()) return v;
    for (auto& kv : e->second) v.push_back(kv.first);
    std::sort(v.begin(), v.end(), [](const std::string& a, const std::string& b) {
        auto x = split_num(a), y = split_num(b);
        return x.first != y.first ? x.first < y.first : x.second < y.second;
    });
    return v;
}

std::string MsgProto::enum_name_of(const std::string& en, int64_t v) const {
    auto e = enums_.find(en);
    if (e == enums_.end()) return "";
    for (auto& kv : e->second) if (kv.second == v) return kv.first;
    return "";
}

std::string MsgProto::enum_summary(const std::string& en, size_t max_len) const {
    std::string out;
    auto keys = enum_keys(en);
    for (size_t i = 0; i < keys.size();) {
        auto a = split_num(keys[i]);
        size_t j = i;
        while (j + 1 < keys.size()) {
            auto b = split_num(keys[j + 1]);
            if (a.second < 0 || b.first != a.first || b.second != split_num(keys[j]).second + 1) break;
            j++;
        }
        std::string part = (i == j) ? keys[i] : keys[i] + ".." + keys[j];
        if (!out.empty()) out += ", ";
        if (out.size() + part.size() > max_len) { out += "..."; break; }
        out += part;
        i = j + 1;
    }
    return out;
}

bool MsgProto::encode(const std::string& cmdline, Bytes& out,
                      std::string* err) const {
    auto parts = split_ws(cmdline);
    if (parts.empty()) return set_err(err, "empty command");
    const MsgFormat* f = command(parts[0]);
    if (!f) return set_err(err, "unknown command: " + parts[0]);
    std::map<std::string, std::string> args;
    for (size_t i = 1; i < parts.size(); i++) {
        auto eq = parts[i].find('=');
        if (eq == std::string::npos) return set_err(err, "bad arg: " + parts[i]);
        args[parts[i].substr(0, eq)] = parts[i].substr(eq + 1);
    }
    vlq_encode(out, f->id);
    for (auto& p : f->params) {
        auto it = args.find(p.name);
        if (it == args.end()) return set_err(err, f->name + ": missing " + p.name);
        const std::string& s = it->second;
        if (is_bytes(p.type)) {
            if (s.size() > 255) return set_err(err, "string too long");
            out.push_back(uint8_t(s.size()));
            out.insert(out.end(), s.begin(), s.end());
            continue;
        }
        int64_t v;
        if (!p.enum_name.empty() && enum_value(p.enum_name, s, v)) {
            // resolved via enumeration (e.g. pin=PA5)
        } else {
            char* end = nullptr;
            v = strtoll(s.c_str(), &end, 0);
            if (end == s.c_str() || *end)
                return set_err(err, f->name + ": bad value " + p.name + "=" + s);
        }
        vlq_encode(out, v);
    }
    return true;
}

bool MsgProto::decode(const Bytes& content, std::vector<Message>& out,
                      std::string* err) const {
    const uint8_t* d = content.data();
    size_t n = content.size(), pos = 0;
    while (pos < n) {
        int64_t id;
        if (!vlq_decode(d, n, pos, false, id)) return set_err(err, "truncated id");
        auto it = resp_by_id_.find(int(id));
        if (it == resp_by_id_.end())
            return set_err(err, "unknown response id " + std::to_string(id));
        const MsgFormat& f = it->second;
        Message m; m.name = f.name;
        for (auto& p : f.params) {
            PVal v;
            if (is_bytes(p.type)) {
                if (pos >= n) return set_err(err, "truncated len");
                size_t l = d[pos++];
                if (pos + l > n) return set_err(err, "truncated bytes");
                v.is_bytes = true;
                v.b.assign(d + pos, d + pos + l);
                pos += l;
            } else if (!vlq_decode(d, n, pos, is_signed(p.type), v.i)) {
                return set_err(err, "truncated param " + p.name);
            }
            m.params[p.name] = std::move(v);
        }
        out.push_back(std::move(m));
    }
    return true;
}

}  // namespace ph
