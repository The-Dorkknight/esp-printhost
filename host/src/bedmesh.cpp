// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
#include "ph/bedmesh.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace ph {

double BedMesh::at(double x, double y) const {
    if (!valid()) return 0;
    double fx = (std::min(std::max(x, min_x), max_x) - min_x) / (max_x - min_x) * (cols - 1);
    double fy = (std::min(std::max(y, min_y), max_y) - min_y) / (max_y - min_y) * (rows - 1);
    int c = std::min(int(fx), cols - 2), r = std::min(int(fy), rows - 2);
    double tx = fx - c, ty = fy - r;
    auto Z = [&](int rr, int cc) { return z[size_t(rr * cols + cc)]; };
    return (Z(r, c) * (1 - tx) + Z(r, c + 1) * tx) * (1 - ty) + (Z(r + 1, c) * (1 - tx) + Z(r + 1, c + 1) * tx) * ty;
}

double BedMesh::lowest() const { return z.empty() ? 0 : *std::min_element(z.begin(), z.end()); }
double BedMesh::highest() const { return z.empty() ? 0 : *std::max_element(z.begin(), z.end()); }
double BedMesh::mean() const {
    double s = 0;
    for (double v : z) s += v;
    return z.empty() ? 0 : s / double(z.size());
}

int BedMesh::level(double v) const {
    // relative to the mean: within 20 % of the largest deviation is "base"
    double m = mean(), dev = std::max(highest() - m, m - lowest());
    if (dev < 0.005) return 2;   // a flat bed: everything is base
    double r = (v - m) / dev;
    return r >= 0.6 ? 4 : r >= 0.2 ? 3 : r > -0.2 ? 2 : r > -0.6 ? 1 : 0;
}

std::vector<std::pair<std::string, std::string>> BedMesh::profile_options() const {
    std::vector<std::pair<std::string, std::string>> o;
    std::string pts;
    char b[32];
    for (int r = 0; r < rows; r++) {
        pts += "\n  ";
        for (int c = 0; c < cols; c++) {
            snprintf(b, sizeof b, "%s%.4f", c ? ", " : "", z[size_t(r * cols + c)]);
            pts += b;
        }
    }
    o.push_back({"version", "1"});
    o.push_back({"points", pts});
    o.push_back({"x_count", std::to_string(cols)});
    o.push_back({"y_count", std::to_string(rows)});
    o.push_back({"mesh_x_pps", "0"});
    o.push_back({"mesh_y_pps", "0"});
    o.push_back({"algo", "direct"});
    auto f = [&](double v) {
        snprintf(b, sizeof b, "%.3f", v);
        return std::string(b);
    };
    o.push_back({"min_x", f(min_x)});
    o.push_back({"max_x", f(max_x)});
    o.push_back({"min_y", f(min_y)});
    o.push_back({"max_y", f(max_y)});
    return o;
}

bool BedMesh::from_profile(const std::string& points, int xc, int yc, double x0, double x1, double y0, double y1,
                           std::string* err) {
    std::vector<double> v;
    std::string tok;
    for (char ch : points + ",") {
        if (ch == ',' || ch == '\n' || ch == ' ' || ch == '\t' || ch == ';') {
            if (!tok.empty()) {
                char* e = nullptr;
                double d = strtod(tok.c_str(), &e);
                if (!e || *e) {
                    if (err) *err = "bad number '" + tok + "' in points";
                    return false;
                }
                v.push_back(d);
                tok.clear();
            }
        } else {
            tok += ch;
        }
    }
    if (xc < 2 || yc < 2 || v.size() != size_t(xc * yc) || !(x1 > x0) || !(y1 > y0)) {
        if (err) *err = "points do not match x_count x y_count, or the bounds are wrong";
        return false;
    }
    cols = xc, rows = yc, min_x = x0, max_x = x1, min_y = y0, max_y = y1, z = v;
    return true;
}

std::string BedMesh::text() const {
    if (!valid()) return "no bed mesh";
    std::string s;
    char b[200];
    snprintf(b, sizeof b, "bed mesh %dx%d, %.1f..%.1f x %.1f..%.1f mm; range %.3f mm (%.3f .. %+.3f)\n", cols, rows, min_x,
             max_x, min_y, max_y, highest() - lowest(), lowest(), highest());
    s += b;
    for (int r = rows - 1; r >= 0; r--) {   // back of the bed on top
        snprintf(b, sizeof b, "y%6.1f |", point_y(r));
        s += b;
        for (int c = 0; c < cols; c++) {
            snprintf(b, sizeof b, " %+.3f", z[size_t(r * cols + c)]);
            s += b;
        }
        s += "\n";
    }
    return s;
}

std::string BedMesh::json() const {
    if (!valid()) return "null";
    std::string s;
    char b[96];
    snprintf(b, sizeof b, "{\"cols\":%d,\"rows\":%d,\"min\":[%.2f,%.2f],\"max\":[%.2f,%.2f],\"z\":[", cols, rows, min_x, min_y,
             max_x, max_y);
    s += b;
    for (size_t i = 0; i < z.size(); i++) {
        snprintf(b, sizeof b, "%s%.4f", i ? "," : "", z[i]);
        s += b;
    }
    s += "],\"lvl\":[";
    for (size_t i = 0; i < z.size(); i++) s += (i ? "," : "") + std::to_string(level(z[i]));
    snprintf(b, sizeof b, "],\"range\":%.4f,\"mean\":%.4f}", highest() - lowest(), mean());
    return s + b;
}

int screw_thread_index(const std::string& n) {
    static const char* const T[8] = {"CW-M3", "CCW-M3", "CW-M4", "CCW-M4", "CW-M5", "CCW-M5", "CW-M6", "CCW-M6"};
    std::string u;
    for (char c : n) u += char(toupper((unsigned char)c));
    for (int i = 0; i < 8; i++)
        if (u == T[i]) return i;
    return -1;
}

std::string screws_report(const ScrewsConfig& c, const std::vector<double>& z, const std::string& direction,
                          std::string* json) {
    static const double factor[8] = {0.5, 0.5, 0.7, 0.7, 0.8, 0.8, 1.0, 1.0};   // thread pitch, mm per turn
    bool cw_thread = (c.thread & 1) == 0;
    size_t base = 0;
    if (direction == "CW" || direction == "CCW") {
        bool use_max = (cw_thread && direction == "CW") || (!cw_thread && direction == "CCW");
        for (size_t i = 1; i < z.size(); i++)
            if (use_max ? z[i] > z[base] : z[i] < z[base]) base = i;
    }
    std::string s = "01:20 means 1 full turn and 20 minutes, CW=clockwise, CCW=counter-clockwise\n";
    std::string js = "[";
    char b[200];
    for (size_t i = 0; i < c.screws.size() && i < z.size(); i++) {
        const Screw& sc = c.screws[i];
        std::string sign, adj = "00:00";
        if (i == base) {
            snprintf(b, sizeof b, "%s (base) : x=%.1f, y=%.1f, z=%.5f\n", sc.name.c_str(), sc.x, sc.y, z[i]);
            sign = cw_thread ? "CW" : "CCW";
        } else {
            double diff = z[base] - z[i];
            double adjust = std::fabs(diff) < 0.001 ? 0 : diff / factor[c.thread & 7];
            sign = cw_thread ? (adjust >= 0 ? "CW" : "CCW") : (adjust >= 0 ? "CCW" : "CW");
            adjust = std::fabs(adjust);
            int full = int(std::trunc(adjust));
            int minutes = int(std::nearbyint((adjust - full) * 60));   // Python round(): halves to even
            char a[16];
            snprintf(a, sizeof a, "%02d:%02d", full, minutes);
            adj = a;
            snprintf(b, sizeof b, "%s : x=%.1f, y=%.1f, z=%.5f : adjust %s %s\n", sc.name.c_str(), sc.x, sc.y, z[i],
                     sign.c_str(), adj.c_str());
        }
        s += b;
        snprintf(b, sizeof b, "%s{\"name\":\"%s\",\"x\":%.1f,\"y\":%.1f,\"z\":%.5f,\"sign\":\"%s\",\"adjust\":\"%s\",\"base\":%s}",
                 i ? "," : "", sc.name.c_str(), sc.x, sc.y, z[i], sign.c_str(), adj.c_str(), i == base ? "true" : "false");
        js += b;
    }
    js += "]";
    if (json) *json = js;
    return s;
}

}  // namespace ph
