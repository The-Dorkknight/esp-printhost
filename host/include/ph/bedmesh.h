// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Bed mesh, screw tilt, firmware retraction (2026-10-24; CAPTAIN / SKIPPER builds), after Klipper
// v0.13.0 bed_mesh.py, screws_tilt_adjust.py, firmware_retraction.py:
//   - the mesh is the probed grid itself, bilinear in between (Klipper with mesh_pps: 0, 0); no fade;
//   - screw tilt: Klipper's turns-and-minutes ("CW 01:20" = one turn and a third clockwise);
//   - G10 / G11 retract and unretract by retract_length at retract_speed.
#pragma once
#include <string>
#include <vector>

namespace ph {

struct MeshConfig {   // [bed_mesh]
    bool present = false;
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0;   // probe coordinates
    int count_x = 3, count_y = 3;
    double horizontal_move_z = 5, speed = 50;
};

struct BedMesh {
    int cols = 0, rows = 0;            // x, y
    double min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    std::vector<double> z;             // row-major: row 0 = min_y, column 0 = min_x
    bool valid() const { return cols >= 2 && rows >= 2 && z.size() == size_t(cols * rows); }
    double at(double x, double y) const;   // bilinear, held at the edge outside the mesh
    double point_x(int c) const { return min_x + (max_x - min_x) * c / (cols - 1); }
    double point_y(int r) const { return min_y + (max_y - min_y) * r / (rows - 1); }
    double lowest() const, highest() const, mean() const;
    // Klipper's [bed_mesh <profile>] option values: "points" (rows on their own lines), counts, bounds
    std::vector<std::pair<std::string, std::string>> profile_options() const;
    bool from_profile(const std::string& points, int x_count, int y_count, double min_x, double max_x, double min_y,
                      double max_y, std::string* err);
    // The text map for the console and the Printer page: back of the bed on top, each value in mm,
    // and its level (0 low, 1 medium-low, 2 base, 3 medium-high, 4 high) for the colours.
    std::string text() const;
    int level(double v) const;
    std::string json() const;   // {"cols":,"rows":,"min":[x,y],"max":[x,y],"z":[...],"lvl":[...]}
};

struct Screw {
    std::string name;
    double x = 0, y = 0;
};
struct ScrewsConfig {   // [screws_tilt_adjust]
    bool present = false;
    std::vector<Screw> screws;
    int thread = 0;                    // 0 CW-M3, 1 CCW-M3, 2 CW-M4, 3 CCW-M4, 4 CW-M5, 5 CCW-M5, 6 CW-M6, 7 CCW-M6
    double horizontal_move_z = 10, speed = 50;
};
// Klipper's probe_finalize: the first screw is the base; direction "" / "CW" / "CCW" (base = the
// highest / lowest screw so all turns go one way). Returns the report; json for the web page.
std::string screws_report(const ScrewsConfig& c, const std::vector<double>& z, const std::string& direction,
                          std::string* json);
int screw_thread_index(const std::string& name);   // -1 if unknown

struct RetractConfig {   // [firmware_retraction]
    double length = 0, speed = 20, unretract_extra = 0, unretract_speed = 10;
};

}  // namespace ph
