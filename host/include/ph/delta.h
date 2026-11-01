// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// Delta kinematics (2026-10-31), Klipper's delta.py: three towers, the effector hangs from three arms.
//   carriage_i = z + sqrt(arm_i^2 - (x - tower_x_i)^2 - (y - tower_y_i)^2)       (inverse)
//   x, y, z = the lower intersection of three spheres, one round each carriage joint (forward)
// Units: mm. Towers at delta_radius from the centre, at `angle` degrees (A 210, B 330, C 90).
#pragma once
#include <cmath>

namespace ph {

struct DeltaGeom {
    double arm[3] = {0, 0, 0}, arm2[3] = {0, 0, 0};
    double tx[3] = {0, 0, 0}, ty[3] = {0, 0, 0};
    double radius = 0;   // delta_radius
    bool valid = false;

    void set(const double arm_length[3], const double angle_deg[3], double delta_radius) {
        radius = delta_radius;
        for (int i = 0; i < 3; i++) {
            arm[i] = arm_length[i], arm2[i] = arm_length[i] * arm_length[i];
            double a = angle_deg[i] * M_PI / 180.0;
            tx[i] = std::cos(a) * delta_radius, ty[i] = std::sin(a) * delta_radius;
        }
        valid = delta_radius > 0 && arm[0] > delta_radius && arm[1] > delta_radius && arm[2] > delta_radius;
    }

    // The carriage heights (mm, on the tower) that put the effector at x, y, z. False if an arm cannot reach.
    bool inverse(double x, double y, double z, double c[3]) const {
        for (int i = 0; i < 3; i++) {
            double dx = x - tx[i], dy = y - ty[i], r = arm2[i] - dx * dx - dy * dy;
            if (r < 0) return false;
            c[i] = z + std::sqrt(r);
        }
        return true;
    }

    // Where the effector is for these carriage heights (Klipper trilateration). False: no such point.
    bool forward(const double c[3], double out[3]) const {
        double p1[3] = {tx[0], ty[0], c[0]}, p2[3] = {tx[1], ty[1], c[1]}, p3[3] = {tx[2], ty[2], c[2]};
        double s21[3], s31[3];
        for (int k = 0; k < 3; k++) s21[k] = p2[k] - p1[k], s31[k] = p3[k] - p1[k];
        double d = std::sqrt(s21[0] * s21[0] + s21[1] * s21[1] + s21[2] * s21[2]);
        if (d <= 0) return false;
        double ex[3] = {s21[0] / d, s21[1] / d, s21[2] / d};
        double i = ex[0] * s31[0] + ex[1] * s31[1] + ex[2] * s31[2];
        double vy[3] = {s31[0] - ex[0] * i, s31[1] - ex[1] * i, s31[2] - ex[2] * i};
        double ly = std::sqrt(vy[0] * vy[0] + vy[1] * vy[1] + vy[2] * vy[2]);
        if (ly <= 0) return false;
        double ey[3] = {vy[0] / ly, vy[1] / ly, vy[2] / ly};
        double ez[3] = {ex[1] * ey[2] - ex[2] * ey[1], ex[2] * ey[0] - ex[0] * ey[2], ex[0] * ey[1] - ex[1] * ey[0]};
        double j = ey[0] * s31[0] + ey[1] * s31[1] + ey[2] * s31[2];
        double X = (arm2[0] - arm2[1] + d * d) / (2 * d);
        double Y = (arm2[0] - arm2[2] - X * X + (X - i) * (X - i) + j * j) / (2 * j);
        double zz = arm2[0] - X * X - Y * Y;
        if (zz < 0) return false;
        double Z = -std::sqrt(zz);
        for (int k = 0; k < 3; k++) out[k] = p1[k] + ex[k] * X + ey[k] * Y + ez[k] * Z;
        return true;
    }
};

}  // namespace ph
