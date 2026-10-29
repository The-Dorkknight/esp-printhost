// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// See resonance.h. The algorithms are Klipper's (resonance_tester.py, shaper_calibrate.py, Dmitry
// Butyugin, GPLv3), rewritten in C++; the inner loops are in float for the ESP32-S3's FPU.
#include "ph/resonance.h"

#include "ph/shaping.h"

#include <algorithm>
#include <cmath>

namespace ph {

// ---------------------------------------------------------------- the test motion

std::vector<TestPoint> resonance_test_sequence(const ResonanceParams& p) {
    // VibrationPulseTestGenerator.gen_test
    std::vector<TestPoint> vib;
    double freq = p.min_freq, sign = 1, time = 0;
    while (freq <= p.max_freq + 0.000001) {
        double t_seg = 0.25 / freq, accel = p.accel_per_hz * freq;
        time += t_seg;
        vib.push_back({time, sign * accel, freq});
        time += t_seg;
        vib.push_back({time, -sign * accel, freq});
        freq += 2 * t_seg * p.hz_per_sec;
        sign = -sign;
    }
    // SweepingVibrationsTestGenerator.gen_test
    double accel_fraction = std::sqrt(2.0) * 0.125, t_rem, sweeping_accel;
    if (p.sweeping_period > 0) {
        t_rem = p.sweeping_period * accel_fraction;
        sweeping_accel = p.sweeping_accel;
    } else {
        t_rem = 1e300;
        sweeping_accel = 0;
    }
    std::vector<TestPoint> res;
    double last_t = 0, sig = 1;
    accel_fraction += 0.25;
    for (auto& v : vib) {
        double t_seg = v.t - last_t;
        while (t_rem <= t_seg) {
            last_t += t_rem;
            res.push_back({last_t, v.accel + sweeping_accel * sig, v.freq});
            t_seg -= t_rem;
            t_rem = p.sweeping_period * accel_fraction;
            accel_fraction = 0.5;
            sig = -sig;
        }
        t_rem -= t_seg;
        res.push_back({v.t, v.accel + sweeping_accel * sig, v.freq});
        last_t = v.t;
    }
    return res;
}

ResonanceMotion resonance_motion(const std::vector<TestPoint>& seq, double end_accel) {
    ResonanceMotion m;
    double x = 0, v = 0, last_t = 0;
    auto add = [&](double t0, double dur, double a) {
        m.pieces.push_back({t0, x, v, a});
        x += v * dur + 0.5 * a * dur * dur;
        v += a * dur;
        m.max_a = std::max(m.max_a, std::fabs(a));
    };
    for (auto& p : seq) {   // run_test: each piece accelerates at `accel` from the last speed
        double dt = p.t - last_t;
        if (dt > 0) add(last_t, dt, p.accel);
        if (std::fabs(v) < 1e-6) v = 0;
        last_t = p.t;
    }
    if (v != 0) {   // then a stop at the printer's own acceleration
        double a = v > 0 ? -std::fabs(end_accel) : std::fabs(end_accel);
        double dt = std::fabs(v / a);
        add(last_t, dt, a);
        last_t += dt;
        v = 0;
    }
    m.end_t = last_t;
    m.end_x = x;
    // extremes: at the ends of pieces and where the speed passes zero
    for (auto& pc : m.pieces) {
        m.min_x = std::min(m.min_x, pc.x0), m.max_x = std::max(m.max_x, pc.x0);
        m.max_v = std::max(m.max_v, std::fabs(pc.v0));
        if (pc.a != 0) {
            double tz = -pc.v0 / pc.a;
            size_t i = size_t(&pc - &m.pieces[0]);
            double dur = (i + 1 < m.pieces.size() ? m.pieces[i + 1].t0 : m.end_t) - pc.t0;
            if (tz > 0 && tz < dur) {
                double xz = pc.x0 + pc.v0 * tz + 0.5 * pc.a * tz * tz;
                m.min_x = std::min(m.min_x, xz), m.max_x = std::max(m.max_x, xz);
            }
        }
    }
    m.min_x = std::min(m.min_x, m.end_x), m.max_x = std::max(m.max_x, m.end_x);
    return m;
}

size_t ResonanceMotion::piece_at(double t) const {
    size_t lo = 0, hi = pieces.size();
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (pieces[mid].t0 <= t) lo = mid;
        else hi = mid;
    }
    return lo;
}

double ResonanceMotion::pos(double t, double* vel) const {
    if (pieces.empty() || t >= end_t) {
        if (vel) *vel = 0;
        return end_x;
    }
    if (t <= 0) {
        if (vel) *vel = 0;
        return 0;
    }
    const Piece& p = pieces[piece_at(t)];
    double dt = t - p.t0;
    if (vel) *vel = p.v0 + p.a * dt;
    return p.x0 + p.v0 * dt + 0.5 * p.a * dt * dt;
}

// ---------------------------------------------------------------- Welch PSD

static double bessel_i0(double x) {
    double sum = 1, term = 1, q = x * x / 4;
    for (int k = 1; k < 60; k++) {
        term *= q / (double(k) * k);
        sum += term;
        if (term < sum * 1e-17) break;
    }
    return sum;
}

void PsdAccumulator::start(double rate) {
    // shaper_calibrate.py: M = 1 << int(SAMPLING_FREQ * WINDOW_T_SEC - 1).bit_length()
    int v = int(rate * 0.5 - 1), bits = 0;
    while (v > 0) bits++, v >>= 1;
    m_ = 1 << bits;
    if (m_ < 64) m_ = 64;
    for (auto& r : ring_) r.assign(size_t(m_), 0.f);
    for (auto& a : acc_) a.assign(size_t(m_ / 2 + 1), 0.0);
    win_.resize(size_t(m_));
    re_.resize(size_t(m_));
    im_.resize(size_t(m_));
    cos_.resize(size_t(m_ / 2));
    sin_.resize(size_t(m_ / 2));
    double i0b = bessel_i0(6.0), wsum = 0;
    for (int n = 0; n < m_; n++) {   // numpy.kaiser(M, 6)
        double r = 2.0 * n / (m_ - 1) - 1.0;
        double w = bessel_i0(6.0 * std::sqrt(std::max(0.0, 1 - r * r))) / i0b;
        win_[size_t(n)] = float(w);
        wsum += w * w;
    }
    wscale_ = 1.0 / wsum;
    for (int k = 0; k < m_ / 2; k++) {
        cos_[size_t(k)] = float(std::cos(2 * M_PI * k / m_));
        sin_[size_t(k)] = float(-std::sin(2 * M_PI * k / m_));
    }
    n_ = since_ = windows_ = 0;
}

size_t PsdAccumulator::bytes() const {
    return size_t(m_) * (3 + 3) * sizeof(float) + size_t(m_) * sizeof(float) + size_t(m_ / 2 + 1) * 3 * sizeof(double);
}

void PsdAccumulator::add(float x, float y, float z) {
    if (!m_) return;
    size_t i = n_ % size_t(m_);
    ring_[0][i] = x, ring_[1][i] = y, ring_[2][i] = z;
    n_++;
    // windows start at 0, M/2, M, ... (shaper_calibrate.py _split_into_windows, overlap M/2)
    if (n_ >= size_t(m_) && (n_ - size_t(m_)) % size_t(m_ / 2) == 0) window();
}

void PsdAccumulator::window() {
    const size_t M = size_t(m_), start = n_ - M;
    for (int ax = 0; ax < 3; ax++) {
        double mean = 0;
        for (size_t k = 0; k < M; k++) mean += ring_[ax][(start + k) % M];
        mean /= double(M);
        for (size_t k = 0; k < M; k++) {
            re_[k] = (ring_[ax][(start + k) % M] - float(mean)) * win_[k];
            im_[k] = 0;
        }
        // in-place radix-2 FFT
        for (size_t i = 1, j = 0; i < M; i++) {
            size_t bit = M >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(re_[i], re_[j]), std::swap(im_[i], im_[j]);
        }
        for (size_t len = 2; len <= M; len <<= 1) {
            size_t half = len / 2, step = M / len;
            for (size_t i = 0; i < M; i += len)
                for (size_t k = 0; k < half; k++) {
                    float wr = cos_[k * step], wi = sin_[k * step];
                    float ur = re_[i + k], ui = im_[i + k];
                    float vr = re_[i + k + half] * wr - im_[i + k + half] * wi;
                    float vi = re_[i + k + half] * wi + im_[i + k + half] * wr;
                    re_[i + k] = ur + vr, im_[i + k] = ui + vi;
                    re_[i + k + half] = ur - vr, im_[i + k + half] = ui - vi;
                }
        }
        for (size_t k = 0; k <= M / 2; k++) acc_[ax][k] += double(re_[k]) * re_[k] + double(im_[k]) * im_[k];
    }
    windows_++;
}

bool PsdAccumulator::finish(double rate, std::vector<double>& freqs, std::vector<double> psd[4]) {
    if (!m_ || windows_ == 0 || rate <= 0) return false;
    size_t K = size_t(m_ / 2 + 1);
    freqs.resize(K);
    for (size_t k = 0; k < K; k++) freqs[k] = double(k) * rate / m_;   // numpy.fft.rfftfreq
    for (int ax = 0; ax < 4; ax++) psd[ax].assign(K, 0.0);
    for (int ax = 0; ax < 3; ax++)
        for (size_t k = 0; k < K; k++) {
            double v = acc_[ax][k] / double(windows_) * wscale_ / rate;
            if (k != 0 && k != K - 1) v *= 2;   // one-sided
            psd[ax][k] = v;
            psd[3][k] += v;
        }
    return true;
}

void CalibrationData::normalize_to_frequencies() {
    const double MIN_FREQ = 5;
    for (auto& p : psd)
        for (size_t k = 0; k < p.size() && k < freqs.size(); k++) {
            p[k] /= freqs[k] + 0.1;
            if (freqs[k] < 2 * MIN_FREQ) p[k] *= std::exp(-std::pow(2 * MIN_FREQ / (freqs[k] + 0.1), 2) + 1);
        }
}

// ---------------------------------------------------------------- the fit

double shaper_smoothing(const std::vector<double>& A, const std::vector<double>& T, double accel, double scv) {
    double half_accel = accel * 0.5, sum = 0, ts = 0;
    for (double a : A) sum += a;
    double inv_D = 1.0 / sum;
    for (size_t i = 0; i < A.size(); i++) ts += A[i] * T[i];
    ts *= inv_D;
    double o90 = 0, o180 = 0;
    for (size_t i = 0; i < A.size(); i++) {
        if (T[i] >= ts) o90 += A[i] * (scv + half_accel * (T[i] - ts)) * (T[i] - ts);
        o180 += A[i] * half_accel * (T[i] - ts) * (T[i] - ts);
    }
    o90 *= inv_D * std::sqrt(2.0);
    o180 *= inv_D;
    return std::max(o90, o180);
}

double shaper_max_accel(const std::vector<double>& A, const std::vector<double>& T, double scv) {
    const double TARGET = 0.12;   // shaper_calibrate.py _bisect
    auto ok = [&](double a) { return shaper_smoothing(A, T, a, scv) <= TARGET; };
    double left = 1, right = 1;
    if (!ok(1e-9)) return 0;
    while (!ok(left)) right = left, left *= 0.5;
    if (right == left)
        while (ok(right)) right *= 2;
    while (right - left > 1e-8) {
        double mid = (left + right) * 0.5;
        if (ok(mid)) left = mid;
        else right = mid;
    }
    return left;
}

namespace {
// _estimate_remaining_vibrations: the shaper's response at each frequency bin, times the spectrum,
// above 1/20 of its peak.
double remaining(const std::vector<double>& A, const std::vector<double>& T, double dr, const std::vector<float>& omega,
                 const std::vector<float>& psd, float thr, double all) {
    double inv_D = 0;
    for (double a : A) inv_D += a;
    inv_D = 1.0 / inv_D;
    const float sq = float(std::sqrt(1.0 - dr * dr));
    const double Tl = T.back();
    double rem = 0;
    for (size_t k = 0; k < omega.size(); k++) {
        float w = omega[k], damping = float(dr) * w, wd = w * sq, S = 0, C = 0;
        for (size_t i = 0; i < A.size(); i++) {
            float W = float(A[i]) * std::exp(-damping * float(Tl - T[i]));
            float s, c, ang = wd * float(T[i]);
            s = std::sin(ang), c = std::cos(ang);
            S += W * s, C += W * c;
        }
        float val = std::sqrt(S * S + C * C) * float(inv_D);
        float r = val * psd[k] - thr;
        if (r > 0) rem += r;
    }
    return rem / all;
}
}  // namespace

ShaperFit find_best_shaper(const CalibrationData& d, double scv, double max_freq, std::vector<ShaperFit>* all_out,
                           double max_smoothing, void (*progress)(int, int)) {
    static const char* const AUTOTUNE[5] = {"zv", "mzv", "ei", "2hump_ei", "3hump_ei"};
    static const double DRS[3] = {0.075, 0.1, 0.15};
    const double MAX_SHAPER_FREQ = 150;
    ShaperFit best;
    bool have_best = false;
    for (int si = 0; si < 5; si++) {
        if (progress) progress(si, 5);
        std::string name = AUTOTUNE[si];
        double f_end = MAX_SHAPER_FREQ, f_start = std::min(shaper_min_freq(name), f_end - 1e-7);
        std::vector<double> tf;   // numpy.arange(f_start, f_end, 0.2)
        size_t nf = size_t(std::ceil((f_end - f_start) / 0.2));
        for (size_t i = 0; i < nf; i++) tf.push_back(f_start + 0.2 * double(i));
        double mf = std::max(max_freq > 0 ? max_freq : 200.0, tf.back());
        std::vector<float> omega, psd;
        float pmax = 0;
        for (size_t k = 0; k < d.freqs.size(); k++)
            if (d.freqs[k] <= mf) {
                omega.push_back(float(2 * M_PI * d.freqs[k]));
                psd.push_back(float(d.psd[3][k]));
                pmax = std::max(pmax, psd.back());
            }
        float thr = pmax / float(SHAPER_VIBRATION_REDUCTION);
        double all = 0;
        for (float p : psd) all += std::max(0.0f, p - thr);
        if (all <= 0) all = 1e-30;
        std::vector<ShaperFit> results;
        ShaperFit best_res;
        bool have = false, stopped = false;
        for (size_t fi = tf.size(); fi-- > 0 && !stopped;) {   // from high to low frequency
            std::vector<double> A, T;
            shaper_def(name, tf[fi], DEFAULT_DAMPING_RATIO, A, T);
            ShaperFit r;
            r.name = name;
            r.freq = tf[fi];
            r.smoothing = shaper_smoothing(A, T, 5000, scv);
            if (max_smoothing > 0 && r.smoothing > max_smoothing && have) {
                stopped = true;
                break;
            }
            for (double dr : DRS) r.vibrs = std::max(r.vibrs, remaining(A, T, dr, omega, psd, thr, all));
            r.max_accel = shaper_max_accel(A, T, scv);
            r.score = r.smoothing * (std::pow(r.vibrs, 1.5) + r.vibrs * 0.2 + 0.01);
            results.push_back(r);
            if (!have || best_res.vibrs > r.vibrs) best_res = r, have = true;
        }
        ShaperFit sel = best_res;
        for (size_t i = results.size(); i-- > 0 && !stopped;)   // (Klipper returns best_res as is when it stopped)
            if (results[i].vibrs < best_res.vibrs * 1.1 && results[i].score < sel.score) sel = results[i];
        if (all_out) all_out->push_back(sel);
        if (!have_best || sel.score * 1.2 < best.score ||
            (sel.score * 1.05 < best.score && sel.smoothing * 1.1 < best.smoothing)) {
            best = sel;
            have_best = true;
        }
    }
    if (progress) progress(5, 5);
    return best;
}

}  // namespace ph
