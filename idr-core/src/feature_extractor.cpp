#include "idr/feature_extractor.hpp"
#include <algorithm>
#include <cmath>

namespace idr {

SlidingWindowFeatureExtractor::SlidingWindowFeatureExtractor(double dt)
    : m_dt(dt)
{
    reset();
}

void SlidingWindowFeatureExtractor::reset() {
    m_head = 0;
    m_count = 0;
}

void SlidingWindowFeatureExtractor::push(const Vector3& accel_b, const Vector3& gyro_b) {
    m_accel_buf[m_head] = accel_b;
    m_gyro_buf[m_head] = gyro_b;
    m_head = (m_head + 1) % WINDOW_SIZE;
    if (m_count < WINDOW_SIZE) {
        m_count++;
    }
}

void SlidingWindowFeatureExtractor::compute_57_features(double* out_57) const {
    if (m_count == 0) {
        for (std::size_t i = 0; i < NUM_RAW_FEATURES; ++i) out_57[i] = 0.0;
        return;
    }

    const std::size_t N = m_count;
    const double inv_N = 1.0 / static_cast<double>(N);

    // Order chronological samples from oldest (0) to newest (N-1)
    Vector3 w_acc[WINDOW_SIZE];
    Vector3 w_gyr[WINDOW_SIZE];
    for (std::size_t i = 0; i < N; ++i) {
        std::size_t buf_idx = (m_head + WINDOW_SIZE - N + i) % WINDOW_SIZE;
        w_acc[i] = m_accel_buf[buf_idx];
        w_gyr[i] = m_gyro_buf[buf_idx];
    }

    // 1. Classical statistical moments
    // Accel moments
    Vector3 sum_acc{0.0, 0.0, 0.0};
    Vector3 sum_sq_acc{0.0, 0.0, 0.0};
    Vector3 min_acc = w_acc[0];
    Vector3 max_acc = w_acc[0];
    double sum_abs_acc = 0.0;

    for (std::size_t i = 0; i < N; ++i) {
        const auto& a = w_acc[i];
        sum_acc += a;
        sum_sq_acc += Vector3{a.x * a.x, a.y * a.y, a.z * a.z};
        min_acc.x = std::min(min_acc.x, a.x);
        min_acc.y = std::min(min_acc.y, a.y);
        min_acc.z = std::min(min_acc.z, a.z);
        max_acc.x = std::max(max_acc.x, a.x);
        max_acc.y = std::max(max_acc.y, a.y);
        max_acc.z = std::max(max_acc.z, a.z);
        sum_abs_acc += std::abs(a.x) + std::abs(a.y) + std::abs(a.z);
    }

    Vector3 mean_acc = sum_acc * inv_N;
    Vector3 var_acc{
        std::max(0.0, (sum_sq_acc.x * inv_N) - (mean_acc.x * mean_acc.x)),
        std::max(0.0, (sum_sq_acc.y * inv_N) - (mean_acc.y * mean_acc.y)),
        std::max(0.0, (sum_sq_acc.z * inv_N) - (mean_acc.z * mean_acc.z))
    };
    Vector3 std_acc{std::sqrt(var_acc.x), std::sqrt(var_acc.y), std::sqrt(var_acc.z)};
    Vector3 rms_acc{
        std::sqrt(sum_sq_acc.x * inv_N),
        std::sqrt(sum_sq_acc.y * inv_N),
        std::sqrt(sum_sq_acc.z * inv_N)
    };
    Vector3 ptp_acc = max_acc - min_acc;

    // Gyro moments
    Vector3 sum_gyr{0.0, 0.0, 0.0};
    Vector3 sum_sq_gyr{0.0, 0.0, 0.0};
    Vector3 min_gyr = w_gyr[0];
    Vector3 max_gyr = w_gyr[0];
    double sum_abs_gyr = 0.0;

    for (std::size_t i = 0; i < N; ++i) {
        const auto& g = w_gyr[i];
        sum_gyr += g;
        sum_sq_gyr += Vector3{g.x * g.x, g.y * g.y, g.z * g.z};
        min_gyr.x = std::min(min_gyr.x, g.x);
        min_gyr.y = std::min(min_gyr.y, g.y);
        min_gyr.z = std::min(min_gyr.z, g.z);
        max_gyr.x = std::max(max_gyr.x, g.x);
        max_gyr.y = std::max(max_gyr.y, g.y);
        max_gyr.z = std::max(max_gyr.z, g.z);
        sum_abs_gyr += std::abs(g.x) + std::abs(g.y) + std::abs(g.z);
    }

    Vector3 mean_gyr = sum_gyr * inv_N;
    Vector3 var_gyr{
        std::max(0.0, (sum_sq_gyr.x * inv_N) - (mean_gyr.x * mean_gyr.x)),
        std::max(0.0, (sum_sq_gyr.y * inv_N) - (mean_gyr.y * mean_gyr.y)),
        std::max(0.0, (sum_sq_gyr.z * inv_N) - (mean_gyr.z * mean_gyr.z))
    };
    Vector3 std_gyr{std::sqrt(var_gyr.x), std::sqrt(var_gyr.y), std::sqrt(var_gyr.z)};
    Vector3 rms_gyr{
        std::sqrt(sum_sq_gyr.x * inv_N),
        std::sqrt(sum_sq_gyr.y * inv_N),
        std::sqrt(sum_sq_gyr.z * inv_N)
    };
    Vector3 ptp_gyr = max_gyr - min_gyr;

    double sma_acc = sum_abs_acc / (3.0 * static_cast<double>(N));
    double sma_gyr = sum_abs_gyr / (3.0 * static_cast<double>(N));

    // 2. Dynamic gravity alignment
    double g_est = mean_acc.norm() + 1e-6;
    Vector3 g_hat = mean_acc / g_est;

    double a_dyn_vert[WINDOW_SIZE];
    double a_dyn_horiz_norm[WINDOW_SIZE];
    double w_yaw[WINDOW_SIZE];
    double w_tilt_norm[WINDOW_SIZE];

    double sum_dyn_vert = 0.0;
    double sum_sq_dyn_vert = 0.0;
    double sum_dyn_horiz_norm = 0.0;
    double sum_sq_dyn_horiz_norm = 0.0;

    double sum_abs_yaw = 0.0;
    double sum_sq_yaw = 0.0;
    double sum_yaw = 0.0;
    double sum_tilt_norm = 0.0;

    double sum_diff_acc_norm = 0.0;
    double acc_norm[WINDOW_SIZE];
    double sum_acc_norm = 0.0;
    double sum_sq_acc_norm = 0.0;

    for (std::size_t i = 0; i < N; ++i) {
        Vector3 diff_a = w_acc[i] - mean_acc;
        double d_norm = diff_a.norm();
        sum_diff_acc_norm += d_norm;

        double a_n = w_acc[i].norm();
        acc_norm[i] = a_n;
        sum_acc_norm += a_n;
        sum_sq_acc_norm += a_n * a_n;

        double vert = diff_a.dot(g_hat);
        a_dyn_vert[i] = vert;
        sum_dyn_vert += vert;
        sum_sq_dyn_vert += vert * vert;

        Vector3 horiz = diff_a - (vert * g_hat);
        double h_norm = horiz.norm();
        a_dyn_horiz_norm[i] = h_norm;
        sum_dyn_horiz_norm += h_norm;
        sum_sq_dyn_horiz_norm += h_norm * h_norm;

        double y = w_gyr[i].dot(g_hat);
        w_yaw[i] = y;
        sum_yaw += y;
        sum_abs_yaw += std::abs(y);
        sum_sq_yaw += y * y;

        Vector3 tilt = w_gyr[i] - (y * g_hat);
        double t_norm = tilt.norm();
        w_tilt_norm[i] = t_norm;
        sum_tilt_norm += t_norm;
    }

    double recent_yaw = std::abs(w_yaw[N - 1]);
    double recent_a_horiz = a_dyn_horiz_norm[N - 1];
    double kinematic_v_hint = (recent_yaw > 0.04)
        ? std::clamp(recent_a_horiz / (recent_yaw + 1e-3), 0.0, 35.0)
        : 0.0;

    double mean_dyn_vert = sum_dyn_vert * inv_N;
    double var_vert = std::max(0.0, (sum_sq_dyn_vert * inv_N) - (mean_dyn_vert * mean_dyn_vert));

    double mean_dyn_horiz_norm = sum_dyn_horiz_norm * inv_N;
    double var_horiz = std::max(0.0, (sum_sq_dyn_horiz_norm * inv_N) - (mean_dyn_horiz_norm * mean_dyn_horiz_norm));

    double vib_ratio = var_vert / (var_horiz + 1e-3);

    // Zero Crossing Rates
    double zcr_vert = 0.0;
    double zcr_horiz = 0.0;
    if (N > 1) {
        int zcr_v_count = 0;
        int zcr_h_count = 0;
        for (std::size_t i = 0; i < N - 1; ++i) {
            if ((a_dyn_vert[i + 1] * a_dyn_vert[i]) < 0.0) {
                zcr_v_count++;
            }
            double dh0 = a_dyn_horiz_norm[i] - mean_dyn_horiz_norm;
            double dh1 = a_dyn_horiz_norm[i + 1] - mean_dyn_horiz_norm;
            if ((dh1 * dh0) < 0.0) {
                zcr_h_count++;
            }
        }
        zcr_vert = static_cast<double>(zcr_v_count) / static_cast<double>(N - 1);
        zcr_horiz = static_cast<double>(zcr_h_count) / static_cast<double>(N - 1);
    }

    double mean_acc_norm = sum_acc_norm * inv_N;
    double roughness = std::sqrt(std::max(0.0, (sum_sq_acc_norm * inv_N) - (mean_acc_norm * mean_acc_norm)));
    double dyn_acc_norm = sum_diff_acc_norm * inv_N;
    double norm_vib_energy = dyn_acc_norm / (roughness + 0.1);

    double mean_yaw_rate = sum_abs_yaw * inv_N;
    double mean_yaw = sum_yaw * inv_N;
    double std_yaw_rate = std::sqrt(std::max(0.0, (sum_sq_yaw * inv_N) - (mean_yaw * mean_yaw)));
    double mean_tilt_rate = sum_tilt_norm * inv_N;
    double pitch_sin_hint = std::clamp((g_est - 9.80665) / 9.80665, -0.3, 0.3);

    // Assign features 0..38 (39 baseline features)
    out_57[0] = mean_acc.x;
    out_57[1] = mean_acc.y;
    out_57[2] = mean_acc.z;
    out_57[3] = std_acc.x;
    out_57[4] = std_acc.y;
    out_57[5] = std_acc.z;
    out_57[6] = rms_acc.x;
    out_57[7] = rms_acc.y;
    out_57[8] = rms_acc.z;
    out_57[9] = ptp_acc.x;
    out_57[10] = ptp_acc.y;
    out_57[11] = ptp_acc.z;

    out_57[12] = mean_gyr.x;
    out_57[13] = mean_gyr.y;
    out_57[14] = mean_gyr.z;
    out_57[15] = std_gyr.x;
    out_57[16] = std_gyr.y;
    out_57[17] = std_gyr.z;
    out_57[18] = rms_gyr.x;
    out_57[19] = rms_gyr.y;
    out_57[20] = rms_gyr.z;
    out_57[21] = ptp_gyr.x;
    out_57[22] = ptp_gyr.y;
    out_57[23] = ptp_gyr.z;

    out_57[24] = sma_acc;
    out_57[25] = sma_gyr;
    out_57[26] = kinematic_v_hint;
    out_57[27] = var_vert;
    out_57[28] = var_horiz;
    out_57[29] = vib_ratio;
    out_57[30] = zcr_vert;
    out_57[31] = zcr_horiz;
    out_57[32] = roughness;
    out_57[33] = dyn_acc_norm;
    out_57[34] = norm_vib_energy;
    out_57[35] = mean_yaw_rate;
    out_57[36] = std_yaw_rate;
    out_57[37] = mean_tilt_rate;
    out_57[38] = pitch_sin_hint;

    // Jerk and Angular Rate Derivatives (18 features: 39..56)
    if (N > 1) {
        double eff_dt = std::max(m_dt, 1e-4);
        double inv_dt = 1.0 / eff_dt;
        const std::size_t M = N - 1;
        const double inv_M = 1.0 / static_cast<double>(M);

        Vector3 sum_jerk_a{0.0, 0.0, 0.0};
        Vector3 sum_sq_jerk_a{0.0, 0.0, 0.0};
        Vector3 sum_jerk_g{0.0, 0.0, 0.0};
        Vector3 sum_sq_jerk_g{0.0, 0.0, 0.0};

        for (std::size_t i = 0; i < M; ++i) {
            Vector3 ja = (w_acc[i + 1] - w_acc[i]) * inv_dt;
            Vector3 jg = (w_gyr[i + 1] - w_gyr[i]) * inv_dt;

            sum_jerk_a += ja;
            sum_sq_jerk_a += Vector3{ja.x * ja.x, ja.y * ja.y, ja.z * ja.z};
            sum_jerk_g += jg;
            sum_sq_jerk_g += Vector3{jg.x * jg.x, jg.y * jg.y, jg.z * jg.z};
        }

        Vector3 mean_ja = sum_jerk_a * inv_M;
        Vector3 var_ja{
            std::max(0.0, (sum_sq_jerk_a.x * inv_M) - (mean_ja.x * mean_ja.x)),
            std::max(0.0, (sum_sq_jerk_a.y * inv_M) - (mean_ja.y * mean_ja.y)),
            std::max(0.0, (sum_sq_jerk_a.z * inv_M) - (mean_ja.z * mean_ja.z))
        };
        Vector3 std_ja{std::sqrt(var_ja.x), std::sqrt(var_ja.y), std::sqrt(var_ja.z)};
        Vector3 rms_ja{
            std::sqrt(sum_sq_jerk_a.x * inv_M),
            std::sqrt(sum_sq_jerk_a.y * inv_M),
            std::sqrt(sum_sq_jerk_a.z * inv_M)
        };

        Vector3 mean_jg = sum_jerk_g * inv_M;
        Vector3 var_jg{
            std::max(0.0, (sum_sq_jerk_g.x * inv_M) - (mean_jg.x * mean_jg.x)),
            std::max(0.0, (sum_sq_jerk_g.y * inv_M) - (mean_jg.y * mean_jg.y)),
            std::max(0.0, (sum_sq_jerk_g.z * inv_M) - (mean_jg.z * mean_jg.z))
        };
        Vector3 std_jg{std::sqrt(var_jg.x), std::sqrt(var_jg.y), std::sqrt(var_jg.z)};
        Vector3 rms_jg{
            std::sqrt(sum_sq_jerk_g.x * inv_M),
            std::sqrt(sum_sq_jerk_g.y * inv_M),
            std::sqrt(sum_sq_jerk_g.z * inv_M)
        };

        // Indices:
        // 39, 40, 41: mean_ja.x, std_ja.x, rms_ja.x
        // 42, 43, 44: mean_ja.y, std_ja.y, rms_ja.y
        // 45, 46, 47: mean_ja.z, std_ja.z, rms_ja.z
        // 48, 49, 50: mean_jg.x, std_jg.x, rms_jg.x
        // 51, 52, 53: mean_jg.y, std_jg.y, rms_jg.y
        // 54, 55, 56: mean_jg.z, std_jg.z, rms_jg.z
        out_57[39] = mean_ja.x; out_57[40] = std_ja.x; out_57[41] = rms_ja.x;
        out_57[42] = mean_ja.y; out_57[43] = std_ja.y; out_57[44] = rms_ja.y;
        out_57[45] = mean_ja.z; out_57[46] = std_ja.z; out_57[47] = rms_ja.z;

        out_57[48] = mean_jg.x; out_57[49] = std_jg.x; out_57[50] = rms_jg.x;
        out_57[51] = mean_jg.y; out_57[52] = std_jg.y; out_57[53] = rms_jg.y;
        out_57[54] = mean_jg.z; out_57[55] = std_jg.z; out_57[56] = rms_jg.z;
    } else {
        for (std::size_t i = 39; i < 57; ++i) {
            out_57[i] = 0.0;
        }
    }
}

void SlidingWindowFeatureExtractor::compute_45_features(double* out_45) const {
    double raw_57[NUM_RAW_FEATURES];
    compute_57_features(raw_57);

    for (std::size_t i = 0; i < NUM_TOP_FEATURES; ++i) {
        int16_t orig_idx = GBDTSpeedModel::FEATURE_INDICES[i];
        out_45[i] = raw_57[orig_idx];
    }
}

void SlidingWindowFeatureExtractor::compute_45_features(std::array<double, NUM_TOP_FEATURES>& out_45) const {
    compute_45_features(out_45.data());
}

} // namespace idr
