#include "idr/vibration_filter.hpp"
#include <algorithm>
#include <cmath>

namespace idr {

VibrationPotholeFilter::VibrationPotholeFilter(
    double dt,
    double pothole_threshold_g,
    double engine_idle_var_min,
    double engine_idle_var_max,
    std::size_t window_size
)
    : m_dt(dt),
      m_g_val(9.80665),
      m_pothole_thresh(pothole_threshold_g * 9.80665),
      m_idle_var_min(engine_idle_var_min),
      m_idle_var_max(engine_idle_var_max),
      m_window_size(std::min(window_size, MAX_WINDOW_SIZE))
{
    reset();
}

void VibrationPotholeFilter::reset() {
    m_head = 0;
    m_count = 0;
    m_pothole_count = 0;
    m_pothole_cooldown = 0;
}

VibrationState VibrationPotholeFilter::process(const Vector3& accel_b, const Vector3& gyro_b) {
    // Append to ring buffer
    m_accel_buf[m_head] = accel_b;
    m_gyro_buf[m_head] = gyro_b;
    m_head = (m_head + 1) % MAX_WINDOW_SIZE;
    if (m_count < m_window_size) {
        m_count++;
    }

    Vector3 cleaned_accel = accel_b;
    Vector3 cleaned_gyro = gyro_b;

    // Shock Detection & Blanking
    double az = accel_b.z;
    double vertical_deviation = std::abs(az - m_g_val);
    double horizontal_norm = std::hypot(accel_b.x, accel_b.y);

    bool is_pothole = false;
    if (m_pothole_cooldown > 0) {
        m_pothole_cooldown--;
    }

    if ((vertical_deviation > m_pothole_thresh || horizontal_norm > 15.0) && m_pothole_cooldown == 0) {
        is_pothole = true;
        m_pothole_count++;
        m_pothole_cooldown = 5;

        // Blanking / Median replacement from recent samples excluding current
        if (m_count >= 3) {
            std::size_t prev_count = m_count - 1;
            double xs[MAX_WINDOW_SIZE];
            double ys[MAX_WINDOW_SIZE];
            double zs[MAX_WINDOW_SIZE];

            for (std::size_t i = 0; i < prev_count; ++i) {
                // Buffer index for element (head - 2 - i)
                std::size_t idx = (m_head + MAX_WINDOW_SIZE - 2 - i) % MAX_WINDOW_SIZE;
                xs[i] = m_accel_buf[idx].x;
                ys[i] = m_accel_buf[idx].y;
                zs[i] = m_accel_buf[idx].z;
            }

            std::sort(xs, xs + prev_count);
            std::sort(ys, ys + prev_count);
            std::sort(zs, zs + prev_count);

            double med_x = (prev_count % 2 == 1) ? xs[prev_count / 2] : 0.5 * (xs[prev_count / 2 - 1] + xs[prev_count / 2]);
            double med_y = (prev_count % 2 == 1) ? ys[prev_count / 2] : 0.5 * (ys[prev_count / 2 - 1] + ys[prev_count / 2]);
            double med_z = (prev_count % 2 == 1) ? zs[prev_count / 2] : 0.5 * (zs[prev_count / 2 - 1] + zs[prev_count / 2]);

            cleaned_accel.z = m_g_val + 0.1 * (med_z - m_g_val);
            cleaned_accel.x = med_x;
            cleaned_accel.y = med_y;
        }
    }

    // Engine Idling Detection
    double sum_abs_ax = 0.0;
    double sum_abs_ay = 0.0;
    double sum_abs_gz = 0.0;
    double sum_az = 0.0;
    double sum_sq_az = 0.0;
    double sum_ax = 0.0;

    for (std::size_t i = 0; i < m_count; ++i) {
        std::size_t idx = (m_head + MAX_WINDOW_SIZE - 1 - i) % MAX_WINDOW_SIZE;
        const auto& a = m_accel_buf[idx];
        const auto& g = m_gyro_buf[idx];

        sum_abs_ax += std::abs(a.x);
        sum_abs_ay += std::abs(a.y);
        sum_abs_gz += std::abs(g.z);
        sum_az += a.z;
        sum_sq_az += a.z * a.z;
        sum_ax += a.x;
    }

    double inv_n = 1.0 / m_count;
    double mean_horiz_acc = sum_abs_ax * inv_n;
    double mean_lat_acc = sum_abs_ay * inv_n;
    double mean_yaw_rate = sum_abs_gz * inv_n;
    double mean_az = sum_az * inv_n;
    double acc_variance = (sum_sq_az * inv_n) - (mean_az * mean_az);
    if (acc_variance < 0.0) acc_variance = 0.0;

    bool is_idling = false;
    if (m_count >= m_window_size) {
        if (mean_horiz_acc < 0.25 &&
            mean_lat_acc < 0.20 &&
            mean_yaw_rate < 0.03 &&
            acc_variance >= m_idle_var_min && acc_variance <= m_idle_var_max) {
            is_idling = true;
            cleaned_accel.x = 0.0;
            cleaned_accel.y = 0.0;
            cleaned_gyro.z = 0.0;
        }
    }

    // Vibration Energy: mean((ax - mean_ax)^2 + (az - mean_az)^2)
    double mean_ax = sum_ax * inv_n;
    double sum_vib = 0.0;
    for (std::size_t i = 0; i < m_count; ++i) {
        std::size_t idx = (m_head + MAX_WINDOW_SIZE - 1 - i) % MAX_WINDOW_SIZE;
        double d_ax = m_accel_buf[idx].x - mean_ax;
        double d_az = m_accel_buf[idx].z - mean_az;
        sum_vib += d_ax * d_ax + d_az * d_az;
    }
    double vib_energy = sum_vib * inv_n;

    // Spectral Entropy placeholder / estimation
    double spectral_entropy = 0.0;

    VibrationState state;
    state.is_engine_idling = is_idling;
    state.is_pothole_detected = is_pothole;
    state.pothole_event_count = m_pothole_count;
    state.cleaned_accel = cleaned_accel;
    state.cleaned_gyro = cleaned_gyro;
    state.vibration_energy = vib_energy;
    state.spectral_entropy = spectral_entropy;

    return state;
}

} // namespace idr
