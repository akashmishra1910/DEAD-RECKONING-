#pragma once

#include "idr/types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace idr {

struct VibrationState {
    bool is_engine_idling{false};
    bool is_pothole_detected{false};
    uint32_t pothole_event_count{0};
    Vector3 cleaned_accel{};
    Vector3 cleaned_gyro{};
    double vibration_energy{0.0};
    double spectral_entropy{0.0};
};

class VibrationPotholeFilter {
public:
    static constexpr std::size_t MAX_WINDOW_SIZE = 32;

    explicit VibrationPotholeFilter(
        double dt = 0.1,
        double pothole_threshold_g = 2.2,
        double engine_idle_var_min = 0.02,
        double engine_idle_var_max = 0.25,
        std::size_t window_size = 10
    );

    void reset();

    // Process a single IMU sample in vehicle body frame
    // Zero heap allocation, fully deterministic
    VibrationState process(const Vector3& accel_b, const Vector3& gyro_b);

    // Get current window count
    std::size_t buffer_size() const { return m_count; }

    double dt() const { return m_dt; }

private:
    double m_dt;
    double m_g_val;
    double m_pothole_thresh;
    double m_idle_var_min;
    double m_idle_var_max;
    std::size_t m_window_size;

    // Fixed-size circular buffer (zero heap allocation)
    std::array<Vector3, MAX_WINDOW_SIZE> m_accel_buf{};
    std::array<Vector3, MAX_WINDOW_SIZE> m_gyro_buf{};
    std::size_t m_head{0};
    std::size_t m_count{0};

    uint32_t m_pothole_count{0};
    int m_pothole_cooldown{0};
};

} // namespace idr
