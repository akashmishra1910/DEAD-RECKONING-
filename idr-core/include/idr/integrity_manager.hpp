#pragma once

#include "idr/types.hpp"
#include <array>
#include <cstddef>
#include <cmath>
#include <string>
#include <algorithm>
#include <optional>

namespace idr {

enum class IntegrityState {
    HEALTHY,
    DEGRADED,
    MAP_UNCERTAIN,
    IMU_UNCERTAIN,
    GNSS_UNCERTAIN,
    RECOVERING,
    FAULT
};

inline const char* integrity_state_to_string(IntegrityState s) {
    switch (s) {
        case IntegrityState::HEALTHY: return "HEALTHY";
        case IntegrityState::DEGRADED: return "DEGRADED";
        case IntegrityState::MAP_UNCERTAIN: return "MAP_UNCERTAIN";
        case IntegrityState::IMU_UNCERTAIN: return "IMU_UNCERTAIN";
        case IntegrityState::GNSS_UNCERTAIN: return "GNSS_UNCERTAIN";
        case IntegrityState::RECOVERING: return "RECOVERING";
        case IntegrityState::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Channel Normalized Innovation Squared (NIS = r^T S^-1 r) tracking with hysteresis.
 */
struct ChannelNISStats {
    const char* channel_name = "";
    int dof = 1;
    double chi2_gate = 9.0;
    double current_nis = 0.0;
    double rolling_nis = 0.0;
    std::array<double, 10> history{};
    size_t history_count = 0;
    size_t history_head = 0;
    int rejection_count = 0;
    int consecutive_rejections = 0;
    int acceptance_count = 0;
    bool last_accepted = true;

    bool update(double nis) {
        current_nis = nis;
        history[history_head] = nis;
        history_head = (history_head + 1) % 10;
        if (history_count < 10) {
            history_count++;
        }

        double sum = 0.0;
        for (size_t i = 0; i < history_count; ++i) {
            sum += history[i];
        }
        rolling_nis = (history_count > 0) ? (sum / history_count) : 0.0;

        bool accepted = (current_nis <= chi2_gate);
        if (accepted) {
            acceptance_count++;
            consecutive_rejections = 0;
            last_accepted = true;
        } else {
            rejection_count++;
            consecutive_rejections++;
            last_accepted = false;
        }
        return accepted;
    }

    void reset() {
        current_nis = 0.0;
        rolling_nis = 0.0;
        history.fill(0.0);
        history_count = 0;
        history_head = 0;
        rejection_count = 0;
        consecutive_rejections = 0;
        acceptance_count = 0;
        last_accepted = true;
    }
};

/**
 * @brief Complete snapshot of navigation integrity.
 */
struct IntegrityReport {
    IntegrityState state = IntegrityState::HEALTHY;
    const char* state_str = "HEALTHY";
    double map_confidence = 0.0;
    bool is_map_usable = false;
    double pos_cov_trace = 0.0;
    double vel_cov_trace = 0.0;
    double yaw_std_deg = 0.0;
    double gnss_pos_nis = 0.0;
    double gnss_vel_nis = 0.0;
    double ai_speed_nis = 0.0;
    double road_heading_nis = 0.0;
    double road_cross_nis = 0.0;
    int total_rejections = 0;
    bool fault_flag = false;
};

/**
 * @brief Production Navigation Integrity Supervisor.
 */
class IntegrityManager {
public:
    IntegrityManager(
        double chi2_gnss_pos = 11.345,
        double chi2_gnss_vel = 9.0,
        double chi2_ai_speed = 6.635,
        double chi2_road_heading = 9.0,
        double chi2_road_cross = 16.0,
        double chi2_nhc = 9.0,
        double max_pos_cov_trace = 1000.0,
        double max_yaw_std_deg = 45.0,
        int consecutive_rejection_limit = 5
    );

    void reset();

    static double compute_continuous_map_confidence(
        double d_2d,
        double delta_h,
        double d_psi,
        double score_margin,
        double yaw_variance,
        int consecutive_matches,
        double elevation_barrier = 4.0,
        double sigma_d = 5.0,
        double sigma_psi = 0.30
    );

    IntegrityReport evaluate_step(
        double time_s,
        bool in_outage,
        bool gnss_valid,
        const Matrix15x15& P_cov,
        bool map_match_valid,
        double map_d_2d,
        double map_delta_h,
        double map_road_heading,
        double map_score_margin,
        int map_consecutive_matches,
        std::optional<double> speed_innovation_m_s,
        std::optional<double> speed_innov_var,
        std::optional<double> gnss_pos_nis,
        std::optional<double> gnss_vel_nis,
        std::optional<double> road_hdg_nis,
        std::optional<double> road_cross_nis,
        double vib_energy = 0.0,
        double turn_rate = 0.0
    );

    IntegrityState get_state() const { return current_state_; }
    const ChannelNISStats& get_channel(size_t idx) const { return channels_[idx]; }

private:
    double chi2_gnss_pos_;
    double chi2_gnss_vel_;
    double chi2_ai_speed_;
    double chi2_road_heading_;
    double chi2_road_cross_;
    double chi2_nhc_;
    double max_pos_cov_trace_;
    double max_yaw_std_deg_;
    int consecutive_rejection_limit_;

    // Channels: 0:gnss_pos, 1:gnss_vel, 2:ai_speed, 3:road_heading, 4:road_cross, 5:nhc_lat, 6:nhc_vert
    std::array<ChannelNISStats, 7> channels_;
    IntegrityState current_state_{IntegrityState::HEALTHY};
    int recovery_counter_{0};
    int required_recovery_epochs_{3};
};

} // namespace idr
