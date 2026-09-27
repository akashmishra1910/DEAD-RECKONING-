#include "idr/integrity_manager.hpp"

namespace idr {

IntegrityManager::IntegrityManager(
    double chi2_gnss_pos,
    double chi2_gnss_vel,
    double chi2_ai_speed,
    double chi2_road_heading,
    double chi2_road_cross,
    double chi2_nhc,
    double max_pos_cov_trace,
    double max_yaw_std_deg,
    int consecutive_rejection_limit
) : chi2_gnss_pos_(chi2_gnss_pos),
    chi2_gnss_vel_(chi2_gnss_vel),
    chi2_ai_speed_(chi2_ai_speed),
    chi2_road_heading_(chi2_road_heading),
    chi2_road_cross_(chi2_road_cross),
    chi2_nhc_(chi2_nhc),
    max_pos_cov_trace_(max_pos_cov_trace),
    max_yaw_std_deg_(max_yaw_std_deg),
    consecutive_rejection_limit_(consecutive_rejection_limit)
{
    channels_[0] = ChannelNISStats{"gnss_pos", 3, chi2_gnss_pos_};
    channels_[1] = ChannelNISStats{"gnss_vel", 3, chi2_gnss_vel_};
    channels_[2] = ChannelNISStats{"ai_speed", 1, chi2_ai_speed_};
    channels_[3] = ChannelNISStats{"road_heading", 1, chi2_road_heading_};
    channels_[4] = ChannelNISStats{"road_cross", 1, chi2_road_cross_};
    channels_[5] = ChannelNISStats{"nhc_lat", 1, chi2_nhc_};
    channels_[6] = ChannelNISStats{"nhc_vert", 1, chi2_nhc_};
    reset();
}

void IntegrityManager::reset() {
    for (auto& ch : channels_) {
        ch.reset();
    }
    current_state_ = IntegrityState::HEALTHY;
    recovery_counter_ = 0;
}

double IntegrityManager::compute_continuous_map_confidence(
    double d_2d,
    double delta_h,
    double d_psi,
    double score_margin,
    double yaw_variance,
    int consecutive_matches,
    double elevation_barrier,
    double sigma_d,
    double sigma_psi
) {
    // 1. Strict 3D Elevation barrier check (if delta_h > barrier, probability is zero)
    if (delta_h > elevation_barrier) {
        return 0.0;
    }

    // 2. Physical Likelihood Factors (Bayesian multiplicative structure)
    double dist_score = std::exp(-0.5 * (d_2d / sigma_d) * (d_2d / sigma_d));
    double hdg_score = std::exp(-0.5 * (d_psi / sigma_psi) * (d_psi / sigma_psi));
    double elev_score = std::exp(-0.5 * (delta_h / 2.0) * (delta_h / 2.0));
    double margin_score = std::tanh(std::max(0.0, score_margin) / 2.0);

    // Multiplicative spatial-kinematic probability
    double spatial_prob = dist_score * (0.50 * hdg_score + 0.30 * elev_score + 0.20 * margin_score);

    // Filter covariance & temporal stability scaling
    double cov_penalty = std::exp(-std::max(0.0, yaw_variance) / 0.05);
    double temporal_factor = std::min(1.0, 0.5 + 0.1 * consecutive_matches);

    double confidence = spatial_prob * cov_penalty * temporal_factor;
    return std::clamp(confidence, 0.0, 1.0);
}

IntegrityReport IntegrityManager::evaluate_step(
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
    double vib_energy,
    double turn_rate
) {
    (void)time_s;
    (void)turn_rate;
    (void)map_road_heading;

    int fault_count = 0;

    // 1. Update Channel NIS
    if (gnss_pos_nis.has_value()) {
        channels_[0].update(*gnss_pos_nis);
    }
    if (gnss_vel_nis.has_value()) {
        channels_[1].update(*gnss_vel_nis);
    }
    if (speed_innovation_m_s.has_value() && speed_innov_var.has_value() && *speed_innov_var > 1e-6) {
        double nis_spd = (*speed_innovation_m_s * *speed_innovation_m_s) / *speed_innov_var;
        channels_[2].update(nis_spd);
    }
    if (road_hdg_nis.has_value()) {
        channels_[3].update(*road_hdg_nis);
    }
    if (road_cross_nis.has_value()) {
        channels_[4].update(*road_cross_nis);
    }

    // 2. Covariance Consistency
    double pos_cov_trace = P_cov(0, 0) + P_cov(1, 1) + P_cov(2, 2);
    double vel_cov_trace = P_cov(3, 3) + P_cov(4, 4) + P_cov(5, 5);
    double yaw_var = P_cov(8, 8);
    double yaw_std_deg = std::sqrt(std::max(0.0, yaw_var)) * (180.0 / M_PI);

    if (pos_cov_trace > max_pos_cov_trace_) {
        fault_count++;
    }
    if (yaw_std_deg > max_yaw_std_deg_) {
        fault_count++;
    }

    // 3. Continuous Map Confidence
    double map_confidence = 0.0;
    if (map_match_valid) {
        double d_psi = 0.0; // Heading discrepancy if road heading is valid
        map_confidence = compute_continuous_map_confidence(
            map_d_2d,
            map_delta_h,
            d_psi,
            map_score_margin,
            yaw_var,
            map_consecutive_matches
        );
    }
    bool is_map_usable = (map_confidence >= 0.35);

    // 4. Sensor & Channel Health
    bool gnss_fault = false;
    if (!in_outage && gnss_valid) {
        if (channels_[0].consecutive_rejections >= consecutive_rejection_limit_ ||
            channels_[1].consecutive_rejections >= consecutive_rejection_limit_) {
            gnss_fault = true;
            fault_count++;
        }
    }

    bool imu_fault = false;
    if (vib_energy > 25.0 || channels_[2].consecutive_rejections >= consecutive_rejection_limit_) {
        imu_fault = true;
        fault_count++;
    }

    bool map_uncertain = false;
    if (in_outage) {
        if (!is_map_usable || map_confidence < 0.35 ||
            channels_[3].consecutive_rejections >= 3 ||
            channels_[4].consecutive_rejections >= 3) {
            map_uncertain = true;
        }
    }

    // 5. State Machine Transitions
    if (pos_cov_trace > (2.0 * max_pos_cov_trace_) || fault_count >= 3) {
        current_state_ = IntegrityState::FAULT;
    } else if (imu_fault) {
        current_state_ = IntegrityState::IMU_UNCERTAIN;
    } else if (gnss_fault) {
        current_state_ = IntegrityState::GNSS_UNCERTAIN;
    } else if (in_outage) {
        if (map_uncertain) {
            current_state_ = IntegrityState::MAP_UNCERTAIN;
        } else {
            current_state_ = IntegrityState::DEGRADED;
        }
    } else {
        if (!gnss_valid) {
            current_state_ = IntegrityState::DEGRADED;
        } else {
            current_state_ = IntegrityState::HEALTHY;
        }
    }

    int total_rejections = 0;
    for (const auto& ch : channels_) {
        total_rejections += ch.rejection_count;
    }

    IntegrityReport rep;
    rep.state = current_state_;
    rep.state_str = integrity_state_to_string(current_state_);
    rep.map_confidence = map_confidence;
    rep.is_map_usable = is_map_usable;
    rep.pos_cov_trace = pos_cov_trace;
    rep.vel_cov_trace = vel_cov_trace;
    rep.yaw_std_deg = yaw_std_deg;
    rep.gnss_pos_nis = channels_[0].current_nis;
    rep.gnss_vel_nis = channels_[1].current_nis;
    rep.ai_speed_nis = channels_[2].current_nis;
    rep.road_heading_nis = channels_[3].current_nis;
    rep.road_cross_nis = channels_[4].current_nis;
    rep.total_rejections = total_rejections;
    rep.fault_flag = (fault_count > 0);

    return rep;
}

} // namespace idr
