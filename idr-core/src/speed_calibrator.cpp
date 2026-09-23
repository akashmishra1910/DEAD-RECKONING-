#include "idr/speed_calibrator.hpp"

namespace idr {

OnlineSpeedScaleBiasCalibrator::OnlineSpeedScaleBiasCalibrator(
    double forgetting_factor,
    int min_samples_for_valid,
    std::pair<double, double> scale_bounds,
    std::pair<double, double> bias_bounds_mps
) : forgetting_factor_(forgetting_factor),
    min_samples_(min_samples_for_valid),
    scale_min_(scale_bounds.first),
    scale_max_(scale_bounds.second),
    bias_min_(bias_bounds_mps.first),
    bias_max_(bias_bounds_mps.second)
{}

CalibratorState OnlineSpeedScaleBiasCalibrator::update_gnss_aiding(
    double time_s,
    double ai_speed,
    double gnss_speed,
    bool gnss_valid
) {
    last_time_ = time_s;
    is_frozen_ = false;

    if (!gnss_valid || gnss_speed < 1.0 || ai_speed < 1.0) {
        return get_state();
    }

    // H = [ai_speed, 1.0]
    double H0 = ai_speed;
    double H1 = 1.0;
    double y = gnss_speed;

    // HP = H * P
    double HP0 = H0 * P00_ + H1 * P10_;
    double HP1 = H0 * P01_ + H1 * P11_;

    double denom = forgetting_factor_ + (HP0 * H0 + HP1 * H1);
    if (denom < 1e-9) {
        return get_state();
    }

    // K = (P * H^T) / denom
    double PH0 = P00_ * H0 + P01_ * H1;
    double PH1 = P10_ * H0 + P11_ * H1;
    double K0 = PH0 / denom;
    double K1 = PH1 / denom;

    double pred_y = H0 * theta_scale_ + H1 * theta_bias_;
    double residual = y - pred_y;

    theta_scale_ += K0 * residual;
    theta_bias_ += K1 * residual;

    // P = (P - K * HP) / lam
    double lam = forgetting_factor_;
    P00_ = (P00_ - K0 * HP0) / lam;
    P01_ = (P01_ - K0 * HP1) / lam;
    P10_ = (P10_ - K1 * HP0) / lam;
    P11_ = (P11_ - K1 * HP1) / lam;

    // Enforce bounds
    theta_scale_ = std::clamp(theta_scale_, scale_min_, scale_max_);
    theta_bias_ = std::clamp(theta_bias_, bias_min_, bias_max_);

    // Symmetrize
    double p_sym = 0.5 * (P01_ + P10_);
    P01_ = p_sym;
    P10_ = p_sym;
    P00_ = std::max(1e-6, P00_);
    P11_ = std::max(1e-6, P11_);

    sample_count_++;
    return get_state();
}

void OnlineSpeedScaleBiasCalibrator::freeze_for_outage() {
    if (!is_frozen_) {
        frozen_scale_ = theta_scale_;
        frozen_bias_ = theta_bias_;
        is_frozen_ = true;
    }
}

void OnlineSpeedScaleBiasCalibrator::unfreeze() {
    is_frozen_ = false;
}

double OnlineSpeedScaleBiasCalibrator::correct_speed(double raw_ai_speed) const {
    if (raw_ai_speed < 0.2) return 0.0;
    if (sample_count_ < min_samples_) return raw_ai_speed;

    double active_scale = is_frozen_ ? frozen_scale_ : theta_scale_;
    double active_bias = is_frozen_ ? frozen_bias_ : theta_bias_;

    double corrected = active_scale * raw_ai_speed + active_bias;
    return std::clamp(corrected, 0.0, 40.0);
}

CalibratorState OnlineSpeedScaleBiasCalibrator::get_state() const {
    double active_scale = is_frozen_ ? frozen_scale_ : theta_scale_;
    double active_bias = is_frozen_ ? frozen_bias_ : theta_bias_;

    CalibratorState st;
    st.scale = active_scale;
    st.bias_mps = active_bias;
    st.scale_std = std::sqrt(std::max(0.0, P00_));
    st.bias_std = std::sqrt(std::max(0.0, P11_));
    st.sample_count = sample_count_;
    st.is_calibrated = (sample_count_ >= min_samples_);
    st.last_update_time = last_time_;
    return st;
}

void OnlineSpeedScaleBiasCalibrator::reset() {
    theta_scale_ = 1.0;
    theta_bias_ = 0.0;
    P00_ = 0.01;
    P11_ = 0.25;
    P01_ = 0.0;
    sample_count_ = 0;
    last_time_ = 0.0;
    is_frozen_ = false;
    frozen_scale_ = 1.0;
    frozen_bias_ = 0.0;
}

} // namespace idr

