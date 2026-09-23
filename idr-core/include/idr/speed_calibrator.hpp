#pragma once

#include "idr/types.hpp"
#include <tuple>
#include <cmath>
#include <algorithm>

namespace idr {

struct CalibratorState {
    double scale = 1.0;
    double bias_mps = 0.0;
    double scale_std = 0.1;
    double bias_std = 0.5;
    int sample_count = 0;
    bool is_calibrated = false;
    double last_update_time = 0.0;
};

class OnlineSpeedScaleBiasCalibrator {
public:
    explicit OnlineSpeedScaleBiasCalibrator(
        double forgetting_factor = 0.995,
        int min_samples_for_valid = 25,
        std::pair<double, double> scale_bounds = {0.70, 1.35},
        std::pair<double, double> bias_bounds_mps = {-3.5, 3.5}
    );

    CalibratorState update_gnss_aiding(
        double time_s,
        double ai_speed,
        double gnss_speed,
        bool gnss_valid = true
    );

    void freeze_for_outage();
    void unfreeze();

    double correct_speed(double raw_ai_speed) const;
    CalibratorState get_state() const;
    void reset();

private:

    double forgetting_factor_;
    int min_samples_;
    double scale_min_;
    double scale_max_;
    double bias_min_;
    double bias_max_;

    double theta_scale_ = 1.0;
    double theta_bias_ = 0.0;

    // 2x2 Covariance P
    double P00_ = 0.04;
    double P01_ = 0.0;
    double P10_ = 0.0;
    double P11_ = 0.25;

    int sample_count_ = 0;
    bool is_frozen_ = false;
    double frozen_scale_ = 1.0;
    double frozen_bias_ = 0.0;
    double last_time_ = 0.0;
};

} // namespace idr
