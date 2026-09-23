#include "idr/heading_supervisor.hpp"
#include <algorithm>

namespace idr {

MagneticDisturbanceDetector::MagneticDisturbanceDetector(
    double reference_field_uT,
    double magnitude_tolerance_uT,
    double max_gradient_uT_s,
    double min_field_uT,
    double max_field_uT,
    double declination_rad,
    const Vector3& hard_iron_bias,
    const Matrix3x3& soft_iron_matrix
) : ref_field_(reference_field_uT),
    mag_tol_(magnitude_tolerance_uT),
    max_grad_(max_gradient_uT_s),
    min_field_(min_field_uT),
    max_field_(max_field_uT),
    declination_(declination_rad),
    hard_iron_(hard_iron_bias),
    soft_iron_(soft_iron_matrix)
{
    reset();
}

void MagneticDisturbanceDetector::reset() {
    last_mag_vec_ = std::nullopt;
    last_time_s_ = std::nullopt;
}

std::tuple<HeadingQuality, double, double, double, const char*> MagneticDisturbanceDetector::evaluate_measurement(
    const Vector3& mag_raw_p,
    double time_s,
    const Matrix3x3& R_p2b,
    const Matrix3x3& R_b2n
) {
    if (!std::isfinite(mag_raw_p.x) || !std::isfinite(mag_raw_p.y) || !std::isfinite(mag_raw_p.z)) {
        return {HeadingQuality::INVALID, 0.0, 1.0, 0.0, "INVALID_OR_NAN_INPUT"};
    }

    Vector3 mag_cal_p = soft_iron_ * (mag_raw_p - hard_iron_);
    double norm_cal = mag_cal_p.norm();

    if (norm_cal < min_field_ || norm_cal > max_field_) {
        return {HeadingQuality::INVALID, 0.0, 1.0, norm_cal, "MAGNITUDE_OUT_OF_BOUNDS"};
    }

    double dt = last_time_s_.has_value() ? (time_s - *last_time_s_) : 0.1;
    if (dt > 0.0 && last_mag_vec_.has_value()) {
        double gradient = (mag_cal_p - *last_mag_vec_).norm() / dt;
        if (gradient > max_grad_) {
            last_mag_vec_ = mag_cal_p;
            last_time_s_ = time_s;
            return {HeadingQuality::DISTURBED, 0.0, 1.0, norm_cal, "EXCESSIVE_GRADIENT"};
        }
    }

    last_mag_vec_ = mag_cal_p;
    last_time_s_ = time_s;

    double mag_delta = std::abs(norm_cal - ref_field_);
    if (mag_delta > mag_tol_) {
        return {HeadingQuality::DISTURBED, 0.0, 1.0, norm_cal, "MAGNITUDE_ANOMALY"};
    }

    Vector3 mag_b = R_p2b * mag_cal_p;
    double Bx = mag_b.x;
    double By = mag_b.y;
    double Bz = mag_b.z;

    double pitch = std::asin(std::clamp(R_b2n(2, 0), -1.0, 1.0));
    double roll = std::atan2(R_b2n(2, 1), R_b2n(2, 2));

    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    double B_fx = Bx * cp + By * sp * sr + Bz * sp * cr;
    double B_fy = By * cr - Bz * sr;

    double psi_mag = std::atan2(B_fy, B_fx) + declination_;
    psi_mag = wrap_2pi(psi_mag);

    HeadingQuality quality;
    double std_rad;
    if (mag_delta < 4.0) {
        quality = HeadingQuality::NOMINAL;
        std_rad = 4.0 * DEG_TO_RAD;
    } else {
        quality = HeadingQuality::WEAK;
        std_rad = 10.0 * DEG_TO_RAD;
    }

    return {quality, psi_mag, std_rad, norm_cal, "VALID_MAGNETIC_MEASUREMENT"};
}

GNSSCourseDeriver::GNSSCourseDeriver(
    double min_speed_mps,
    double max_yaw_rate_rad_s,
    double base_velocity_noise_mps
) : min_speed_(min_speed_mps),
    max_yaw_rate_(max_yaw_rate_rad_s),
    base_vel_noise_(base_velocity_noise_mps)
{}

std::tuple<HeadingQuality, double, double, double, const char*> GNSSCourseDeriver::derive_course(
    const Vector3& vel_enu,
    double current_yaw_rate,
    bool is_forward
) {
    if (!std::isfinite(vel_enu.x) || !std::isfinite(vel_enu.y)) {
        return {HeadingQuality::INVALID, 0.0, 1.0, 0.0, "INVALID_VELOCITY_VECTOR"};
    }

    double ve = vel_enu.x;
    double vn = vel_enu.y;
    double speed_horiz = std::sqrt(ve * ve + vn * vn);

    if (speed_horiz < min_speed_) {
        return {HeadingQuality::INVALID, 0.0, 1.0, speed_horiz, "REJECTED_LOW_SPEED"};
    }

    double course_rad = std::atan2(ve, vn);
    if (!is_forward) {
        course_rad += PI;
    }
    course_rad = wrap_2pi(course_rad);

    double sigma_psi = std::clamp(base_vel_noise_ / speed_horiz, 1.0 * DEG_TO_RAD, 15.0 * DEG_TO_RAD);

    if (std::abs(current_yaw_rate) > max_yaw_rate_) {
        return {HeadingQuality::WEAK, course_rad, sigma_psi * 2.5, speed_horiz, "HIGH_YAW_RATE_DYNAMICS"};
    }

    HeadingQuality quality = (speed_horiz > 8.0) ? HeadingQuality::HIGH_ACCURACY : HeadingQuality::NOMINAL;
    return {quality, course_rad, sigma_psi, speed_horiz, "VALID_GNSS_COURSE"};
}

HeadingSupervisor::HeadingSupervisor(
    double chi2_gate,
    double min_gnss_speed_mps,
    double ref_mag_field_uT,
    bool enable_magnetometer,
    bool enable_gnss_course
) : chi2_gate_(chi2_gate),
    enable_mag_(enable_magnetometer),
    enable_gnss_(enable_gnss_course),
    mag_detector_(ref_mag_field_uT),
    gnss_deriver_(min_gnss_speed_mps)
{
    reset();
    candidates_scratch_.reserve(4); // at most 2 sources (GNSS + mag); 4 ensures no realloc
}

void HeadingSupervisor::reset() {
    mag_detector_.reset();
    total_updates_ = 0;
    total_rejections_ = 0;
    last_update_time_s_ = 0.0;
    outage_duration_s_ = 0.0;
    reacquisition_count_ = 0;
    last_candidate_heading_rad_ = std::nullopt;
}

HeadingSupervisorTelemetry HeadingSupervisor::process_step(
    double time_s,
    ErrorStateKalmanFilter& eskf,
    const Matrix3x3& R_p2b,
    const std::optional<Vector3>& gnss_vel,
    bool gnss_valid,
    const std::optional<Vector3>& mag_raw_p,
    double current_yaw_rate,
    bool is_forward
) {
    candidates_scratch_.clear();
    double mag_field_norm = 0.0;
    bool mag_is_disturbed = false;

    if (enable_gnss_ && gnss_valid && gnss_vel.has_value()) {
        auto [g_qual, g_course, g_std, g_spd, g_reason] = gnss_deriver_.derive_course(
            *gnss_vel, current_yaw_rate, is_forward
        );
        if (g_qual == HeadingQuality::NOMINAL || g_qual == HeadingQuality::HIGH_ACCURACY || g_qual == HeadingQuality::WEAK) {
            HeadingMeasurement m;
            m.source = HeadingSourceType::GNSS_COURSE;
            m.heading_rad = g_course;
            m.heading_deg = g_course * RAD_TO_DEG;
            m.std_rad = g_std;
            m.quality = g_qual;
            m.raw_magnitude = g_spd;
            m.rejection_reason = g_reason;
            candidates_scratch_.push_back(m);
        }
    }

    if (enable_mag_ && mag_raw_p.has_value()) {
        auto [m_qual, m_hdg, m_std, m_norm, m_reason] = mag_detector_.evaluate_measurement(
            *mag_raw_p, time_s, R_p2b, eskf.get_rotation_matrix()
        );
        mag_field_norm = m_norm;
        mag_is_disturbed = (m_qual == HeadingQuality::DISTURBED);

        if (m_qual == HeadingQuality::NOMINAL || m_qual == HeadingQuality::WEAK) {
            HeadingMeasurement m;
            m.source = HeadingSourceType::MAGNETOMETER;
            m.heading_rad = m_hdg;
            m.heading_deg = m_hdg * RAD_TO_DEG;
            m.std_rad = m_std;
            m.quality = m_qual;
            m.raw_magnitude = m_norm;
            m.rejection_reason = m_reason;
            candidates_scratch_.push_back(m);
        }
    }

    // Arbitration: HIGH_ACCURACY > NOMINAL > WEAK, then smallest std_rad
    std::optional<HeadingMeasurement> selected = std::nullopt;
    if (!candidates_scratch_.empty()) {
        auto quality_rank = [](HeadingQuality q) {
            if (q == HeadingQuality::HIGH_ACCURACY) return 0;
            if (q == HeadingQuality::NOMINAL) return 1;
            if (q == HeadingQuality::WEAK) return 2;
            return 99;
        };
        std::sort(candidates_scratch_.begin(), candidates_scratch_.end(), [&](const HeadingMeasurement& a, const HeadingMeasurement& b) {
            int ra = quality_rank(a.quality);
            int rb = quality_rank(b.quality);
            if (ra != rb) return ra < rb;
            return a.std_rad < b.std_rad;
        });
        selected = candidates_scratch_[0];
    }

    bool is_updated = false;
    const char* rejection_reason = "NO_VALID_HEADING_SOURCE";
    HeadingSourceType selected_source = HeadingSourceType::GYRO_INTEGRATION;
    HeadingQuality selected_quality = HeadingQuality::INVALID;
    std::optional<double> meas_hdg_deg = std::nullopt;
    std::optional<double> innov_deg = std::nullopt;
    std::optional<double> gamma_val = std::nullopt;

    if (selected.has_value()) {
        selected_source = selected->source;
        selected_quality = selected->quality;
        meas_hdg_deg = selected->heading_deg;

        double dyaw = wrap_pi(selected->heading_rad - eskf.get_yaw());
        innov_deg = dyaw * RAD_TO_DEG;

        double P88 = eskf.get_covariance()(8, 8);
        double S = P88 + (selected->std_rad * selected->std_rad);
        double gamma = (dyaw * dyaw) / S;
        gamma_val = gamma;

        selected->innovation_rad = dyaw;
        selected->innovation_deg = innov_deg;
        selected->innovation_variance = S;
        selected->chi2_normalized = gamma;

        if (gamma <= chi2_gate_) {
            eskf.update_heading(selected->heading_rad, selected->std_rad, std::nullopt);
            is_updated = true;
            selected->is_accepted = true;
            total_updates_++;
            last_update_time_s_ = time_s;
            outage_duration_s_ = 0.0;
            reacquisition_count_ = 0;
            rejection_reason = "ACCEPTED_AND_UPDATED";
        } else {
            if (last_candidate_heading_rad_.has_value() &&
                std::abs(wrap_pi(selected->heading_rad - *last_candidate_heading_rad_)) < (15.0 * DEG_TO_RAD)) {
                reacquisition_count_++;
            } else {
                reacquisition_count_ = 1;
            }
            last_candidate_heading_rad_ = selected->heading_rad;

            if (reacquisition_count_ >= 2) {
                eskf.update_heading(selected->heading_rad, selected->std_rad, std::nullopt);
                is_updated = true;
                selected->is_accepted = true;
                total_updates_++;
                last_update_time_s_ = time_s;
                outage_duration_s_ = 0.0;
                reacquisition_count_ = 0;
                rejection_reason = "ACCEPTED_REACQUISITION_CONFIRMED";
            } else {
                total_rejections_++;
                rejection_reason = "REJECTED_CHI2_GATE_EXCEEDED_PENDING_CONFIRMATION";
                selected->is_accepted = false;
            }
        }
    }

    if (!is_updated) {
        if (last_update_time_s_ > 0.0) {
            outage_duration_s_ = time_s - last_update_time_s_;
        } else {
            outage_duration_s_ = time_s;
        }
    }

    double gyro_bias_z_deg_s = eskf.get_gyro_bias().z * RAD_TO_DEG;
    double heading_unc_deg = std::sqrt(std::max(0.0, eskf.get_covariance()(8, 8))) * RAD_TO_DEG;

    HeadingSupervisorTelemetry telem;
    telem.time_s = time_s;
    telem.filter_heading_deg = std::fmod(eskf.get_yaw() * RAD_TO_DEG + 360.0, 360.0);
    telem.selected_source = selected_source;
    telem.selected_quality = selected_quality;
    telem.measurement_heading_deg = meas_hdg_deg;
    telem.innovation_deg = innov_deg;
    telem.chi2_normalized = gamma_val;
    telem.is_updated = is_updated;
    telem.rejection_reason = rejection_reason;
    telem.outage_duration_s = outage_duration_s_;
    telem.total_updates_count = total_updates_;
    telem.total_rejections_count = total_rejections_;
    telem.gnss_course_available = (!candidates_scratch_.empty() && candidates_scratch_[0].source == HeadingSourceType::GNSS_COURSE);
    telem.mag_available = mag_raw_p.has_value();
    telem.mag_field_magnitude_uT = mag_field_norm;
    telem.mag_disturbed = mag_is_disturbed;
    telem.gyro_bias_z_deg_s = gyro_bias_z_deg_s;
    telem.heading_uncertainty_deg = heading_unc_deg;

    return telem;
}

} // namespace idr
