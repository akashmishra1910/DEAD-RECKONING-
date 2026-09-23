#pragma once

#include "idr/types.hpp"
#include "idr/math_utils.hpp"
#include "idr/eskf.hpp"
#include <vector>
#include <cmath>


namespace idr {

enum class HeadingQuality {
    INVALID,
    DISTURBED,
    WEAK,
    NOMINAL,
    HIGH_ACCURACY
};

inline const char* heading_quality_to_string(HeadingQuality q) {
    switch (q) {
        case HeadingQuality::INVALID: return "INVALID";
        case HeadingQuality::DISTURBED: return "DISTURBED";
        case HeadingQuality::WEAK: return "WEAK";
        case HeadingQuality::NOMINAL: return "NOMINAL";
        case HeadingQuality::HIGH_ACCURACY: return "HIGH_ACCURACY";
        default: return "UNKNOWN";
    }
}

enum class HeadingSourceType {
    GYRO_INTEGRATION,
    GNSS_COURSE,
    MAGNETOMETER,
    DUAL_ANTENNA,
    MOTION_CONSTRAINTS
};

inline const char* heading_source_to_string(HeadingSourceType s) {
    switch (s) {
        case HeadingSourceType::GYRO_INTEGRATION: return "GYRO_INTEGRATION";
        case HeadingSourceType::GNSS_COURSE: return "GNSS_COURSE";
        case HeadingSourceType::MAGNETOMETER: return "MAGNETOMETER";
        case HeadingSourceType::DUAL_ANTENNA: return "DUAL_ANTENNA";
        case HeadingSourceType::MOTION_CONSTRAINTS: return "MOTION_CONSTRAINTS";
        default: return "UNKNOWN";
    }
}

struct HeadingMeasurement {
    HeadingSourceType source = HeadingSourceType::GYRO_INTEGRATION;
    double heading_rad = 0.0;
    double heading_deg = 0.0;
    double std_rad = 0.0;
    HeadingQuality quality = HeadingQuality::INVALID;
    double raw_magnitude = 0.0;
    std::optional<double> innovation_rad = std::nullopt;
    std::optional<double> innovation_deg = std::nullopt;
    std::optional<double> innovation_variance = std::nullopt;
    std::optional<double> chi2_normalized = std::nullopt;
    bool is_accepted = false;
    const char* rejection_reason = nullptr;
};

struct HeadingSupervisorTelemetry {
    double time_s = 0.0;
    double filter_heading_deg = 0.0;
    HeadingSourceType selected_source = HeadingSourceType::GYRO_INTEGRATION;
    HeadingQuality selected_quality = HeadingQuality::INVALID;
    std::optional<double> measurement_heading_deg = std::nullopt;
    std::optional<double> innovation_deg = std::nullopt;
    std::optional<double> chi2_normalized = std::nullopt;
    bool is_updated = false;
    const char* rejection_reason = "NO_VALID_HEADING_SOURCE";
    double outage_duration_s = 0.0;
    int total_updates_count = 0;
    int total_rejections_count = 0;
    bool gnss_course_available = false;
    bool mag_available = false;
    double mag_field_magnitude_uT = 0.0;
    bool mag_disturbed = false;
    double gyro_bias_z_deg_s = 0.0;
    double heading_uncertainty_deg = 0.0;
};

class MagneticDisturbanceDetector {
public:
    explicit MagneticDisturbanceDetector(
        double reference_field_uT = 48.0,
        double magnitude_tolerance_uT = 12.0,
        double max_gradient_uT_s = 35.0,
        double min_field_uT = 5.0,
        double max_field_uT = 250.0,
        double declination_rad = 0.0,
        const Vector3& hard_iron_bias = Vector3::zero(),
        const Matrix3x3& soft_iron_matrix = Matrix3x3::identity()
    );

    void reset();

    std::tuple<HeadingQuality, double, double, double, const char*> evaluate_measurement(
        const Vector3& mag_raw_p,
        double time_s,
        const Matrix3x3& R_p2b,
        const Matrix3x3& R_b2n
    );

private:
    double ref_field_;
    double mag_tol_;
    double max_grad_;
    double min_field_;
    double max_field_;
    double declination_;
    Vector3 hard_iron_;
    Matrix3x3 soft_iron_;

    std::optional<Vector3> last_mag_vec_;
    std::optional<double> last_time_s_;
};

class GNSSCourseDeriver {
public:
    explicit GNSSCourseDeriver(
        double min_speed_mps = 2.5,
        double max_yaw_rate_rad_s = 0.75,
        double base_velocity_noise_mps = 0.15
    );

    std::tuple<HeadingQuality, double, double, double, const char*> derive_course(
        const Vector3& vel_enu,
        double current_yaw_rate = 0.0,
        bool is_forward = true
    );

private:
    double min_speed_;
    double max_yaw_rate_;
    double base_vel_noise_;
};

class HeadingSupervisor {
public:
    explicit HeadingSupervisor(
        double chi2_gate = 6.635,
        double min_gnss_speed_mps = 2.5,
        double ref_mag_field_uT = 48.0,
        bool enable_magnetometer = true,
        bool enable_gnss_course = true
    );

    void reset();

    HeadingSupervisorTelemetry process_step(
        double time_s,
        ErrorStateKalmanFilter& eskf,
        const Matrix3x3& R_p2b,
        const std::optional<Vector3>& gnss_vel = std::nullopt,
        bool gnss_valid = false,
        const std::optional<Vector3>& mag_raw_p = std::nullopt,
        double current_yaw_rate = 0.0,
        bool is_forward = true
    );

    int get_total_updates() const { return total_updates_; }
    int get_total_rejections() const { return total_rejections_; }
    double get_outage_duration() const { return outage_duration_s_; }

private:
    double chi2_gate_;
    bool enable_mag_;
    bool enable_gnss_;

    MagneticDisturbanceDetector mag_detector_;
    GNSSCourseDeriver gnss_deriver_;

    int total_updates_ = 0;
    int total_rejections_ = 0;
    double last_update_time_s_ = 0.0;
    double outage_duration_s_ = 0.0;
    int reacquisition_count_ = 0;
    std::optional<double> last_candidate_heading_rad_ = std::nullopt;
    std::vector<HeadingMeasurement> candidates_scratch_;
};

} // namespace idr
