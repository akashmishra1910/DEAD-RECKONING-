#pragma once

#include "idr/types.hpp"
#include "idr/math_utils.hpp"
#include <vector>
#include <cmath>
#include <algorithm>


namespace idr {

enum class AlignmentMode {
    INITIALIZING,
    STATIONARY_LEVELED,
    QUASI_STATIC,
    DYNAMIC_ALIGNED,
    DEGRADED,
    RECALIBRATING
};

inline const char* alignment_mode_to_string(AlignmentMode mode) {
    switch (mode) {
        case AlignmentMode::INITIALIZING: return "INITIALIZING";
        case AlignmentMode::STATIONARY_LEVELED: return "STATIONARY_LEVELED";
        case AlignmentMode::QUASI_STATIC: return "QUASI_STATIC";
        case AlignmentMode::DYNAMIC_ALIGNED: return "DYNAMIC_ALIGNED";
        case AlignmentMode::DEGRADED: return "DEGRADED";
        case AlignmentMode::RECALIBRATING: return "RECALIBRATING";
        default: return "UNKNOWN";
    }
}

struct AlignmentQuality {
    double gravity_confidence = 0.0;
    double forward_axis_confidence = 0.0;
    double heading_confidence = 0.0;
    const char* motion_state = "UNKNOWN";
    bool alignment_valid = false;
    double det_residual = 0.0;
    double orthogonality_residual = 0.0;
    Vector3 initial_gyro_bias = Vector3::zero();
    double gyro_bias_variance = 0.0;
    double stationary_duration_s = 0.0;
    double eigenvalue_ratio = 0.0;
    double peak_surge_accel = 0.0;
};

struct AlignmentState {
    bool is_leveled = false;
    bool is_aligned = false;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    Matrix3x3 R_p2b = Matrix3x3::identity();
    Matrix3x3 R_b2p = Matrix3x3::identity();
    double confidence = 0.0;
    AlignmentMode mode = AlignmentMode::INITIALIZING;
    AlignmentQuality quality;
};

Vector3 sanitize_imu_sample(const Vector3& vec, const Vector3& fallback);

class InVehicleAlignmentEngine {
public:
    explicit InVehicleAlignmentEngine(
        double g_nominal = STANDARD_GRAVITY,
        int leveling_samples = 25,
        double min_accel_transient = 0.4,
        double slip_threshold_deg = 5.0,
        double dt = 0.1
    );

    void reset();

    AlignmentState process_sample(
        const Vector3& accel_p,
        const Vector3& gyro_p,
        bool is_stationary,
        std::optional<double> gnss_speed = std::nullopt
    );

    Vector3 transform_phone_to_vehicle(const Vector3& vec_phone) const;

    static double angular_difference(const Vector3& v1, const Vector3& v2);

    static Matrix3x3 estimate_mounting_matrix_batch(
        const std::vector<Vector3>& accel_phone,
        const std::vector<Vector3>& gyro_phone = {},
        const std::vector<double>& speed = {},
        AlignmentQuality* out_quality = nullptr
    );

    const AlignmentState& get_state() const { return state_; }
    const AlignmentQuality& get_quality() const { return quality_; }
    const Matrix3x3& get_R_p2b() const { return R_p2b_; }
    const Matrix3x3& get_R_b2p() const { return R_b2p_; }

private:
    void perform_leveling();
    void perform_forward_alignment();

    double g_nominal_;
    int leveling_samples_;
    double min_accel_transient_;
    double slip_threshold_deg_;
    double dt_;

    std::vector<Vector3> stationary_accels_;
    std::vector<Vector3> stationary_gyros_;
    std::vector<Vector3> transient_accels_;
    Vector3 lp_gravity_;
    Vector3 last_calibrated_gravity_;

    Vector3 z_b_phone_;
    Vector3 x_b_phone_;
    Vector3 y_b_phone_;

    Matrix3x3 R_p2b_;
    Matrix3x3 R_b2p_;

    Vector3 gyro_bias_phone_;
    double gyro_bias_var_;

    AlignmentQuality quality_;
    AlignmentState state_;
};

} // namespace idr
