#pragma once

#include "idr/types.hpp"
#include "idr/math_utils.hpp"
#include <tuple>
#include <cmath>
#include <array>


namespace idr {

class ErrorStateKalmanFilter {
public:
    ErrorStateKalmanFilter(
        const std::optional<Vector3>& init_pos = std::nullopt,
        const std::optional<Vector3>& init_vel = std::nullopt,
        const std::optional<Vector3>& init_euler = std::nullopt, // [roll, pitch, yaw]
        const std::optional<Vector3>& init_ba = std::nullopt,
        const std::optional<Vector3>& init_bg = std::nullopt,
        double acc_noise = 0.15,
        double gyro_noise = 0.008,
        double acc_bias_noise = 1e-4,
        double gyro_bias_noise = 1e-5
    );

    // Predict / Mechanization
    void predict(const Vector3& accel_meas, const Vector3& gyro_meas, double dt);

    // Measurement updates
    void update_gnss(const Vector3& gnss_pos, double r_gnss = 2.5);
    void update_gnss_velocity(const Vector3& gnss_vel, double r_vel = 0.2);
    void update_ai_velocity(double v_ai, double r_ai = 0.35);
    void update_ai_velocity_vector(const Vector3& v_ai_b, const Vector3& sigma_v);
    void update_nhc(double r_lat = 0.05, double r_vert = 0.05);
    void update_zupt(double r_zupt = 0.01);
    void update_zaru(double gyro_z_meas, double r_zaru = 0.005);
    void update_altitude(double alt_meas, double r_alt = 0.8);
    std::pair<bool, double> update_heading(double heading_meas, double r_yaw = 0.05, std::optional<double> chi2_gate = std::nullopt);
    std::pair<bool, double> update_road_heading(double road_azimuth, double r_road_yaw = 0.087, std::optional<double> chi2_gate = 9.0);
    std::pair<bool, double> update_road_position(const Vector3& road_snapped_pos, const Vector3& road_tangent_2d, double r_cross = 1.75, std::optional<double> chi2_gate = 16.0);

    // State getters
    const Vector3& get_position() const { return p_; }
    const Vector3& get_velocity() const { return v_; }
    const Matrix3x3& get_rotation_matrix() const { return R_; }
    double get_roll() const { return roll_; }
    double get_pitch() const { return pitch_; }
    double get_yaw() const { return yaw_; }
    const Vector3& get_accel_bias() const { return ba_; }
    const Vector3& get_gyro_bias() const { return bg_; }
    const Matrix15x15& get_covariance() const { return P_; }

    // State setters (e.g. alignment handoff)
    void set_position(const Vector3& p) { p_ = p; }
    void set_velocity(const Vector3& v) { v_ = v; }
    void set_attitude(const Matrix3x3& R);
    void set_accel_bias(const Vector3& ba) { ba_ = ba; }
    void set_gyro_bias(const Vector3& bg) { bg_ = bg; }
    void set_covariance(const Matrix15x15& P) { P_ = P; }
    void reset() { *this = ErrorStateKalmanFilter(); }

private:

    // Core Kalman update with dynamic dimension M in {1, 2, 3}
    template <std::size_t M>
    std::pair<bool, double> apply_kalman_update(
        const std::array<double, M>& dz,
        const std::array<std::array<double, 15>, M>& H,
        const std::array<std::array<double, M>, M>& R_cov,
        std::optional<double> chi2_gate = std::nullopt
    );

    // Nominal states
    Vector3 p_{0.0, 0.0, 0.0};
    Vector3 v_{0.0, 0.0, 0.0};
    double roll_{0.0};
    double pitch_{0.0};
    double yaw_{0.0};
    Matrix3x3 R_{Matrix3x3::identity()};
    Vector3 ba_{0.0, 0.0, 0.0};
    Vector3 bg_{0.0, 0.0, 0.0};

    Vector3 g_nav_{0.0, 0.0, -STANDARD_GRAVITY};

    // Error state covariance (15x15)
    Matrix15x15 P_{Matrix15x15::identity()};

    // Process noise densities
    double q_acc_;
    double q_gyr_;
    double q_ba_;
    double q_bg_;
};

} // namespace idr
