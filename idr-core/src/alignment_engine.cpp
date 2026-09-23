#include "idr/alignment_engine.hpp"
#include <numeric>

namespace idr {

Vector3 sanitize_imu_sample(const Vector3& vec, const Vector3& fallback) {
    Vector3 clean;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(vec[i])) {
            clean[i] = fallback[i];
        } else {
            clean[i] = vec[i];
        }
    }
    return clean;
}

InVehicleAlignmentEngine::InVehicleAlignmentEngine(
    double g_nominal,
    int leveling_samples,
    double min_accel_transient,
    double slip_threshold_deg,
    double dt
) : g_nominal_(g_nominal),
    leveling_samples_(leveling_samples),
    min_accel_transient_(min_accel_transient),
    slip_threshold_deg_(slip_threshold_deg),
    dt_(dt)
{
    reset();
}

void InVehicleAlignmentEngine::reset() {
    stationary_accels_.clear();
    stationary_gyros_.clear();
    transient_accels_.clear();
    lp_gravity_ = Vector3(0.0, 0.0, g_nominal_);
    last_calibrated_gravity_ = Vector3(0.0, 0.0, g_nominal_);

    z_b_phone_ = Vector3(0.0, 0.0, 1.0);
    x_b_phone_ = Vector3(1.0, 0.0, 0.0);
    y_b_phone_ = Vector3(0.0, 1.0, 0.0);

    R_p2b_ = Matrix3x3::identity();
    R_b2p_ = Matrix3x3::identity();

    gyro_bias_phone_ = Vector3::zero();
    gyro_bias_var_ = 0.0;

    quality_ = AlignmentQuality();
    state_ = AlignmentState();
    state_.is_leveled = false;
    state_.is_aligned = false;
    state_.roll_deg = 0.0;
    state_.pitch_deg = 0.0;
    state_.yaw_deg = 0.0;
    state_.R_p2b = Matrix3x3::identity();
    state_.R_b2p = Matrix3x3::identity();
    state_.confidence = 0.0;
    state_.mode = AlignmentMode::INITIALIZING;
    state_.quality = quality_;
}

double InVehicleAlignmentEngine::angular_difference(const Vector3& v1, const Vector3& v2) {
    double n1 = v1.norm();
    double n2 = v2.norm();
    if (n1 < 1e-6 || n2 < 1e-6) {
        return 0.0;
    }
    double cos_theta = std::clamp(v1.dot(v2) / (n1 * n2), -1.0, 1.0);
    return std::acos(cos_theta) * RAD_TO_DEG;
}

AlignmentState InVehicleAlignmentEngine::process_sample(
    const Vector3& accel_p,
    const Vector3& gyro_p,
    bool is_stationary,
    std::optional<double> gnss_speed
) {
    Vector3 accel_clean = sanitize_imu_sample(accel_p, lp_gravity_);
    Vector3 gyro_clean = sanitize_imu_sample(gyro_p, Vector3::zero());

    // Update low-pass gravity filter: alpha = 0.02
    lp_gravity_ = lp_gravity_ * 0.98 + accel_clean * 0.02;

    // --- Phase 1: Leveling (Gravity Vector Estimation & Gyro Bias) ---
    if (!state_.is_leveled) {
        if (is_stationary) {
            if (gyro_clean.norm() < 0.25) {
                stationary_accels_.push_back(accel_clean);
                stationary_gyros_.push_back(gyro_clean);
                quality_.stationary_duration_s = static_cast<double>(stationary_accels_.size()) * dt_;
            }
            if (static_cast<int>(stationary_accels_.size()) >= leveling_samples_) {
                perform_leveling();
            }
        } else {
            quality_.motion_state = (static_cast<bool>(gnss_speed) && *gnss_speed < 2.0) ? "QUASI_STATIC" : "UNKNOWN";
        }

        return state_;
    }

    // --- Check Watchdog for Phone Mount Slip / Bump ---
    if (state_.is_aligned) {
        Vector3 test_gravity = is_stationary ? accel_clean : lp_gravity_;
        double angle_diff = angular_difference(test_gravity, last_calibrated_gravity_);
        double gyro_rate = gyro_clean.norm();

        if (angle_diff > slip_threshold_deg_ || (is_stationary && gyro_rate > 0.8)) {
            state_.is_aligned = false;
            state_.is_leveled = false;
            state_.mode = AlignmentMode::RECALIBRATING;
            stationary_accels_.clear();
            stationary_accels_.push_back(accel_clean);
            stationary_gyros_.clear();
            stationary_gyros_.push_back(gyro_clean);
            transient_accels_.clear();
            quality_.alignment_valid = false;
            quality_.forward_axis_confidence = 0.0;
            return state_;
        }
    }

    // --- Phase 2: Forward Axis Determination via Acceleration Transients ---
    if (state_.is_leveled && !state_.is_aligned) {
        Vector3 delta_f = accel_clean - lp_gravity_;
        Vector3 horiz_f = delta_f - z_b_phone_ * delta_f.dot(z_b_phone_);
        double horiz_mag = horiz_f.norm();
        quality_.peak_surge_accel = std::max(quality_.peak_surge_accel, horiz_mag);

        if (is_stationary) {
            quality_.motion_state = "STATIONARY";
        } else if (horiz_mag >= min_accel_transient_) {
            quality_.motion_state = "ACCELERATING";
            transient_accels_.push_back(horiz_f);
            if (transient_accels_.size() >= 15) {
                perform_forward_alignment();
            }
        } else {
            quality_.motion_state = "CRUISING";
        }
    }

    return state_;
}

void InVehicleAlignmentEngine::perform_leveling() {
    if (stationary_accels_.empty()) return;

    Vector3 sum_g = Vector3::zero();
    for (const auto& a : stationary_accels_) {
        sum_g = sum_g + a;
    }
    Vector3 mean_g = sum_g * (1.0 / static_cast<double>(stationary_accels_.size()));
    double norm_g = mean_g.norm();
    if (norm_g < 1e-3) return;

    z_b_phone_ = mean_g / norm_g;
    last_calibrated_gravity_ = mean_g;
    lp_gravity_ = mean_g;

    if (!stationary_gyros_.empty()) {
        Vector3 sum_gyro = Vector3::zero();
        for (const auto& w : stationary_gyros_) {
            sum_gyro = sum_gyro + w;
        }
        gyro_bias_phone_ = sum_gyro * (1.0 / static_cast<double>(stationary_gyros_.size()));
        
        // Variance across all components
        double total_sq_diff = 0.0;
        for (const auto& w : stationary_gyros_) {
            Vector3 diff = w - gyro_bias_phone_;
            total_sq_diff += diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        }
        // In python np.var(stationary_gyros) divides by 3*N
        gyro_bias_var_ = total_sq_diff / (3.0 * static_cast<double>(stationary_gyros_.size()));
    } else {
        gyro_bias_phone_ = Vector3::zero();
        gyro_bias_var_ = 1e-6;
    }

    // Mount pitch & roll in phone frame
    // pitch = -asin(clip(z_b_phone[1], -1, 1))
    // roll = atan2(z_b_phone[0], z_b_phone[2])
    double pitch = -std::asin(std::clamp(z_b_phone_.y, -1.0, 1.0));
    double roll = std::atan2(z_b_phone_.x, z_b_phone_.z);

    double g_err = std::abs(norm_g - g_nominal_) / g_nominal_;

    // Sample variance of stationary accels
    double var_g_sum = 0.0;
    for (int col = 0; col < 3; ++col) {
        double mean_c = mean_g[col];
        double var_c = 0.0;
        for (const auto& a : stationary_accels_) {
            double d = a[col] - mean_c;
            var_c += d * d;
        }
        var_g_sum += var_c / static_cast<double>(stationary_accels_.size());
    }
    double var_g = var_g_sum / 3.0;

    double grav_conf = std::clamp(1.0 - (g_err * 5.0) - (var_g * 0.5), 0.1, 1.0);

    // Form intermediate leveled mounting matrix
    Vector3 test_cand(1.0, 0.0, 0.0);
    Vector3 cand = test_cand - z_b_phone_ * test_cand.dot(z_b_phone_);
    if (cand.norm() < 1e-3) {
        test_cand = Vector3(0.0, 1.0, 0.0);
        cand = test_cand - z_b_phone_ * test_cand.dot(z_b_phone_);
    }
    Vector3 x_init = cand / cand.norm();
    Vector3 y_init = z_b_phone_.cross(x_init);
    y_init = y_init / y_init.norm();

    // R_b2p = [x_init, y_init, z_b_phone] (columns)
    for (int i = 0; i < 3; ++i) {
        R_b2p_(i, 0) = x_init[i];
        R_b2p_(i, 1) = y_init[i];
        R_b2p_(i, 2) = z_b_phone_[i];
    }
    R_b2p_ = renormalize_so3(R_b2p_);
    R_p2b_ = R_b2p_.transpose();

    quality_.gravity_confidence = grav_conf;
    quality_.forward_axis_confidence = 0.0;
    quality_.heading_confidence = 0.0;
    quality_.alignment_valid = false;
    quality_.initial_gyro_bias = R_p2b_ * gyro_bias_phone_;
    quality_.gyro_bias_variance = gyro_bias_var_;
    quality_.det_residual = std::abs(R_p2b_.determinant() - 1.0);
    quality_.orthogonality_residual = (R_p2b_ * R_p2b_.transpose() - Matrix3x3::identity()).norm_frobenius();
    quality_.motion_state = "STATIONARY";

    state_.is_leveled = true;
    state_.is_aligned = false;
    state_.roll_deg = roll * RAD_TO_DEG;
    state_.pitch_deg = pitch * RAD_TO_DEG;
    state_.yaw_deg = 0.0;
    state_.R_p2b = R_p2b_;
    state_.R_b2p = R_b2p_;
    state_.confidence = grav_conf * 0.5;
    state_.mode = AlignmentMode::STATIONARY_LEVELED;
    state_.quality = quality_;
}

void InVehicleAlignmentEngine::perform_forward_alignment() {
    size_t N = transient_accels_.size();
    if (N < 5) return;

    // Compute covariance of horizontal transients
    Vector3 mean_X = Vector3::zero();
    for (const auto& v : transient_accels_) {
        mean_X = mean_X + v;
    }
    mean_X = mean_X * (1.0 / static_cast<double>(N));

    Matrix3x3 cov_h = Matrix3x3::zero();
    for (const auto& v : transient_accels_) {
        Vector3 d = v - mean_X;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                cov_h(r, c) += d[r] * d[c];
            }
        }
    }
    cov_h = cov_h * (1.0 / static_cast<double>(N - 1));

    Vector3 eigvals;
    Matrix3x3 eigvecs;
    eigen_symmetric3x3(cov_h, eigvals, eigvecs);

    // Note: eigen_symmetric3x3 sorts descending: eigvals[0] >= eigvals[1] >= eigvals[2]
    // Python np.linalg.eigh sorts ascending: eigvals[-1] is max, eigvals[-2] is second
    double lambda_principal = std::max(1e-9, eigvals[0]);
    double lambda_transverse = std::max(1e-9, eigvals[1]);
    double kappa = lambda_principal / lambda_transverse;
    quality_.eigenvalue_ratio = kappa;

    // Primary eigenvector column 0 in our descending eigvecs
    Vector3 x_cand(eigvecs(0, 0), eigvecs(1, 0), eigvecs(2, 0));
    x_cand = x_cand - z_b_phone_ * x_cand.dot(z_b_phone_);
    double norm_cand = x_cand.norm();
    if (norm_cand < 1e-3) return;
    x_cand = x_cand / norm_cand;

    // Resolve forward sign via mean transient surge vector
    if (mean_X.dot(x_cand) < 0.0) {
        x_cand = -x_cand;
    }

    x_b_phone_ = x_cand;
    y_b_phone_ = z_b_phone_.cross(x_b_phone_);
    y_b_phone_ = y_b_phone_ / y_b_phone_.norm();

    for (int i = 0; i < 3; ++i) {
        R_b2p_(i, 0) = x_b_phone_[i];
        R_b2p_(i, 1) = y_b_phone_[i];
        R_b2p_(i, 2) = z_b_phone_[i];
    }
    R_b2p_ = renormalize_so3(R_b2p_);
    R_p2b_ = R_b2p_.transpose();

    double yaw = std::atan2(R_p2b_(0, 1), R_p2b_(0, 0));
    double fwd_conf = std::clamp((kappa - 1.5) / 5.0, 0.2, 1.0);

    quality_.forward_axis_confidence = fwd_conf;
    quality_.heading_confidence = fwd_conf;
    quality_.alignment_valid = true;
    quality_.det_residual = std::abs(R_p2b_.determinant() - 1.0);
    quality_.orthogonality_residual = (R_p2b_ * R_p2b_.transpose() - Matrix3x3::identity()).norm_frobenius();
    quality_.initial_gyro_bias = R_p2b_ * gyro_bias_phone_;

    state_.yaw_deg = yaw * RAD_TO_DEG;
    state_.R_p2b = R_p2b_;
    state_.R_b2p = R_b2p_;
    state_.is_aligned = true;
    state_.confidence = quality_.gravity_confidence * 0.4 + fwd_conf * 0.6;
    state_.mode = AlignmentMode::DYNAMIC_ALIGNED;
    state_.quality = quality_;
}

Vector3 InVehicleAlignmentEngine::transform_phone_to_vehicle(const Vector3& vec_phone) const {
    Vector3 clean = sanitize_imu_sample(vec_phone, Vector3::zero());
    return R_p2b_ * clean;
}

Matrix3x3 InVehicleAlignmentEngine::estimate_mounting_matrix_batch(
    const std::vector<Vector3>& accel_phone,
    const std::vector<Vector3>& gyro_phone,
    const std::vector<double>& speed,
    AlignmentQuality* out_quality
) {
    AlignmentQuality quality;

    if (accel_phone.size() < 5) {
        if (out_quality) *out_quality = quality;
        return Matrix3x3::identity();
    }

    size_t N = accel_phone.size();
    Vector3 sum_a = Vector3::zero();
    for (const auto& a : accel_phone) {
        sum_a = sum_a + sanitize_imu_sample(a, Vector3(0.0, 0.0, STANDARD_GRAVITY));
    }
    Vector3 mean_a = sum_a * (1.0 / static_cast<double>(N));
    double norm_a = mean_a.norm();

    Vector3 z_b;
    if (norm_a < 1e-3 || !std::isfinite(norm_a)) {
        z_b = Vector3(0.0, 0.0, 1.0);
        quality.gravity_confidence = 0.0;
    } else {
        z_b = mean_a / norm_a;
        double g_err = std::abs(norm_a - STANDARD_GRAVITY) / STANDARD_GRAVITY;
        quality.gravity_confidence = std::clamp(1.0 - g_err * 5.0, 0.1, 1.0);
    }

    // 2. Horizontal Acceleration Extraction
    std::vector<Vector3> a_horiz(N);
    double peak_surge = 0.0;
    for (size_t i = 0; i < N; ++i) {
        Vector3 diff_a = accel_phone[i] - mean_a;
        double a_vert = diff_a.dot(z_b);
        a_horiz[i] = diff_a - z_b * a_vert;
        double m = a_horiz[i].norm();
        if (m > peak_surge) peak_surge = m;
    }
    quality.peak_surge_accel = peak_surge;

    // 3. Covariance & Horizontal Eigenvalue Separation Analysis
    Vector3 mean_horiz = Vector3::zero();
    for (const auto& h : a_horiz) mean_horiz = mean_horiz + h;
    mean_horiz = mean_horiz * (1.0 / static_cast<double>(N));

    Matrix3x3 cov_h = Matrix3x3::zero();
    for (const auto& h : a_horiz) {
        Vector3 d = h - mean_horiz;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                cov_h(r, c) += d[r] * d[c];
            }
        }
    }
    cov_h = cov_h * (1.0 / static_cast<double>(N - 1));

    Vector3 eigvals;
    Matrix3x3 eigvecs;
    eigen_symmetric3x3(cov_h, eigvals, eigvecs);

    double lambda_principal = std::max(1e-9, eigvals[0]);
    double lambda_transverse = std::max(1e-9, eigvals[1]);
    double kappa = lambda_principal / lambda_transverse;
    quality.eigenvalue_ratio = kappa;

    // Check observable
    if (peak_surge < 0.35 || kappa < 2.0) {
        quality.forward_axis_confidence = 0.0;
        quality.heading_confidence = 0.0;
        quality.alignment_valid = false;
        quality.motion_state = "STATIONARY";

        Vector3 test_cand(1.0, 0.0, 0.0);
        Vector3 cand = test_cand - z_b * test_cand.dot(z_b);
        if (cand.norm() < 1e-3) {
            test_cand = Vector3(0.0, 1.0, 0.0);
            cand = test_cand - z_b * test_cand.dot(z_b);
        }
        Vector3 x_b = cand / cand.norm();
        Vector3 y_b = z_b.cross(x_b);
        y_b = y_b / y_b.norm();

        Matrix3x3 R_p2b;
        for (int i = 0; i < 3; ++i) {
            R_p2b(0, i) = x_b[i];
            R_p2b(1, i) = y_b[i];
            R_p2b(2, i) = z_b[i];
        }
        R_p2b = renormalize_so3(R_p2b);
        quality.det_residual = std::abs(R_p2b.determinant() - 1.0);
        quality.orthogonality_residual = (R_p2b * R_p2b.transpose() - Matrix3x3::identity()).norm_frobenius();

        if (out_quality) *out_quality = quality;
        return R_p2b;
    }

    // Dynamic excitation is valid
    Vector3 x_b(eigvecs(0, 0), eigvecs(1, 0), eigvecs(2, 0));
    x_b = x_b - z_b * x_b.dot(z_b);
    double norm_x = x_b.norm();
    if (norm_x > 1e-3) {
        x_b = x_b / norm_x;
    } else {
        Vector3 test_cand(1.0, 0.0, 0.0);
        Vector3 cand = test_cand - z_b * test_cand.dot(z_b);
        x_b = cand / (cand.norm() + 1e-6);
    }

    // 5. Resolve Forward Direction Sign
    if (speed.size() > 10 && speed.size() == N) {
        // dv = gradient(speed)
        std::vector<double> dv(N);
        dv[0] = speed[1] - speed[0];
        dv[N - 1] = speed[N - 1] - speed[N - 2];
        for (size_t i = 1; i < N - 1; ++i) {
            dv[i] = (speed[i + 1] - speed[i - 1]) * 0.5;
        }

        std::vector<double> a_x(N);
        for (size_t i = 0; i < N; ++i) {
            a_x[i] = a_horiz[i].dot(x_b);
        }

        // Check correlation on valid
        std::vector<double> valid_ax, valid_dv;
        for (size_t i = 0; i < N; ++i) {
            if (std::abs(dv[i]) > 0.05 && std::abs(a_x[i]) > 0.1) {
                valid_ax.push_back(a_x[i]);
                valid_dv.push_back(dv[i]);
            }
        }
        if (valid_ax.size() > 10) {
            double mean_ax = std::accumulate(valid_ax.begin(), valid_ax.end(), 0.0) / valid_ax.size();
            double mean_dv = std::accumulate(valid_dv.begin(), valid_dv.end(), 0.0) / valid_dv.size();
            double cov = 0.0, var_ax = 0.0, var_dv = 0.0;
            for (size_t i = 0; i < valid_ax.size(); ++i) {
                double d1 = valid_ax[i] - mean_ax;
                double d2 = valid_dv[i] - mean_dv;
                cov += d1 * d2;
                var_ax += d1 * d1;
                var_dv += d2 * d2;
            }
            if (var_ax > 1e-12 && var_dv > 1e-12) {
                double corr = cov / std::sqrt(var_ax * var_dv);
                if (std::isfinite(corr) && corr < 0.0) {
                    x_b = -x_b;
                }
            }
        }
    } else {
        // Vehicle braking decelerations are sharper than acceleration (negative skewness)
        std::vector<double> a_x(N);
        double mean_ax = 0.0;
        for (size_t i = 0; i < N; ++i) {
            a_x[i] = a_horiz[i].dot(x_b);
            mean_ax += a_x[i];
        }
        mean_ax /= static_cast<double>(N);

        double var_ax = 0.0;
        for (double v : a_x) {
            double d = v - mean_ax;
            var_ax += d * d;
        }
        var_ax /= static_cast<double>(N);
        double std_ax = std::sqrt(var_ax);

        if (std_ax > 0.05) {
            double m3 = 0.0;
            for (double v : a_x) {
                double d = v - mean_ax;
                m3 += d * d * d;
            }
            m3 /= static_cast<double>(N);
            double skew_x = m3 / (std_ax * std_ax * std_ax + 1e-6);
            if (skew_x > 0.1) {
                x_b = -x_b;
            }
        }
    }

    // 6. Build Orthonormal Right-Handed Basis (FLU Body Frame)
    Vector3 y_b = z_b.cross(x_b);
    double norm_y = y_b.norm();
    if (norm_y > 1e-3) {
        y_b = y_b / norm_y;
    } else {
        y_b = Vector3(0.0, 1.0, 0.0);
    }

    x_b = y_b.cross(z_b);
    x_b = x_b / x_b.norm();

    Matrix3x3 R_p2b;
    for (int i = 0; i < 3; ++i) {
        R_p2b(0, i) = x_b[i];
        R_p2b(1, i) = y_b[i];
        R_p2b(2, i) = z_b[i];
    }
    R_p2b = renormalize_so3(R_p2b);

    double fwd_conf = std::clamp((kappa - 1.8) / 4.0, 0.3, 1.0);
    quality.forward_axis_confidence = fwd_conf;
    quality.heading_confidence = fwd_conf;
    quality.alignment_valid = true;
    quality.motion_state = "ACCELERATING";
    quality.det_residual = std::abs(R_p2b.determinant() - 1.0);
    quality.orthogonality_residual = (R_p2b * R_p2b.transpose() - Matrix3x3::identity()).norm_frobenius();

    if (!gyro_phone.empty()) {
        Vector3 sum_gyro = Vector3::zero();
        for (const auto& w : gyro_phone) sum_gyro = sum_gyro + w;
        Vector3 mean_gyro = sum_gyro * (1.0 / static_cast<double>(gyro_phone.size()));
        quality.initial_gyro_bias = R_p2b * mean_gyro;

        double total_sq_diff = 0.0;
        for (const auto& w : gyro_phone) {
            Vector3 diff = w - mean_gyro;
            total_sq_diff += diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        }
        quality.gyro_bias_variance = total_sq_diff / (3.0 * static_cast<double>(gyro_phone.size()));
    }

    if (out_quality) *out_quality = quality;
    return R_p2b;
}

} // namespace idr
