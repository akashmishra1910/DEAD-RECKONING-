#include "idr/eskf.hpp"
#include <cmath>
#include <algorithm>

namespace idr {

ErrorStateKalmanFilter::ErrorStateKalmanFilter(
    const std::optional<Vector3>& init_pos,
    const std::optional<Vector3>& init_vel,
    const std::optional<Vector3>& init_euler,
    const std::optional<Vector3>& init_ba,
    const std::optional<Vector3>& init_bg,
    double acc_noise,
    double gyro_noise,
    double acc_bias_noise,
    double gyro_bias_noise
) {
    if (init_pos.has_value()) p_ = *init_pos;
    if (init_vel.has_value()) v_ = *init_vel;
    if (init_ba.has_value()) ba_ = *init_ba;
    if (init_bg.has_value()) bg_ = *init_bg;

    if (init_euler.has_value()) {
        roll_ = (*init_euler)[0];
        pitch_ = (*init_euler)[1];
        yaw_ = (*init_euler)[2];
        R_ = euler_to_rotation_matrix(roll_, pitch_, yaw_);
    } else {
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        R_ = euler_to_rotation_matrix(roll_, pitch_, yaw_);
    }

    g_nav_ = Vector3(0.0, 0.0, -STANDARD_GRAVITY);

    // Initial covariance diag([1, 1, 2, 0.2, 0.2, 0.2, 0.005, 0.005, 0.01, 0.01, 0.01, 0.01, 1e-5, 1e-5, 1e-5])
    P_ = Matrix15x15::zero();
    P_(0, 0) = 1.0;
    P_(1, 1) = 1.0;
    P_(2, 2) = 2.0;
    P_(3, 3) = 0.2;
    P_(4, 4) = 0.2;
    P_(5, 5) = 0.2;
    P_(6, 6) = 0.005;
    P_(7, 7) = 0.005;
    P_(8, 8) = 0.01;
    P_(9, 9) = 0.01;
    P_(10, 10) = 0.01;
    P_(11, 11) = 0.01;
    P_(12, 12) = 1e-5;
    P_(13, 13) = 1e-5;
    P_(14, 14) = 1e-5;

    q_acc_ = acc_noise * acc_noise;
    q_gyr_ = gyro_noise * gyro_noise;
    q_ba_ = acc_bias_noise * acc_bias_noise;
    q_bg_ = gyro_bias_noise * gyro_bias_noise;
}

void ErrorStateKalmanFilter::set_attitude(const Matrix3x3& R) {
    R_ = renormalize_so3(R);
    auto [r, p, y] = rotation_matrix_to_euler(R_);
    roll_ = r;
    pitch_ = p;
    yaw_ = y;
}

void ErrorStateKalmanFilter::predict(const Vector3& accel_meas, const Vector3& gyro_meas, double dt) {
    Vector3 f_b = accel_meas - ba_;
    Vector3 w_b = gyro_meas - bg_;

    // Specific force transformed to navigation frame: f^n = R * f^b
    Vector3 f_n = R_ * f_b;
    Vector3 a_n = f_n + g_nav_;

    // Trapezoidal position & velocity
    p_ += v_ * dt + 0.5 * a_n * (dt * dt);
    v_ += a_n * dt;

    // Full 3D attitude propagation: R(t + dt) = R(t) @ Exp([w_b * dt]_x)
    Vector3 dtheta_b = w_b * dt;
    R_ = R_ * rodrigues(dtheta_b);
    R_ = renormalize_so3(R_);

    auto [r, p, y] = rotation_matrix_to_euler(R_);
    roll_ = r;
    pitch_ = p;
    yaw_ = y;

    // State transition matrix Phi = I + F * dt
    // F:
    // F[0:3, 3:6] = I
    // F[3:6, 6:9] = -skew_symmetric(f_n)
    // F[3:6, 9:12] = -R
    // F[6:9, 12:15] = -R
    Matrix15x15 Phi = Matrix15x15::identity();

    // Position error dynamics: dot(delta_p) = delta_v
    for (int i = 0; i < 3; ++i) {
        Phi(i, 3 + i) += dt;
    }

    // Velocity error dynamics: -skew_symmetric(f_n)
    Matrix3x3 skew_fn = skew_symmetric(f_n);
    for (int r_idx = 0; r_idx < 3; ++r_idx) {
        for (int c_idx = 0; c_idx < 3; ++c_idx) {
            Phi(3 + r_idx, 6 + c_idx) += -skew_fn(r_idx, c_idx) * dt;
            Phi(3 + r_idx, 9 + c_idx) += -R_(r_idx, c_idx) * dt;
            Phi(6 + r_idx, 12 + c_idx) += -R_(r_idx, c_idx) * dt;
        }
    }

    // Q_d: process noise covariance
    Matrix15x15 Q_d = Matrix15x15::zero();
    for (int i = 0; i < 3; ++i) {
        Q_d(3 + i, 3 + i) = q_acc_ * dt;
        Q_d(6 + i, 6 + i) = q_gyr_ * dt;
        Q_d(9 + i, 9 + i) = q_ba_ * dt;
        Q_d(12 + i, 12 + i) = q_bg_ * dt;
    }

    // P = Phi * P * Phi.T + Q_d
    Matrix15x15 P_new = Phi * P_ * Phi.transpose() + Q_d;

    // Symmetrize
    for (int r_idx = 0; r_idx < 15; ++r_idx) {
        for (int c_idx = r_idx; c_idx < 15; ++c_idx) {
            double sym = 0.5 * (P_new(r_idx, c_idx) + P_new(c_idx, r_idx));
            P_(r_idx, c_idx) = sym;
            P_(c_idx, r_idx) = sym;
        }
    }
}

// Template implementation of Joseph-form Kalman update
template <std::size_t M>
std::pair<bool, double> ErrorStateKalmanFilter::apply_kalman_update(
    const std::array<double, M>& dz,
    const std::array<std::array<double, 15>, M>& H,
    const std::array<std::array<double, M>, M>& R_cov,
    std::optional<double> chi2_gate
) {
    // 1. S = H * P * H^T + R_cov  (M x M)
    // Compute H * P (M x 15)
    std::array<std::array<double, 15>, M> HP{};
    for (std::size_t m = 0; m < M; ++m) {
        for (std::size_t j = 0; j < 15; ++j) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 15; ++k) {
                sum += H[m][k] * P_(k, j);
            }
            HP[m][j] = sum;
        }
    }

    // Compute S = (HP) * H^T + R_cov
    std::array<std::array<double, M>, M> S{};
    for (std::size_t r = 0; r < M; ++r) {
        for (std::size_t c = 0; c < M; ++c) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 15; ++k) {
                sum += HP[r][k] * H[c][k];
            }
            S[r][c] = sum + R_cov[r][c];
        }
    }

    // Invert S (M x M)
    std::array<std::array<double, M>, M> S_inv{};
    if constexpr (M == 1) {
        if (std::abs(S[0][0]) < 1e-12) return {false, 999.0};
        S_inv[0][0] = 1.0 / S[0][0];
    } else if constexpr (M == 2) {
        double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (std::abs(det) < 1e-12) return {false, 999.0};
        double inv_det = 1.0 / det;
        S_inv[0][0] = S[1][1] * inv_det;
        S_inv[0][1] = -S[0][1] * inv_det;
        S_inv[1][0] = -S[1][0] * inv_det;
        S_inv[1][1] = S[0][0] * inv_det;
    } else if constexpr (M == 3) {
        Matrix3x3 s_mat(S[0][0], S[0][1], S[0][2],
                        S[1][0], S[1][1], S[1][2],
                        S[2][0], S[2][1], S[2][2]);
        Matrix3x3 s_inv_mat;
        if (!invert_matrix3x3(s_mat, s_inv_mat)) return {false, 999.0};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                S_inv[r][c] = s_inv_mat(r, c);
            }
        }
    }

    // gamma = dz^T * S_inv * dz
    double gamma = 0.0;
    for (std::size_t r = 0; r < M; ++r) {
        double tmp = 0.0;
        for (std::size_t c = 0; c < M; ++c) {
            tmp += S_inv[r][c] * dz[c];
        }
        gamma += dz[r] * tmp;
    }

    if (chi2_gate.has_value() && gamma > *chi2_gate) {
        return {false, gamma};
    }

    // Kalman gain K = P * H^T * S_inv  (15 x M)
    // First P * H^T (15 x M)
    std::array<std::array<double, M>, 15> PHT{};
    for (std::size_t i = 0; i < 15; ++i) {
        for (std::size_t m = 0; m < M; ++m) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 15; ++k) {
                sum += P_(i, k) * H[m][k];
            }
            PHT[i][m] = sum;
        }
    }

    std::array<std::array<double, M>, 15> K{};
    for (std::size_t i = 0; i < 15; ++i) {
        for (std::size_t m = 0; m < M; ++m) {
            double sum = 0.0;
            for (std::size_t k = 0; k < M; ++k) {
                sum += PHT[i][k] * S_inv[k][m];
            }
            K[i][m] = sum;
        }
    }

    // dx = K * dz  (15 x 1)
    std::array<double, 15> dx{};
    for (std::size_t i = 0; i < 15; ++i) {
        double sum = 0.0;
        for (std::size_t m = 0; m < M; ++m) {
            sum += K[i][m] * dz[m];
        }
        dx[i] = sum;
    }

    // 1. Inject into nominal translational states
    p_.x += dx[0];
    p_.y += dx[1];
    p_.z += dx[2];

    v_.x += dx[3];
    v_.y += dx[4];
    v_.z += dx[5];

    // 2. Inject attitude error: R_corrected = rodrigues(dtheta_n) * R
    Vector3 dtheta_n(dx[6], dx[7], dx[8]);
    R_ = rodrigues(dtheta_n) * R_;
    R_ = renormalize_so3(R_);
    auto [r_ang, p_ang, y_ang] = rotation_matrix_to_euler(R_);
    roll_ = r_ang;
    pitch_ = p_ang;
    yaw_ = y_ang;

    // 3. Inject bias corrections
    ba_.x += dx[9];
    ba_.y += dx[10];
    ba_.z += dx[11];

    bg_.x += dx[12];
    bg_.y += dx[13];
    bg_.z += dx[14];

    // 4. Joseph-form covariance update:
    // I_KH = I - K * H (15 x 15)
    Matrix15x15 I_KH = Matrix15x15::identity();
    for (std::size_t i = 0; i < 15; ++i) {
        for (std::size_t j = 0; j < 15; ++j) {
            double sum = 0.0;
            for (std::size_t m = 0; m < M; ++m) {
                sum += K[i][m] * H[m][j];
            }
            I_KH(i, j) -= sum;
        }
    }

    // P_new = I_KH * P * I_KH^T + K * R_cov * K^T
    Matrix15x15 term1 = I_KH * P_ * I_KH.transpose();

    // Compute K * R_cov * K^T
    std::array<std::array<double, M>, 15> KR{};
    for (std::size_t i = 0; i < 15; ++i) {
        for (std::size_t m = 0; m < M; ++m) {
            double sum = 0.0;
            for (std::size_t k = 0; k < M; ++k) {
                sum += K[i][k] * R_cov[k][m];
            }
            KR[i][m] = sum;
        }
    }
    Matrix15x15 term2 = Matrix15x15::zero();
    for (std::size_t i = 0; i < 15; ++i) {
        for (std::size_t j = 0; j < 15; ++j) {
            double sum = 0.0;
            for (std::size_t m = 0; m < M; ++m) {
                sum += KR[i][m] * K[j][m];
            }
            term2(i, j) = sum;
        }
    }

    Matrix15x15 P_updated = term1 + term2;

    // 5. Error state reset Jacobian: G_reset = I - 0.5 * skew(dtheta_n) on attitude error block
    Matrix15x15 G_reset = Matrix15x15::identity();
    Matrix3x3 skew_dtheta = skew_symmetric(dtheta_n);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            G_reset(6 + r, 6 + c) -= 0.5 * skew_dtheta(r, c);
        }
    }

    Matrix15x15 P_final = G_reset * P_updated * G_reset.transpose();

    // Symmetrize
    for (int r = 0; r < 15; ++r) {
        for (int c = r; c < 15; ++c) {
            double sym = 0.5 * (P_final(r, c) + P_final(c, r));
            P_(r, c) = sym;
            P_(c, r) = sym;
        }
    }

    return {true, gamma};
}

void ErrorStateKalmanFilter::update_gnss(const Vector3& gnss_pos, double r_gnss) {
    std::array<double, 3> dz = {gnss_pos.x - p_.x, gnss_pos.y - p_.y, gnss_pos.z - p_.z};
    std::array<std::array<double, 15>, 3> H{};
    for (int i = 0; i < 3; ++i) H[i][i] = 1.0;

    double r2 = r_gnss * r_gnss;
    std::array<std::array<double, 3>, 3> R_cov{};
    R_cov[0][0] = r2;
    R_cov[1][1] = r2;
    R_cov[2][2] = r2;

    apply_kalman_update<3>(dz, H, R_cov);
}

void ErrorStateKalmanFilter::update_gnss_velocity(const Vector3& gnss_vel, double r_vel) {
    std::array<double, 3> dz = {gnss_vel.x - v_.x, gnss_vel.y - v_.y, gnss_vel.z - v_.z};
    std::array<std::array<double, 15>, 3> H{};
    for (int i = 0; i < 3; ++i) H[i][3 + i] = 1.0;

    double r2 = r_vel * r_vel;
    std::array<std::array<double, 3>, 3> R_cov{};
    R_cov[0][0] = r2;
    R_cov[1][1] = r2;
    R_cov[2][2] = r2;

    apply_kalman_update<3>(dz, H, R_cov);
}

void ErrorStateKalmanFilter::update_ai_velocity(double v_ai, double r_ai) {
    Vector3 v_b = R_.transpose() * v_;
    std::array<double, 1> dz = {v_ai - v_b.x};

    // Forward body axis in nav frame is column 0 of R: R[:, 0]
    Vector3 r0 = R_.col(0);
    Vector3 r0_skew_v = r0.cross(v_); // [1, 0, 0] @ R.T @ [v]_x = (R[:, 0])^T @ [v]_x -> r0 @ skew(v)

    // Note: in Python: H[0, 6:9] = self.R[:, 0] @ skew_symmetric(self.v)
    Matrix3x3 skew_v = skew_symmetric(v_);
    Vector3 h_att = r0 * skew_v; // row vector times matrix

    std::array<std::array<double, 15>, 1> H{};
    H[0][3] = r0.x;
    H[0][4] = r0.y;
    H[0][5] = r0.z;

    H[0][6] = h_att.x;
    H[0][7] = h_att.y;
    H[0][8] = h_att.z;

    std::array<std::array<double, 1>, 1> R_cov{};
    R_cov[0][0] = r_ai * r_ai;

    apply_kalman_update<1>(dz, H, R_cov);
}

void ErrorStateKalmanFilter::update_nhc(double r_lat, double r_vert) {
    Vector3 v_b = R_.transpose() * v_;
    std::array<double, 2> dz = {-v_b.y, -v_b.z};

    Vector3 r1 = R_.col(1);
    Vector3 r2 = R_.col(2);
    Matrix3x3 skew_v = skew_symmetric(v_);
    Vector3 h_att1 = r1 * skew_v;
    Vector3 h_att2 = r2 * skew_v;

    std::array<std::array<double, 15>, 2> H{};
    H[0][3] = r1.x; H[0][4] = r1.y; H[0][5] = r1.z;
    H[0][6] = h_att1.x; H[0][7] = h_att1.y; H[0][8] = h_att1.z;

    H[1][3] = r2.x; H[1][4] = r2.y; H[1][5] = r2.z;
    H[1][6] = h_att2.x; H[1][7] = h_att2.y; H[1][8] = h_att2.z;

    std::array<std::array<double, 2>, 2> R_cov{};
    R_cov[0][0] = r_lat * r_lat;
    R_cov[1][1] = r_vert * r_vert;

    apply_kalman_update<2>(dz, H, R_cov);
}

void ErrorStateKalmanFilter::update_zupt(double r_zupt) {
    std::array<double, 3> dz = {-v_.x, -v_.y, -v_.z};
    std::array<std::array<double, 15>, 3> H{};
    for (int i = 0; i < 3; ++i) H[i][3 + i] = 1.0;

    double r2 = r_zupt * r_zupt;
    std::array<std::array<double, 3>, 3> R_cov{};
    R_cov[0][0] = r2;
    R_cov[1][1] = r2;
    R_cov[2][2] = r2;

    apply_kalman_update<3>(dz, H, R_cov);
}

void ErrorStateKalmanFilter::update_altitude(double alt_meas, double r_alt) {
    std::array<double, 1> dz = {alt_meas - p_.z};
    std::array<std::array<double, 15>, 1> H{};
    H[0][2] = 1.0;

    std::array<std::array<double, 1>, 1> R_cov{};
    R_cov[0][0] = r_alt * r_alt;

    apply_kalman_update<1>(dz, H, R_cov);
}

std::pair<bool, double> ErrorStateKalmanFilter::update_heading(double heading_meas, double r_yaw, std::optional<double> chi2_gate) {
    double dyaw = wrap_pi(heading_meas - yaw_);
    std::array<double, 1> dz = {dyaw};
    std::array<std::array<double, 15>, 1> H{};
    H[0][8] = -1.0;

    std::array<std::array<double, 1>, 1> R_cov{};
    R_cov[0][0] = r_yaw * r_yaw;

    return apply_kalman_update<1>(dz, H, R_cov, chi2_gate);
}

std::pair<bool, double> ErrorStateKalmanFilter::update_road_heading(double road_azimuth, double r_road_yaw, std::optional<double> chi2_gate) {
    double dyaw = wrap_pi(road_azimuth - yaw_);
    if (std::abs(dyaw) > (60.0 * DEG_TO_RAD)) {
        return {false, 999.0};
    }

    std::array<double, 1> dz = {dyaw};
    std::array<std::array<double, 15>, 1> H{};
    H[0][8] = -1.0;

    std::array<std::array<double, 1>, 1> R_cov{};
    R_cov[0][0] = r_road_yaw * r_road_yaw;

    return apply_kalman_update<1>(dz, H, R_cov, chi2_gate);
}

std::pair<bool, double> ErrorStateKalmanFilter::update_road_position(
    const Vector3& road_snapped_pos,
    const Vector3& road_tangent_2d,
    double r_cross,
    std::optional<double> chi2_gate
) {
    double t_E = road_tangent_2d.x;
    double t_N = road_tangent_2d.y;
    double n_E = -t_N;
    double n_N = t_E;

    double err_E = road_snapped_pos.x - p_.x;
    double err_N = road_snapped_pos.y - p_.y;
    double dz_cross = n_E * err_E + n_N * err_N;

    std::array<double, 1> dz = {dz_cross};
    std::array<std::array<double, 15>, 1> H{};
    H[0][0] = n_E;
    H[0][1] = n_N;

    std::array<std::array<double, 1>, 1> R_cov{};
    R_cov[0][0] = r_cross * r_cross;

    return apply_kalman_update<1>(dz, H, R_cov, chi2_gate);
}

} // namespace idr
