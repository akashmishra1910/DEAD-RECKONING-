#include "idr/math_utils.hpp"
#include <algorithm>
#include <cmath>

namespace idr {

double wrap_pi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double wrap_2pi(double angle) {
    double w = std::fmod(angle, TWO_PI);
    if (w < 0.0) {
        w += TWO_PI;
    }
    return w;
}

Matrix3x3 skew_symmetric(const Vector3& v) {
    return Matrix3x3(
        0.0, -v.z,  v.y,
        v.z,  0.0, -v.x,
       -v.y,  v.x,  0.0
    );
}

Matrix3x3 rodrigues(const Vector3& rot_vec) {
    double theta = rot_vec.norm();
    if (theta < 1e-12) {
        return Matrix3x3::identity() + skew_symmetric(rot_vec);
    }
    Vector3 axis = rot_vec * (1.0 / theta);
    Matrix3x3 K = skew_symmetric(axis);
    Matrix3x3 K2 = K * K;
    return Matrix3x3::identity() + (K * std::sin(theta)) + (K2 * (1.0 - std::cos(theta)));
}

Matrix3x3 euler_to_rotation_matrix(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    // Column 0: Forward x_b in nav frame
    Vector3 r1{sy * cp, cy * cp, sp};

    // Base lateral and vertical columns before roll
    Vector3 r2_base{-cy, sy, 0.0};
    Vector3 r3_base{-sy * sp, -cy * sp, cp};

    // Bank/roll applied around forward axis x_b
    Vector3 r2 = cr * r2_base + sr * r3_base;
    Vector3 r3 = (-sr) * r2_base + cr * r3_base;

    return Matrix3x3::from_columns(r1, r2, r3);
}

std::tuple<double, double, double> rotation_matrix_to_euler(const Matrix3x3& R) {
    double clamped_r20 = std::max(-1.0, std::min(1.0, R(2, 0)));
    double pitch = std::asin(clamped_r20);
    double yaw = std::atan2(R(0, 0), R(1, 0));
    double roll = std::atan2(R(2, 1), R(2, 2));
    return {roll, pitch, yaw};
}

bool invert_matrix3x3(const Matrix3x3& in, Matrix3x3& out) {
    double det = in.determinant();
    if (std::abs(det) < 1e-15) {
        return false;
    }
    double inv_det = 1.0 / det;

    out(0, 0) =  (in(1, 1) * in(2, 2) - in(1, 2) * in(2, 1)) * inv_det;
    out(0, 1) = -(in(0, 1) * in(2, 2) - in(0, 2) * in(2, 1)) * inv_det;
    out(0, 2) =  (in(0, 1) * in(1, 2) - in(0, 2) * in(1, 1)) * inv_det;

    out(1, 0) = -(in(1, 0) * in(2, 2) - in(1, 2) * in(2, 0)) * inv_det;
    out(1, 1) =  (in(0, 0) * in(2, 2) - in(0, 2) * in(2, 0)) * inv_det;
    out(1, 2) = -(in(0, 0) * in(1, 2) - in(0, 2) * in(1, 0)) * inv_det;

    out(2, 0) =  (in(1, 0) * in(2, 1) - in(1, 1) * in(2, 0)) * inv_det;
    out(2, 1) = -(in(0, 0) * in(2, 1) - in(0, 1) * in(2, 0)) * inv_det;
    out(2, 2) =  (in(0, 0) * in(1, 1) - in(0, 1) * in(1, 0)) * inv_det;

    return true;
}

Matrix3x3 renormalize_so3(const Matrix3x3& R) {
    // Polar decomposition via Newton-Schulz / Higham iteration:
    // X_{k+1} = 0.5 * (X_k + X_k^{-T})
    Matrix3x3 X = R;
    for (int iter = 0; iter < 10; ++iter) {
        Matrix3x3 X_inv;
        if (!invert_matrix3x3(X, X_inv)) {
            break;
        }
        Matrix3x3 X_next = (X + X_inv.transpose()) * 0.5;

        // Check convergence
        double max_diff = 0.0;
        for (std::size_t i = 0; i < 9; ++i) {
            max_diff = std::max(max_diff, std::abs(X_next.m[i] - X.m[i]));
        }
        X = X_next;
        if (max_diff < 1e-15) {
            break;
        }
    }

    // Ensure det(X) == +1.0
    if (X.determinant() < 0.0) {
        // If improper rotation, reflect third column
        X(0, 2) = -X(0, 2);
        X(1, 2) = -X(1, 2);
        X(2, 2) = -X(2, 2);
    }
    return X;
}

void eigen_symmetric3x3(const Matrix3x3& A, Vector3& eigenvalues, Matrix3x3& eigenvectors) {
    Matrix3x3 D = A;
    eigenvectors = Matrix3x3::identity();

    // Cyclic Jacobi iterations
    for (int iter = 0; iter < 50; ++iter) {
        // Find largest off-diagonal element
        int p = 0, q = 1;
        double max_val = std::abs(D(0, 1));
        if (std::abs(D(0, 2)) > max_val) {
            max_val = std::abs(D(0, 2));
            p = 0; q = 2;
        }
        if (std::abs(D(1, 2)) > max_val) {
            max_val = std::abs(D(1, 2));
            p = 1; q = 2;
        }

        if (max_val < 1e-15) {
            break;
        }

        double app = D(p, p);
        double aqq = D(q, q);
        double apq = D(p, q);

        double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
        double c = std::cos(phi);
        double s = std::sin(phi);

        // Apply rotation to D: J^T * D * J
        Matrix3x3 D_new = D;
        D_new(p, p) = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        D_new(q, q) = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        D_new(p, q) = 0.0;
        D_new(q, p) = 0.0;

        int r = 3 - p - q; // remaining index (0, 1, or 2)
        D_new(p, r) = c * D(p, r) - s * D(q, r);
        D_new(r, p) = D_new(p, r);
        D_new(q, r) = s * D(p, r) + c * D(q, r);
        D_new(r, q) = D_new(q, r);

        D = D_new;

        // Accumulate eigenvectors: V = V * J
        for (int i = 0; i < 3; ++i) {
            double vip = eigenvectors(i, p);
            double viq = eigenvectors(i, q);
            eigenvectors(i, p) = c * vip - s * viq;
            eigenvectors(i, q) = s * vip + c * viq;
        }
    }

    eigenvalues = {D(0, 0), D(1, 1), D(2, 2)};

    // Sort eigenvalues descending: lambda[0] >= lambda[1] >= lambda[2]
    int idx[3] = {0, 1, 2};
    if (eigenvalues[idx[0]] < eigenvalues[idx[1]]) std::swap(idx[0], idx[1]);
    if (eigenvalues[idx[1]] < eigenvalues[idx[2]]) std::swap(idx[1], idx[2]);
    if (eigenvalues[idx[0]] < eigenvalues[idx[1]]) std::swap(idx[0], idx[1]);

    Vector3 sorted_evals{eigenvalues[idx[0]], eigenvalues[idx[1]], eigenvalues[idx[2]]};
    Matrix3x3 sorted_evecs = Matrix3x3::from_columns(
        eigenvectors.col(idx[0]),
        eigenvectors.col(idx[1]),
        eigenvectors.col(idx[2])
    );

    eigenvalues = sorted_evals;
    eigenvectors = sorted_evecs;
}

bool invert_matrix15x15(const Matrix15x15& in, Matrix15x15& out) {
    // Gauss-Jordan elimination with partial pivoting on augmented matrix [A | I]
    // Stack-allocated 15x30 matrix
    double aug[15][30];
    for (std::size_t r = 0; r < 15; ++r) {
        for (std::size_t c = 0; c < 15; ++c) {
            aug[r][c] = in(r, c);
            aug[r][c + 15] = (r == c) ? 1.0 : 0.0;
        }
    }

    for (std::size_t col = 0; col < 15; ++col) {
        // Find pivot
        std::size_t max_row = col;
        double max_val = std::abs(aug[col][col]);
        for (std::size_t r = col + 1; r < 15; ++r) {
            double v = std::abs(aug[r][col]);
            if (v > max_val) {
                max_val = v;
                max_row = r;
            }
        }

        if (max_val < 1e-15) {
            return false; // Singular matrix
        }

        // Swap rows
        if (max_row != col) {
            for (std::size_t c = 0; c < 30; ++c) {
                std::swap(aug[col][c], aug[max_row][c]);
            }
        }

        // Normalize pivot row
        double pivot = aug[col][col];
        double inv_pivot = 1.0 / pivot;
        for (std::size_t c = 0; c < 30; ++c) {
            aug[col][c] *= inv_pivot;
        }

        // Eliminate column entries in other rows
        for (std::size_t r = 0; r < 15; ++r) {
            if (r != col) {
                double factor = aug[r][col];
                if (std::abs(factor) > 1e-18) {
                    for (std::size_t c = 0; c < 30; ++c) {
                        aug[r][c] -= factor * aug[col][c];
                    }
                }
            }
        }
    }

    // Extract right half into out
    for (std::size_t r = 0; r < 15; ++r) {
        for (std::size_t c = 0; c < 15; ++c) {
            out(r, c) = aug[r][c + 15];
        }
    }
    return true;
}

} // namespace idr
