#pragma once

#include "idr/types.hpp"
#include <tuple>

namespace idr {

// Mathematical constants
inline constexpr double PI = 3.14159265358979323846;
inline constexpr double TWO_PI = 6.28318530717958647692;
inline constexpr double HALF_PI = 1.57079632679489661923;
inline constexpr double DEG_TO_RAD = PI / 180.0;
inline constexpr double RAD_TO_DEG = 180.0 / PI;
inline constexpr double STANDARD_GRAVITY = 9.80665;

// Angle wrapping to [-pi, +pi]
double wrap_pi(double angle);

// Angle wrapping to [0, 2*pi)
double wrap_2pi(double angle);

// Skew-symmetric cross-product matrix [v]_x
Matrix3x3 skew_symmetric(const Vector3& v);

// 3D Rodrigues rotation formula from rotation vector (rot_vec) to SO(3) DCM
Matrix3x3 rodrigues(const Vector3& rot_vec);

// Direction Cosine Matrix R_b^n from vehicle body frame (b: FLU) to navigation frame (n: ENU)
// Yaw: azimuth clockwise from North [0, 2*pi)
// Pitch: elevation angle (positive = nose up)
// Roll: bank angle (positive = right side down / left side up)
Matrix3x3 euler_to_rotation_matrix(double roll, double pitch, double yaw);

// Extracts (roll, pitch, yaw) from R_b^n (FLU -> ENU)
// Yaw in [-pi, +pi], pitch in [-pi/2, +pi/2], roll in [-pi, +pi]
std::tuple<double, double, double> rotation_matrix_to_euler(const Matrix3x3& R);

// Analytical 3x3 matrix inverse (returns false if singular)
bool invert_matrix3x3(const Matrix3x3& in, Matrix3x3& out);

// Re-orthogonalizes 3x3 matrix to closest SO(3) matrix via iterative polar decomposition
// Guarantees R @ R.T = I and det(R) = +1.0 within 1e-13
Matrix3x3 renormalize_so3(const Matrix3x3& R);

// Exact 3x3 symmetric eigendecomposition via Jacobi rotations
// A must be symmetric: A = V * diag(eigenvalues) * V^T
// Eigenvalues and eigenvectors sorted in descending order
void eigen_symmetric3x3(const Matrix3x3& A, Vector3& eigenvalues, Matrix3x3& eigenvectors);

// Inverts 15x15 matrix using Gauss-Jordan elimination with partial pivoting
// Used in ESKF Joseph-form / Kalman gain calculation
bool invert_matrix15x15(const Matrix15x15& in, Matrix15x15& out);

} // namespace idr
