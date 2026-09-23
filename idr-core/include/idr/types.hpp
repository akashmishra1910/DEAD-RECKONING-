#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

namespace idr {

// ============================================================================
// Fixed-Size Stack Allocation Linear Algebra Primitives (Zero Heap Allocation)
// ============================================================================

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vector3() = default;
    constexpr Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr double& operator[](std::size_t idx) {
        return idx == 0 ? x : (idx == 1 ? y : z);
    }

    constexpr const double& operator[](std::size_t idx) const {
        return idx == 0 ? x : (idx == 1 ? y : z);
    }

    constexpr Vector3 operator+(const Vector3& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr Vector3 operator-(const Vector3& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }

    constexpr Vector3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }

    Vector3& operator+=(const Vector3& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }

    Vector3& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    Vector3& operator/=(double scalar) {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }

    constexpr Vector3 operator-() const {
        return {-x, -y, -z};
    }

    constexpr double dot(const Vector3& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    constexpr Vector3 cross(const Vector3& rhs) const {
        return {
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        };
    }

    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    double squared_norm() const {
        return x * x + y * y + z * z;
    }

    Vector3 normalized() const {
        double n = norm();
        return *this * (1.0 / n);
    }

    static constexpr Vector3 zero() {
        return {0.0, 0.0, 0.0};
    }
};

inline constexpr Vector3 operator*(double scalar, const Vector3& v) {
    return v * scalar;
}

// 3x3 Matrix (Row-Major Storage)
struct Matrix3x3 {
    std::array<double, 9> m{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    constexpr Matrix3x3() = default;

    constexpr Matrix3x3(double m00, double m01, double m02,
                        double m10, double m11, double m12,
                        double m20, double m21, double m22)
        : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {}

    static constexpr Matrix3x3 identity() {
        return {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
    }

    static constexpr Matrix3x3 zero() {
        return Matrix3x3();
    }

    static constexpr Matrix3x3 diag(double d0, double d1, double d2) {
        return {d0, 0.0, 0.0,
                0.0, d1, 0.0,
                0.0, 0.0, d2};
    }

    static constexpr Matrix3x3 from_columns(const Vector3& c0, const Vector3& c1, const Vector3& c2) {
        return {c0.x, c1.x, c2.x,
                c0.y, c1.y, c2.y,
                c0.z, c1.z, c2.z};
    }

    double& operator()(std::size_t r, std::size_t c) {
        return m[r * 3 + c];
    }

    constexpr const double& operator()(std::size_t r, std::size_t c) const {
        return m[r * 3 + c];
    }

    constexpr Vector3 col(std::size_t c) const {
        return {m[0 * 3 + c], m[1 * 3 + c], m[2 * 3 + c]};
    }

    constexpr Vector3 row(std::size_t r) const {
        return {m[r * 3 + 0], m[r * 3 + 1], m[r * 3 + 2]};
    }

    Matrix3x3 transpose() const {
        return {m[0], m[3], m[6],
                m[1], m[4], m[7],
                m[2], m[5], m[8]};
    }

    Matrix3x3 operator+(const Matrix3x3& rhs) const {
        Matrix3x3 res;
        for (std::size_t i = 0; i < 9; ++i) res.m[i] = m[i] + rhs.m[i];
        return res;
    }

    Matrix3x3 operator-(const Matrix3x3& rhs) const {
        Matrix3x3 res;
        for (std::size_t i = 0; i < 9; ++i) res.m[i] = m[i] - rhs.m[i];
        return res;
    }

    Matrix3x3 operator*(double scalar) const {
        Matrix3x3 res;
        for (std::size_t i = 0; i < 9; ++i) res.m[i] = m[i] * scalar;
        return res;
    }

    constexpr Vector3 operator*(const Vector3& v) const {
        return {
            m[0] * v.x + m[1] * v.y + m[2] * v.z,
            m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z
        };
    }

    Matrix3x3 operator*(const Matrix3x3& rhs) const {
        Matrix3x3 res;
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                res(r, c) = (*this)(r, 0) * rhs(0, c) +
                            (*this)(r, 1) * rhs(1, c) +
                            (*this)(r, 2) * rhs(2, c);
            }
        }
        return res;
    }

    constexpr double determinant() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) -
               m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    }

    constexpr double trace() const {
        return m[0] + m[4] + m[8];
    }

    double norm_frobenius() const {
        double sum = 0.0;
        for (std::size_t i = 0; i < 9; ++i) sum += m[i] * m[i];
        return std::sqrt(sum);
    }
};

inline Matrix3x3 operator*(double scalar, const Matrix3x3& mat) {
    return mat * scalar;
}

inline Vector3 operator*(const Vector3& v, const Matrix3x3& mat) {
    return {
        v.x * mat(0, 0) + v.y * mat(1, 0) + v.z * mat(2, 0),
        v.x * mat(0, 1) + v.y * mat(1, 1) + v.z * mat(2, 1),
        v.x * mat(0, 2) + v.y * mat(1, 2) + v.z * mat(2, 2)
    };
}

// Fixed-size 15-Element Vector
struct Vector15 {
    std::array<double, 15> data{};

    constexpr Vector15() = default;

    double& operator[](std::size_t i) { return data[i]; }
    constexpr const double& operator[](std::size_t i) const { return data[i]; }

    Vector15 operator+(const Vector15& rhs) const {
        Vector15 res;
        for (std::size_t i = 0; i < 15; ++i) res.data[i] = data[i] + rhs.data[i];
        return res;
    }

    Vector15 operator-(const Vector15& rhs) const {
        Vector15 res;
        for (std::size_t i = 0; i < 15; ++i) res.data[i] = data[i] - rhs.data[i];
        return res;
    }
};

// Fixed-size 15x15 Matrix
struct Matrix15x15 {
    std::array<double, 225> data{};

    constexpr Matrix15x15() = default;

    static Matrix15x15 identity() {
        Matrix15x15 res;
        for (std::size_t i = 0; i < 15; ++i) res(i, i) = 1.0;
        return res;
    }

    static Matrix15x15 zero() {
        return Matrix15x15();
    }

    double& operator()(std::size_t r, std::size_t c) {
        return data[r * 15 + c];
    }

    constexpr const double& operator()(std::size_t r, std::size_t c) const {
        return data[r * 15 + c];
    }

    Matrix15x15 transpose() const {
        Matrix15x15 res;
        for (std::size_t r = 0; r < 15; ++r) {
            for (std::size_t c = 0; c < 15; ++c) {
                res(c, r) = (*this)(r, c);
            }
        }
        return res;
    }

    Matrix15x15 operator+(const Matrix15x15& rhs) const {
        Matrix15x15 res;
        for (std::size_t i = 0; i < 225; ++i) res.data[i] = data[i] + rhs.data[i];
        return res;
    }

    Matrix15x15 operator-(const Matrix15x15& rhs) const {
        Matrix15x15 res;
        for (std::size_t i = 0; i < 225; ++i) res.data[i] = data[i] - rhs.data[i];
        return res;
    }

    Matrix15x15 operator*(double scalar) const {
        Matrix15x15 res;
        for (std::size_t i = 0; i < 225; ++i) res.data[i] = data[i] * scalar;
        return res;
    }

    Vector15 operator*(const Vector15& v) const {
        Vector15 res;
        for (std::size_t r = 0; r < 15; ++r) {
            double sum = 0.0;
            for (std::size_t c = 0; c < 15; ++c) {
                sum += (*this)(r, c) * v[c];
            }
            res[r] = sum;
        }
        return res;
    }

    Matrix15x15 operator*(const Matrix15x15& rhs) const {
        Matrix15x15 res;
        for (std::size_t r = 0; r < 15; ++r) {
            for (std::size_t c = 0; c < 15; ++c) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 15; ++k) {
                    sum += (*this)(r, k) * rhs(k, c);
                }
                res(r, c) = sum;
            }
        }
        return res;
    }

    double trace() const {
        double sum = 0.0;
        for (std::size_t i = 0; i < 15; ++i) sum += (*this)(i, i);
        return sum;
    }

    void symmetrize() {
        for (std::size_t r = 0; r < 15; ++r) {
            for (std::size_t c = r + 1; c < 15; ++c) {
                double avg = 0.5 * ((*this)(r, c) + (*this)(c, r));
                (*this)(r, c) = avg;
                (*this)(c, r) = avg;
            }
        }
    }
};

// ============================================================================
// Core Navigation Enumerations
// ============================================================================

enum class NavigationMode {
    INITIALIZING = 0,
    GNSS_AIDED = 1,
    GNSS_DEGRADED = 2,
    TRANSITION_TO_DR = 3,
    DR_ONLY = 4,
    TRANSITION_TO_GNSS = 5,
    GNSS_REACQUIRED = 6
};

inline const char* navigation_mode_to_string(NavigationMode mode) {
    switch (mode) {
        case NavigationMode::INITIALIZING: return "INITIALIZING";
        case NavigationMode::GNSS_AIDED: return "GNSS_AIDED";
        case NavigationMode::GNSS_DEGRADED: return "GNSS_DEGRADED";
        case NavigationMode::TRANSITION_TO_DR: return "TRANSITION_TO_DR";
        case NavigationMode::DR_ONLY: return "DR_ONLY";
        case NavigationMode::TRANSITION_TO_GNSS: return "TRANSITION_TO_GNSS";
        case NavigationMode::GNSS_REACQUIRED: return "GNSS_REACQUIRED";
    }
    return "UNKNOWN";
}

} // namespace idr
