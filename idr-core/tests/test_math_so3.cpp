#include "idr/math_utils.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

namespace {

void assert_close(double a, double b, double tol, const char* msg) {
    if (std::abs(a - b) > tol) {
        std::cerr << "Assertion failed: " << msg << " (" << a << " vs " << b << ", tol=" << tol << ")\n";
        std::exit(1);
    }
}

void test_so3_orthonormality_and_det() {
    std::mt19937_64 rng(42);
    std::normal_distribution<double> dist(0.0, 2.0 * idr::PI);

    for (int i = 0; i < 1000; ++i) {
        idr::Vector3 rot_vec{dist(rng), dist(rng), dist(rng)};
        idr::Matrix3x3 R = idr::rodrigues(rot_vec);

        // Orthonormality error ||R @ R.T - I||
        idr::Matrix3x3 I_test = R * R.transpose();
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                double expected = (r == c) ? 1.0 : 0.0;
                assert_close(I_test(r, c), expected, 1e-13, "SO(3) orthonormality violated");
            }
        }

        double det = R.determinant();
        assert_close(det, 1.0, 1e-12, "SO(3) determinant not +1.0");
    }
    std::cout << "  [PASS] Test 1: SO(3) orthonormality and det=+1.0 across 1000 arbitrary rotations\n";
}

void test_frame_conventions_enu_flu() {
    // 1. Facing North (yaw = 0): Forward = North, Left = West (-East), Up = Up
    idr::Matrix3x3 R_north = idr::euler_to_rotation_matrix(0.0, 0.0, 0.0);
    assert_close(R_north(0, 0), 0.0, 1e-12, "North: Fwd.x != 0");
    assert_close(R_north(1, 0), 1.0, 1e-12, "North: Fwd.y != 1 (North)");
    assert_close(R_north(2, 0), 0.0, 1e-12, "North: Fwd.z != 0");
    assert_close(R_north(0, 1), -1.0, 1e-12, "North: Left.x != -1 (West)");
    assert_close(R_north(1, 1), 0.0, 1e-12, "North: Left.y != 0");
    assert_close(R_north(2, 1), 0.0, 1e-12, "North: Left.z != 0");
    assert_close(R_north(0, 2), 0.0, 1e-12, "North: Up.x != 0");
    assert_close(R_north(1, 2), 0.0, 1e-12, "North: Up.y != 0");
    assert_close(R_north(2, 2), 1.0, 1e-12, "North: Up.z != 1 (Up)");
    assert_close(R_north.determinant(), 1.0, 1e-12, "North det");

    // 2. Facing East (yaw = pi/2): Forward = East, Left = North, Up = Up
    idr::Matrix3x3 R_east = idr::euler_to_rotation_matrix(0.0, 0.0, idr::HALF_PI);
    assert_close(R_east(0, 0), 1.0, 1e-12, "East: Fwd.x != 1 (East)");
    assert_close(R_east(1, 0), 0.0, 1e-12, "East: Fwd.y != 0");
    assert_close(R_east(2, 0), 0.0, 1e-12, "East: Fwd.z != 0");
    assert_close(R_east(0, 1), 0.0, 1e-12, "East: Left.x != 0");
    assert_close(R_east(1, 1), 1.0, 1e-12, "East: Left.y != 1 (North)");
    assert_close(R_east(2, 1), 0.0, 1e-12, "East: Left.z != 0");
    assert_close(R_east(0, 2), 0.0, 1e-12, "East: Up.x != 0");
    assert_close(R_east(1, 2), 0.0, 1e-12, "East: Up.y != 0");
    assert_close(R_east(2, 2), 1.0, 1e-12, "East: Up.z != 1");
    assert_close(R_east.determinant(), 1.0, 1e-12, "East det");

    // 3. Facing South (yaw = pi): Forward = South, Left = East, Up = Up
    idr::Matrix3x3 R_south = idr::euler_to_rotation_matrix(0.0, 0.0, idr::PI);
    assert_close(R_south(0, 0), 0.0, 1e-12, "South: Fwd.x != 0");
    assert_close(R_south(1, 0), -1.0, 1e-12, "South: Fwd.y != -1 (South)");
    assert_close(R_south(2, 0), 0.0, 1e-12, "South: Fwd.z != 0");
    assert_close(R_south(0, 1), 1.0, 1e-12, "South: Left.x != 1 (East)");
    assert_close(R_south(1, 1), 0.0, 1e-12, "South: Left.y != 0");
    assert_close(R_south(2, 1), 0.0, 1e-12, "South: Left.z != 0");

    // 4. Facing West (yaw = 3*pi/2): Forward = West, Left = South, Up = Up
    idr::Matrix3x3 R_west = idr::euler_to_rotation_matrix(0.0, 0.0, 1.5 * idr::PI);
    assert_close(R_west(0, 0), -1.0, 1e-12, "West: Fwd.x != -1 (West)");
    assert_close(R_west(1, 0), 0.0, 1e-12, "West: Fwd.y != 0");
    assert_close(R_west(2, 0), 0.0, 1e-12, "West: Fwd.z != 0");
    assert_close(R_west(0, 1), 0.0, 1e-12, "West: Left.x != 0");
    assert_close(R_west(1, 1), -1.0, 1e-12, "West: Left.y != -1 (South)");
    assert_close(R_west(2, 1), 0.0, 1e-12, "West: Left.z != 0");

    std::cout << "  [PASS] Test 2: Frame conventions ENU Navigation + FLU Body across 4 cardinals\n";
}

void test_euler_dcm_roundtrip() {
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> dist_yaw(0.0, idr::TWO_PI);
    std::uniform_real_distribution<double> dist_pitch(-0.45 * idr::PI, 0.45 * idr::PI);
    std::uniform_real_distribution<double> dist_roll(-0.45 * idr::PI, 0.45 * idr::PI);

    for (int i = 0; i < 500; ++i) {
        double roll = dist_roll(rng);
        double pitch = dist_pitch(rng);
        double yaw = dist_yaw(rng);

        idr::Matrix3x3 R = idr::euler_to_rotation_matrix(roll, pitch, yaw);
        auto [r_out, p_out, y_out] = idr::rotation_matrix_to_euler(R);

        assert_close(roll, r_out, 1e-12, "Roll roundtrip mismatch");
        assert_close(pitch, p_out, 1e-12, "Pitch roundtrip mismatch");
        assert_close(idr::wrap_pi(yaw), idr::wrap_pi(y_out), 1e-12, "Yaw roundtrip mismatch");
    }
    std::cout << "  [PASS] Test 3: Euler <-> DCM roundtrip across 500 arbitrary orientations\n";
}

void test_matrix_inversion_and_eigen() {
    // Test 3x3 inversion
    idr::Matrix3x3 A(
        2.0, 1.0, 0.0,
        1.0, 3.0, 1.0,
        0.0, 1.0, 2.0
    );
    idr::Matrix3x3 A_inv;
    bool ok = idr::invert_matrix3x3(A, A_inv);
    assert(ok);
    idr::Matrix3x3 prod = A * A_inv;
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            assert_close(prod(r, c), expected, 1e-12, "A * A_inv != I");
        }
    }

    // Test 3x3 symmetric eigendecomposition
    idr::Vector3 evals;
    idr::Matrix3x3 evecs;
    idr::eigen_symmetric3x3(A, evals, evecs);
    // V * diag(evals) * V^T should reconstruct A
    idr::Matrix3x3 recon = evecs * idr::Matrix3x3::diag(evals.x, evals.y, evals.z) * evecs.transpose();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            assert_close(recon(r, c), A(r, c), 1e-12, "Eigendecomposition reconstruction mismatch");
        }
    }

    // Test 15x15 inversion
    idr::Matrix15x15 P = idr::Matrix15x15::identity() * 2.5;
    P(0, 1) = 0.5; P(1, 0) = 0.5;
    idr::Matrix15x15 P_inv;
    bool ok15 = idr::invert_matrix15x15(P, P_inv);
    assert(ok15);
    idr::Matrix15x15 P_prod = P * P_inv;
    for (std::size_t r = 0; r < 15; ++r) {
        for (std::size_t c = 0; c < 15; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            assert_close(P_prod(r, c), expected, 1e-12, "P * P_inv != I");
        }
    }

    std::cout << "  [PASS] Test 4: Matrix inversion (3x3 and 15x15) and symmetric eigendecomposition\n";
}

void test_so3_renormalization() {
    idr::Matrix3x3 R = idr::euler_to_rotation_matrix(0.1, -0.2, 1.4);
    // Perturb R slightly
    idr::Matrix3x3 R_noisy = R;
    R_noisy(0, 1) += 0.005;
    R_noisy(2, 0) -= 0.003;

    idr::Matrix3x3 R_clean = idr::renormalize_so3(R_noisy);
    idr::Matrix3x3 I_clean = R_clean * R_clean.transpose();
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            double expected = (r == c) ? 1.0 : 0.0;
            assert_close(I_clean(r, c), expected, 1e-13, "Renormalized SO(3) orthonormality violated");
        }
    }
    assert_close(R_clean.determinant(), 1.0, 1e-12, "Renormalized SO(3) det != 1.0");

    std::cout << "  [PASS] Test 5: Polar decomposition SO(3) re-normalization\n";
}

} // namespace

int main() {
    std::cout << "============================================================\n";
    std::cout << "GATE A: C++17 MATHEMATICS & SO(3) VERIFICATION SUITE\n";
    std::cout << "============================================================\n";

    test_so3_orthonormality_and_det();
    test_frame_conventions_enu_flu();
    test_euler_dcm_roundtrip();
    test_matrix_inversion_and_eigen();
    test_so3_renormalization();

    std::cout << "============================================================\n";
    std::cout << "ALL GATE A MATHEMATICS TESTS PASSED [100%]\n";
    std::cout << "============================================================\n";
    return 0;
}
