#include "idr/math_utils.hpp"
#include "math_golden_vectors.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

#include <algorithm>

int main() {
    std::cout << "============================================================\n";
    std::cout << "GATE A: PYTHON <-> C++ MATHEMATICAL GOLDEN PARITY VERIFICATION\n";
    std::cout << "============================================================\n";

    double max_R_err = 0.0;
    double max_euler_err = 0.0;
    double max_rod_err = 0.0;
    double max_skew_err = 0.0;

    for (int i = 0; i < 100; ++i) {
        const auto& g = kMathGoldenVectors[i];

        // 1. Euler -> R
        idr::Matrix3x3 R_cpp = idr::euler_to_rotation_matrix(g.roll, g.pitch, g.yaw);
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                double diff = std::abs(R_cpp(r, c) - g.R[r * 3 + c]);
                if (diff > max_R_err) max_R_err = diff;
                if (diff > 1e-12) {
                    std::cerr << "Mismatch in R(" << r << "," << c << ") at vector " << i
                              << ": cpp=" << R_cpp(r, c) << " vs py=" << g.R[r * 3 + c] << "\n";
                    return 1;
                }
            }
        }

        // 2. R -> Euler
        auto euler_rec_cpp = idr::rotation_matrix_to_euler(R_cpp);
        double roll_rec = std::get<0>(euler_rec_cpp);
        double pitch_rec = std::get<1>(euler_rec_cpp);
        double yaw_rec = std::get<2>(euler_rec_cpp);
        double err_r = std::abs(roll_rec - g.euler_rec[0]);
        double err_p = std::abs(pitch_rec - g.euler_rec[1]);
        double err_y = std::abs(idr::wrap_pi(yaw_rec) - idr::wrap_pi(g.euler_rec[2]));
        max_euler_err = std::max(max_euler_err, std::max(err_r, std::max(err_p, err_y)));
        if (err_r > 1e-12 || err_p > 1e-12 || err_y > 1e-12) {
            std::cerr << "Mismatch in recovered Euler at vector " << i << "\n";
            return 1;
        }

        // 3. Rodrigues
        idr::Vector3 rot_vec{g.rot_vec[0], g.rot_vec[1], g.rot_vec[2]};
        idr::Matrix3x3 R_rod_cpp = idr::rodrigues(rot_vec);
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                double diff = std::abs(R_rod_cpp(r, c) - g.R_rod[r * 3 + c]);
                if (diff > max_rod_err) max_rod_err = diff;
                if (diff > 1e-12) {
                    std::cerr << "Mismatch in Rodrigues at vector " << i << "\n";
                    return 1;
                }
            }
        }

        // 4. Skew-symmetric
        idr::Vector3 v_rand{g.v_rand[0], g.v_rand[1], g.v_rand[2]};
        idr::Matrix3x3 K_cpp = idr::skew_symmetric(v_rand);
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) {
                double diff = std::abs(K_cpp(r, c) - g.K_skew[r * 3 + c]);
                if (diff > max_skew_err) max_skew_err = diff;
                if (diff > 1e-15) {
                    std::cerr << "Mismatch in Skew-symmetric at vector " << i << "\n";
                    return 1;
                }
            }
        }
    }

    std::cout << "  Verified 100 golden vectors with Python reference:\n";
    std::cout << "    Max Euler->DCM absolute error:      " << max_R_err << " (tol: 1e-12)\n";
    std::cout << "    Max DCM->Euler absolute error:      " << max_euler_err << " (tol: 1e-12)\n";
    std::cout << "    Max Rodrigues rotation error:       " << max_rod_err << " (tol: 1e-12)\n";
    std::cout << "    Max Skew-Symmetric matrix error:    " << max_skew_err << " (tol: 1e-15)\n";
    std::cout << "============================================================\n";
    std::cout << "GATE A GOLDEN PARITY PASSED [100%]\n";
    std::cout << "============================================================\n";
    return 0;
}
