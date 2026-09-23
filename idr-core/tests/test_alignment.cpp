#include "idr/alignment_engine.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <cassert>
#include <string>

using namespace idr;

int main() {
    std::cout << "[TEST] Starting Alignment Engine Golden Parity Tests..." << std::endl;

    InVehicleAlignmentEngine engine(STANDARD_GRAVITY, 25, 0.4, 5.0, 0.1);

    // G phone from json: [4.903324999999999, 1.4747606369939157, 8.363783188871427]
    Vector3 g_phone(4.903324999999999, 1.4747606369939157, 8.363783188871427);
    Vector3 gyro_stat(0.001, -0.002, 0.001);

    AlignmentState st;
    for (int i = 0; i < 30; ++i) {
        st = engine.process_sample(g_phone, gyro_stat, true);
    }

    assert(st.is_leveled == true);
    assert(st.is_aligned == false);
    assert(st.mode == AlignmentMode::STATIONARY_LEVELED);

    std::cout << "  Stationary roll_deg: " << st.roll_deg << " (target: 30.3813)" << std::endl;
    std::cout << "  Stationary pitch_deg: " << st.pitch_deg << " (target: -8.64917)" << std::endl;
    assert(std::abs(st.roll_deg - 30.381255) < 1e-3);
    assert(std::abs(st.pitch_deg - (-8.649165)) < 1e-3);
    assert(std::abs(st.quality.gravity_confidence - 1.0) < 1e-3);

    // Check R_p2b SO(3) condition
    Matrix3x3 R_p2b = st.R_p2b;
    double det = R_p2b.determinant();
    std::cout << "  R_p2b det: " << det << std::endl;
    assert(std::abs(det - 1.0) < 1e-12);

    // Test Dynamic Forward Surge
    // True R_true_p2b roll 10 deg, pitch 30 deg, yaw 0
    Matrix3x3 R_true_p2b = euler_to_rotation_matrix(10.0 * DEG_TO_RAD, 30.0 * DEG_TO_RAD, 0.0);
    Matrix3x3 R_true_b2p = R_true_p2b.transpose();

    for (int step = 0; step < 20; ++step) {
        Vector3 a_b(0.8 + 0.1 * std::sin(step), 0.0, 9.80665);
        Vector3 a_p = R_true_b2p * a_b;
        st = engine.process_sample(a_p, Vector3::zero(), false, 5.0);
    }

    assert(st.is_leveled == true);
    assert(st.is_aligned == true);
    assert(st.mode == AlignmentMode::DYNAMIC_ALIGNED);
    std::cout << "  Dynamic aligned yaw_deg: " << st.yaw_deg << " (target: -90.0)" << std::endl;
    assert(std::abs(st.yaw_deg - (-90.0)) < 1e-2);
    assert(st.quality.eigenvalue_ratio > 1000.0);

    // Test Mount Slip Detection
    Vector3 slip_g(5.0, 0.0, 8.4);
    Vector3 slip_w(1.2, 0.0, 0.0);
    st = engine.process_sample(slip_g, slip_w, true);
    assert(st.is_leveled == false);
    assert(st.is_aligned == false);
    assert(st.mode == AlignmentMode::RECALIBRATING);
    std::cout << "  Mount slip triggered recalibration successfully." << std::endl;

    // Test Batch Estimation
    std::vector<Vector3> batch_accels;
    for (int i = 0; i < 50; ++i) {
        Vector3 a_b(0.8 * std::sin(i * 0.1), 0.0, 9.80665);
        batch_accels.push_back(R_true_b2p * a_b);
    }
    AlignmentQuality b_qual;
    Matrix3x3 R_batch = InVehicleAlignmentEngine::estimate_mounting_matrix_batch(batch_accels, {}, {}, &b_qual);
    assert(b_qual.alignment_valid == true);
    assert(std::abs(R_batch.determinant() - 1.0) < 1e-12);
    std::cout << "  Batch mounting estimation passed SO(3) checks." << std::endl;

    std::cout << "[TEST] All Alignment Engine Tests PASSED!" << std::endl;
    return 0;
}
