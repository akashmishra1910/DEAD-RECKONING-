#include "idr/feature_extractor.hpp"
#include "idr/vibration_filter.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main() {
    std::cout << "============================================================\n";
    std::cout << "GATE C: FEATURE EXTRACTION & VIBRATION FILTER VERIFICATION\n";
    std::cout << "============================================================\n";

    const std::vector<std::string> candidate_paths = {
        "tests/cross_language/feat_vib_golden.bin",
        "../../tests/cross_language/feat_vib_golden.bin",
        "../../../tests/cross_language/feat_vib_golden.bin",
        "../tests/cross_language/feat_vib_golden.bin",
        "/Users/akashmishra/SIH MOdel/tests/cross_language/feat_vib_golden.bin"
    };

    std::ifstream file;
    for (const auto& p : candidate_paths) {
        file.open(p, std::ios::binary);
        if (file.is_open()) break;
    }
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open feat_vib_golden.bin\n";
        return 1;
    }

    uint32_t N = 0;
    file.read(reinterpret_cast<char*>(&N), sizeof(N));
    assert(N == 300);

    std::vector<double> raw_acc(N * 3);
    std::vector<double> raw_gyr(N * 3);
    std::vector<double> py_f57(N * 57);
    std::vector<double> py_f45(N * 45);
    std::vector<double> py_clean_acc(N * 3);
    std::vector<double> py_clean_gyr(N * 3);
    std::vector<int32_t> py_pothole(N);
    std::vector<int32_t> py_idling(N);
    std::vector<int32_t> py_pothole_cnt(N);
    std::vector<double> py_vib_energy(N);

    file.read(reinterpret_cast<char*>(raw_acc.data()), N * 3 * sizeof(double));
    file.read(reinterpret_cast<char*>(raw_gyr.data()), N * 3 * sizeof(double));
    file.read(reinterpret_cast<char*>(py_f57.data()), N * 57 * sizeof(double));
    file.read(reinterpret_cast<char*>(py_f45.data()), N * 45 * sizeof(double));
    file.read(reinterpret_cast<char*>(py_clean_acc.data()), N * 3 * sizeof(double));
    file.read(reinterpret_cast<char*>(py_clean_gyr.data()), N * 3 * sizeof(double));
    file.read(reinterpret_cast<char*>(py_pothole.data()), N * sizeof(int32_t));
    file.read(reinterpret_cast<char*>(py_idling.data()), N * sizeof(int32_t));
    file.read(reinterpret_cast<char*>(py_pothole_cnt.data()), N * sizeof(int32_t));
    file.read(reinterpret_cast<char*>(py_vib_energy.data()), N * sizeof(double));

    // Test 1: Feature Extractor
    idr::SlidingWindowFeatureExtractor extractor(0.1);
    double max_f57_err = 0.0;
    double max_f45_err = 0.0;

    for (uint32_t i = 0; i < N; ++i) {
        idr::Vector3 a{raw_acc[i * 3 + 0], raw_acc[i * 3 + 1], raw_acc[i * 3 + 2]};
        idr::Vector3 g{raw_gyr[i * 3 + 0], raw_gyr[i * 3 + 1], raw_gyr[i * 3 + 2]};
        extractor.push(a, g);

        double f57[57];
        double f45[45];
        extractor.compute_57_features(f57);
        extractor.compute_45_features(f45);

        for (int k = 0; k < 57; ++k) {
            double diff = std::abs(f57[k] - py_f57[i * 57 + k]);
            if (diff > max_f57_err) {
                max_f57_err = diff;
                if (diff > 1e-10) {
                    std::cout << "  [feat " << k << " at step " << i << "]: diff=" << diff
                              << " cpp=" << f57[k] << " py=" << py_f57[i * 57 + k] << "\n";
                }
            }
        }

        for (int k = 0; k < 45; ++k) {
            double diff = std::abs(f45[k] - py_f45[i * 45 + k]);
            if (diff > max_f45_err) max_f45_err = diff;
        }
    }

    std::cout << "  [1] Feature Extractor Parity:\n";
    std::cout << "      Max error across all 57 raw features:  " << max_f57_err << " (tol: 1e-7)\n";
    std::cout << "      Max error across Top-45 features:      " << max_f45_err << " (tol: 1e-7)\n";
    assert(max_f57_err < 1e-7);
    assert(max_f45_err < 1e-7);

    // Test 2: Vibration Pothole Filter
    idr::VibrationPotholeFilter filter(0.1);
    double max_clean_acc_err = 0.0;
    double max_clean_gyr_err = 0.0;
    double max_vib_err = 0.0;
    uint32_t pothole_flag_diffs = 0;
    uint32_t idling_flag_diffs = 0;

    for (uint32_t i = 0; i < N; ++i) {
        idr::Vector3 a{raw_acc[i * 3 + 0], raw_acc[i * 3 + 1], raw_acc[i * 3 + 2]};
        idr::Vector3 g{raw_gyr[i * 3 + 0], raw_gyr[i * 3 + 1], raw_gyr[i * 3 + 2]};

        idr::VibrationState st = filter.process(a, g);

        double diff_ax = std::abs(st.cleaned_accel.x - py_clean_acc[i * 3 + 0]);
        double diff_ay = std::abs(st.cleaned_accel.y - py_clean_acc[i * 3 + 1]);
        double diff_az = std::abs(st.cleaned_accel.z - py_clean_acc[i * 3 + 2]);
        max_clean_acc_err = std::max({max_clean_acc_err, diff_ax, diff_ay, diff_az});

        double diff_gx = std::abs(st.cleaned_gyro.x - py_clean_gyr[i * 3 + 0]);
        double diff_gy = std::abs(st.cleaned_gyro.y - py_clean_gyr[i * 3 + 1]);
        double diff_gz = std::abs(st.cleaned_gyro.z - py_clean_gyr[i * 3 + 2]);
        max_clean_gyr_err = std::max({max_clean_gyr_err, diff_gx, diff_gy, diff_gz});

        double diff_vib = std::abs(st.vibration_energy - py_vib_energy[i]);
        if (diff_vib > max_vib_err) max_vib_err = diff_vib;

        if ((st.is_pothole_detected ? 1 : 0) != py_pothole[i]) pothole_flag_diffs++;
        if ((st.is_engine_idling ? 1 : 0) != py_idling[i]) idling_flag_diffs++;
        assert(st.pothole_event_count == static_cast<uint32_t>(py_pothole_cnt[i]));
    }

    std::cout << "  [2] Vibration & Pothole Filter Parity:\n";
    std::cout << "      Max Cleaned Accel Error: " << max_clean_acc_err << " (tol: 1e-12)\n";
    std::cout << "      Max Cleaned Gyro Error:  " << max_clean_gyr_err << " (tol: 1e-12)\n";
    std::cout << "      Max Vibration Energy:    " << max_vib_err << " (tol: 1e-12)\n";
    std::cout << "      Pothole Flag Mismatches: " << pothole_flag_diffs << "\n";
    std::cout << "      Idling Flag Mismatches:  " << idling_flag_diffs << "\n";

    assert(max_clean_acc_err < 1e-12);
    assert(max_clean_gyr_err < 1e-12);
    assert(max_vib_err < 1e-12);
    assert(pothole_flag_diffs == 0);
    assert(idling_flag_diffs == 0);

    std::cout << "============================================================\n";
    std::cout << "GATE C: FEATURE EXTRACTION & VIBRATION FILTER PASSED [100%]\n";
    std::cout << "============================================================\n";
    return 0;
}
