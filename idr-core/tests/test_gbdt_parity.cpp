#include "idr/gbdt_model.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    std::cout << "============================================================\n";
    std::cout << "GATE B: GBDT MODEL TRANSPILATION & PARITY VERIFICATION\n";
    std::cout << "============================================================\n";

    // 1. Verify Model Metadata
    assert(idr::GBDTSpeedModel::NUM_FEATURES == 45);
    assert(idr::GBDTSpeedModel::NUM_TREES == 120);
    assert(idr::GBDTSpeedModel::LEARNING_RATE == 0.08);
    assert(std::abs(idr::GBDTSpeedModel::INIT_PREDICTION - 11.089444444444444) < 1e-12);
    assert(idr::GBDTSpeedModel::MIN_SPEED_MPS == 0.0);
    assert(idr::GBDTSpeedModel::MAX_SPEED_MPS == 35.0);
    std::cout << "  [PASS] Metadata verified: 120 trees, 45 features, LR=0.08, Base=11.0894\n";

    // 2. Open Parity Corpus Binary
    const std::vector<std::string> candidate_paths = {
        "tests/cross_language/gbdt_parity_corpus.bin",
        "../../tests/cross_language/gbdt_parity_corpus.bin",
        "../../../tests/cross_language/gbdt_parity_corpus.bin",
        "../tests/cross_language/gbdt_parity_corpus.bin",
        "/Users/akashmishra/SIH MOdel/tests/cross_language/gbdt_parity_corpus.bin"
    };

    std::ifstream file;
    std::string opened_path;
    for (const auto& path : candidate_paths) {
        file.open(path, std::ios::binary);
        if (file.is_open()) {
            opened_path = path;
            break;
        }
    }

    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open parity corpus. Checked paths:\n";
        for (const auto& path : candidate_paths) {
            std::cerr << "  - " << path << "\n";
        }
        return 1;
    }

    uint32_t num_vectors = 0;
    uint32_t num_features = 0;
    file.read(reinterpret_cast<char*>(&num_vectors), sizeof(num_vectors));
    file.read(reinterpret_cast<char*>(&num_features), sizeof(num_features));

    assert(num_features == 45);
    assert(num_vectors > 5000);
    std::cout << "  [+] Loaded parity corpus: " << num_vectors << " vectors, " << num_features << " features\n";

    std::vector<double> all_features(num_vectors * num_features);
    std::vector<double> py_predictions(num_vectors);

    file.read(reinterpret_cast<char*>(all_features.data()), num_vectors * num_features * sizeof(double));
    file.read(reinterpret_cast<char*>(py_predictions.data()), num_vectors * sizeof(double));

    // 3. Evaluate Predictions
    double max_abs_err = 0.0;
    double sum_abs_err = 0.0;
    double sum_sq_err = 0.0;
    uint32_t mismatch_count = 0;
    const double HARD_GATE_TOLERANCE = 1e-5;

    for (uint32_t i = 0; i < num_vectors; ++i) {
        const double* vec = &all_features[i * num_features];
        double py_pred = py_predictions[i];

        // Evaluate in C++
        double cpp_pred = idr::GBDTSpeedModel::predict(vec);

        // Determinism check (repeat evaluation)
        double cpp_pred_repeat = idr::GBDTSpeedModel::predict(vec);
        if (cpp_pred != cpp_pred_repeat) {
            std::cerr << "Non-deterministic inference at vector " << i << "!\n";
            return 1;
        }

        // Bounded output check [0.0, 35.0]
        if (cpp_pred < 0.0 || cpp_pred > 35.0) {
            std::cerr << "Prediction out of physical bounds [0, 35]: " << cpp_pred << " at vector " << i << "\n";
            return 1;
        }

        double diff = std::abs(cpp_pred - py_pred);
        if (diff > max_abs_err) {
            max_abs_err = diff;
        }
        sum_abs_err += diff;
        sum_sq_err += diff * diff;

        if (diff > HARD_GATE_TOLERANCE) {
            mismatch_count++;
            if (mismatch_count <= 5) {
                std::cerr << "MISMATCH at vector " << i << ": cpp=" << std::setprecision(10) << cpp_pred
                          << " vs py=" << py_pred << " (diff=" << diff << ")\n";
            }
        }
    }

    double mae = sum_abs_err / num_vectors;
    double rmse = std::sqrt(sum_sq_err / num_vectors);

    std::cout << "\n============================================================\n";
    std::cout << "GATE B: PARITY EVALUATION SUMMARY REPORT\n";
    std::cout << "============================================================\n";
    std::cout << "  Total Vectors Evaluated:    " << num_vectors << "\n";
    std::cout << "  Hard Gate Tolerance:        " << HARD_GATE_TOLERANCE << " m/s\n";
    std::cout << "  Max Absolute Error:         " << std::scientific << max_abs_err << " m/s\n";
    std::cout << "  Mean Absolute Error (MAE):  " << std::scientific << mae << " m/s\n";
    std::cout << "  Root Mean Squared (RMSE):   " << std::scientific << rmse << " m/s\n";
    std::cout << "  Mismatch Count (>1e-5 m/s): " << mismatch_count << "\n";
    std::cout << "============================================================\n";

    if (max_abs_err >= HARD_GATE_TOLERANCE || mismatch_count > 0) {
        std::cerr << "GATE B HARD GATE FAILED: max error exceeds " << HARD_GATE_TOLERANCE << " m/s!\n";
        return 1;
    }

    std::cout << "GATE B: GBDT MODEL TRANSPILATION & PARITY PASSED [100%]\n";
    std::cout << "============================================================\n";
    return 0;
}
