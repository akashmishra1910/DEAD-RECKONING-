#pragma once

#include "idr/types.hpp"

#include <array>
#include <cstdint>
#include <cstddef>


namespace idr {

struct TreeNode {
    int16_t feature_idx; // -1 if leaf
    int16_t left_child;
    int16_t right_child;
    double threshold;
    double leaf_value;
};

struct GBDTTree {
    int16_t node_count;
    const TreeNode* nodes;
};

class GBDTSpeedModel {
public:
    static constexpr std::size_t NUM_FEATURES = 45;
    static constexpr std::size_t NUM_TREES = 120;
    static constexpr double LEARNING_RATE = 0.08;
    static constexpr double INIT_PREDICTION = 11.089444444444444;
    static constexpr double MIN_SPEED_MPS = 0.0;
    static constexpr double MAX_SPEED_MPS = 35.0;

    static const char* const FEATURE_NAMES[NUM_FEATURES];
    static const int16_t FEATURE_INDICES[NUM_FEATURES];
    static const double SCALER_MEAN[NUM_FEATURES];
    static const double SCALER_SCALE[NUM_FEATURES];

    // Predict forward speed in m/s from unscaled 45-feature vector
    // Matches scikit-learn which converts features to np.float32 before tree evaluation
    // Zero heap allocation, deterministic O(depth * n_trees) execution
    static double predict(const double* features_raw);
    static double predict(const std::array<double, NUM_FEATURES>& features_raw);

    // Predict forward speed from pre-scaled 45-feature vector (float32)
    static double predict_scaled(const float* features_scaled);
};

} // namespace idr
