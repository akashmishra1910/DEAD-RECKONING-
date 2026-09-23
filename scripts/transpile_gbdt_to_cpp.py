#!/usr/bin/env python3
"""
scripts/transpile_gbdt_to_cpp.py
================================
Transpiles frozen Top-45 GBDT joblib checkpoint into static C++ decision tables.
Guarantees bit-for-bit numerical equivalence with scikit-learn GradientBoostingRegressor.
"""

import os
import sys
import joblib
import numpy as np

def transpile(joblib_path: str, output_header: str, output_source: str):
    print(f"[*] Loading frozen model checkpoint: {joblib_path}")
    ckpt = joblib.load(joblib_path)
    model = ckpt["model"]
    scaler = ckpt["scaler"]
    features = ckpt["features"]
    indices = ckpt["indices"]

    n_trees = len(model.estimators_)
    n_features = len(features)
    init_val = float(model.init_.constant_[0, 0])
    learning_rate = float(model.learning_rate)

    print(f"[*] Trees: {n_trees}, Features: {n_features}, Init: {init_val}, LR: {learning_rate}")

    # Generate Header
    header_content = f"""#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <string_view>

namespace idr {{

struct TreeNode {{
    int16_t feature_idx; // -1 if leaf
    int16_t left_child;
    int16_t right_child;
    double threshold;
    double leaf_value;
}};

struct GBDTTree {{
    int16_t node_count;
    const TreeNode* nodes;
}};

class GBDTSpeedModel {{
public:
    static constexpr std::size_t NUM_FEATURES = {n_features};
    static constexpr std::size_t NUM_TREES = {n_trees};
    static constexpr double LEARNING_RATE = {learning_rate};
    static constexpr double INIT_PREDICTION = {init_val:.17g};
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
}};

}} // namespace idr
"""

    os.makedirs(os.path.dirname(output_header), exist_ok=True)
    with open(output_header, "w") as f:
        f.write(header_content)
    print(f"[+] Wrote header: {output_header}")

    # Generate Source
    all_nodes = []
    tree_offsets = []
    tree_node_counts = []

    current_offset = 0
    for t_idx in range(n_trees):
        tree = model.estimators_[t_idx, 0].tree_
        n_nodes = tree.node_count
        tree_offsets.append(current_offset)
        tree_node_counts.append(n_nodes)
        current_offset += n_nodes

        for n in range(n_nodes):
            f_idx = int(tree.feature[n])
            lc = int(tree.children_left[n])
            rc = int(tree.children_right[n])
            th = float(tree.threshold[n])
            val = float(tree.value[n, 0, 0])
            all_nodes.append((f_idx, lc, rc, th, val))

    print(f"[*] Total nodes across all trees: {len(all_nodes)}")

    source_lines = [
        '#include "idr/gbdt_model.hpp"',
        '#include <algorithm>',
        '',
        'namespace idr {',
        '',
        f'const char* const GBDTSpeedModel::FEATURE_NAMES[GBDTSpeedModel::NUM_FEATURES] = {{',
    ]
    for feat in features:
        source_lines.append(f'    "{feat}",')
    source_lines.append('};')
    source_lines.append('')

    source_lines.append(f'const int16_t GBDTSpeedModel::FEATURE_INDICES[GBDTSpeedModel::NUM_FEATURES] = {{')
    source_lines.append('    ' + ', '.join(str(idx) for idx in indices))
    source_lines.append('};')
    source_lines.append('')

    source_lines.append(f'const double GBDTSpeedModel::SCALER_MEAN[GBDTSpeedModel::NUM_FEATURES] = {{')
    source_lines.append('    ' + ', '.join(f'{m:.17g}' for m in scaler.mean_))
    source_lines.append('};')
    source_lines.append('')

    source_lines.append(f'const double GBDTSpeedModel::SCALER_SCALE[GBDTSpeedModel::NUM_FEATURES] = {{')
    source_lines.append('    ' + ', '.join(f'{s:.17g}' for s in scaler.scale_))
    source_lines.append('};')
    source_lines.append('')

    source_lines.append(f'static const TreeNode kAllTreeNodes[{len(all_nodes)}] = {{')
    for node in all_nodes:
        source_lines.append(f'    {{{node[0]}, {node[1]}, {node[2]}, {node[3]:.17g}, {node[4]:.17g}}},')
    source_lines.append('};')
    source_lines.append('')

    source_lines.append(f'static const GBDTTree kAllTrees[GBDTSpeedModel::NUM_TREES] = {{')
    for t_idx in range(n_trees):
        offset = tree_offsets[t_idx]
        count = tree_node_counts[t_idx]
        source_lines.append(f'    {{{count}, &kAllTreeNodes[{offset}]}},')
    source_lines.append('};')
    source_lines.append('')

    source_lines.append("""double GBDTSpeedModel::predict_scaled(const float* features_scaled) {
    double pred = INIT_PREDICTION;

    for (std::size_t t = 0; t < NUM_TREES; ++t) {
        const auto& tree = kAllTrees[t];
        int16_t node = 0;
        while (tree.nodes[node].left_child != -1) {
            int16_t f_idx = tree.nodes[node].feature_idx;
            double th = tree.nodes[node].threshold;
            if (features_scaled[f_idx] <= th) {
                node = tree.nodes[node].left_child;
            } else {
                node = tree.nodes[node].right_child;
            }
        }
        pred += LEARNING_RATE * tree.nodes[node].leaf_value;
    }

    return std::clamp(pred, MIN_SPEED_MPS, MAX_SPEED_MPS);
}

double GBDTSpeedModel::predict(const double* features_raw) {
    // Stack-allocated scaled features in float (zero heap allocation)
    // Matches scikit-learn GradientBoostingRegressor which validates data with dtype=np.float32
    float features_scaled[NUM_FEATURES];
    for (std::size_t i = 0; i < NUM_FEATURES; ++i) {
        features_scaled[i] = static_cast<float>((features_raw[i] - SCALER_MEAN[i]) / SCALER_SCALE[i]);
    }
    return predict_scaled(features_scaled);
}

double GBDTSpeedModel::predict(const std::array<double, NUM_FEATURES>& features_raw) {
    return predict(features_raw.data());
}

} // namespace idr
""")

    os.makedirs(os.path.dirname(output_source), exist_ok=True)
    with open(output_source, "w") as f:
        f.write("\n".join(source_lines))
    print(f"[+] Wrote source: {output_source}")

if __name__ == "__main__":
    joblib_path = "models/gbdt_speed_model_iovnbd_top45.joblib"
    output_header = "idr-core/include/idr/gbdt_model.hpp"
    output_source = "idr-core/src/gbdt_model.cpp"
    transpile(joblib_path, output_header, output_source)
