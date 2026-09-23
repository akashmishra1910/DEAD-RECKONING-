#!/usr/bin/env python3
"""
scripts/generate_gbdt_parity_corpus.py
======================================
Generates a comprehensive parity corpus for Gate B:
1. Normal synthetic vectors (random feature distributions)
2. Boundary / threshold vectors (features tested right at split thresholds)
3. Extreme / outlier vectors (shock spikes, saturating values)
4. Zero / near-zero vectors
5. Authentic IO-VNBD feature vectors from real driving logs
"""

import os
import sys
import struct
import joblib
import numpy as np

sys.path.insert(0, os.path.abspath('.'))
from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine

def generate_parity_corpus():
    np.random.seed(42)

    ckpt_path = "models/gbdt_speed_model_iovnbd_top45.joblib"
    print(f"[*] Loading model checkpoint: {ckpt_path}")
    ckpt = joblib.load(ckpt_path)
    model = ckpt["model"]
    scaler = ckpt["scaler"]
    features = ckpt["features"]
    indices = ckpt["indices"]

    all_vectors = []
    vector_types = []

    # 1. Zero and near-zero vectors
    print("[*] Generating zero and near-zero vectors...")
    zero_vec = np.zeros(45)
    all_vectors.append(zero_vec)
    vector_types.append("zero")

    for _ in range(50):
        nz = np.random.uniform(-1e-6, 1e-6, size=45)
        all_vectors.append(nz)
        vector_types.append("near_zero")

    # 2. Normal synthetic vectors matching scaler distribution
    print("[*] Generating normal synthetic vectors...")
    for _ in range(1000):
        # Sample standard normal, then inverse scale
        z = np.random.randn(45)
        x_raw = z * scaler.scale_ + scaler.mean_
        all_vectors.append(x_raw)
        vector_types.append("normal_synthetic")

    # 3. Boundary vectors: test at and around exact decision thresholds
    print("[*] Generating boundary vectors near split thresholds...")
    thresholds_by_feat = {i: [] for i in range(45)}
    for est in model.estimators_[:, 0]:
        t = est.tree_
        for n in range(t.node_count):
            if t.children_left[n] != -1:
                f_idx = t.feature[n]
                th = t.threshold[n]
                thresholds_by_feat[f_idx].append(th)

    for f_idx in range(45):
        ths = thresholds_by_feat[f_idx]
        if not ths:
            continue
        sample_ths = np.random.choice(ths, size=min(10, len(ths)), replace=False)
        for th in sample_ths:
            for eps in [-1e-7, 0.0, 1e-7]:
                vec_scaled = np.random.randn(45) * 0.5
                vec_scaled[f_idx] = th + eps
                vec_raw = vec_scaled * scaler.scale_ + scaler.mean_
                all_vectors.append(vec_raw)
                vector_types.append("boundary_threshold")

    # 4. Extreme outlier vectors (vibration shocks, braking, saturation)
    print("[*] Generating extreme / outlier vectors...")
    for _ in range(200):
        sign = np.random.choice([-1.0, 1.0], size=45)
        magnitude = np.random.uniform(10.0, 100.0, size=45)
        vec_scaled = sign * magnitude
        vec_raw = vec_scaled * scaler.scale_ + scaler.mean_
        all_vectors.append(vec_raw)
        vector_types.append("extreme_outlier")

    # 5. Authentic IO-VNBD vectors from real drives
    print("[*] Extracting authentic feature vectors from IO-VNBD dataset...")
    drives = RealIOVNBDLoader.load_all_drives('real_iovnbd_data')
    target_drives = ['Vta01b', 'Driver_A_S3b', 'Vta04', 'Vta03', 'Vta05']

    authentic_count = 0
    for drive_id in target_drives:
        if drive_id not in drives:
            continue
        drive = drives[drive_id]
        accel = drive.accel
        gyro = drive.gyro
        if len(accel) < 20:
            continue

        # Alignment
        R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, gyro)
        accel_b = (R_p2b @ accel.T).T
        gyro_b = (R_p2b @ gyro.T).T

        # Extract 57 features (39 baseline + 18 jerk)
        X_57 = extract_sliding_window_features(
            accel_b, gyro_b, window_size=10, stride=2, dt=0.1,
            include_temporal_extrema=False, include_jerk=True, include_frequency=False
        )
        X_45 = X_57[:, indices]

        for row in X_45:
            all_vectors.append(row)
            vector_types.append(f"authentic_{drive_id}")
            authentic_count += 1
            if authentic_count >= 2500:
                break
        if authentic_count >= 2500:
            break

    print(f"[*] Extracted {authentic_count} authentic vectors.")

    all_vectors = np.array(all_vectors, dtype=np.float64)
    N = len(all_vectors)
    print(f"[+] Total corpus size: {N} feature vectors.")

    # Compute Python reference predictions
    print("[*] Computing Python reference predictions with scikit-learn...")
    X_scaled = scaler.transform(all_vectors)
    preds_raw = model.predict(X_scaled)
    preds_clamped = np.clip(preds_raw, 0.0, 35.0)

    # Save to binary file for fast, exact, allocation-free C++ loading
    os.makedirs("tests/cross_language", exist_ok=True)
    bin_path = "tests/cross_language/gbdt_parity_corpus.bin"

    # Binary Format:
    # uint32_t num_vectors (N)
    # uint32_t num_features (45)
    # double features[N * 45]
    # double py_predictions[N]
    with open(bin_path, "wb") as f:
        f.write(struct.pack("<II", N, 45))
        f.write(all_vectors.tobytes())
        f.write(preds_clamped.tobytes())

    print(f"[+] Wrote binary corpus ({N} vectors) -> {bin_path}")

    # Summary by type
    from collections import Counter
    counts = Counter(vector_types)
    for vtype, cnt in counts.items():
        print(f"    - {vtype}: {cnt} vectors")

if __name__ == "__main__":
    generate_parity_corpus()
