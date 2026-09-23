#!/usr/bin/env python3
"""
scripts/generate_feature_vibration_golden_vectors.py
====================================================
Generates deterministic golden test vectors for Gate C:
1. SlidingWindowFeatureExtractor (all 57 features and Top-45 features)
2. VibrationPotholeFilter (shock detection, blanking, idle detection)
"""

import os
import sys
import struct
import joblib
import numpy as np

sys.path.insert(0, os.path.abspath('.'))
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.vibration_filter import VibrationPotholeFilter

def generate():
    np.random.seed(12345)
    ckpt = joblib.load("models/gbdt_speed_model_iovnbd_top45.joblib")
    indices = ckpt["indices"]

    N = 300
    accel = np.zeros((N, 3))
    gyro = np.zeros((N, 3))

    # Phase 1: Startup & cruising (0..100)
    for i in range(100):
        t = i * 0.1
        accel[i] = [0.2 * np.sin(0.5 * t), 0.1 * np.cos(0.4 * t), 9.80665 + 0.3 * np.sin(2.0 * t)]
        gyro[i] = [0.01 * np.cos(t), 0.02 * np.sin(t), 0.05 * np.sin(0.3 * t)]

    # Phase 2: Pothole shock at step 120 (35 m/s^2 vertical shock)
    for i in range(100, 150):
        accel[i] = [0.1, 0.0, 9.80665 + 0.1 * np.random.randn()]
        gyro[i] = [0.0, 0.0, 0.0]
    accel[120] = [0.5, 0.2, 35.0]

    # Phase 3: Engine idle harmonics at standstill (150..200)
    for i in range(150, 200):
        t = i * 0.1
        accel[i] = [0.05 * np.sin(t), 0.03 * np.cos(t), 9.80665 + 0.3 * np.sin(t * 3.0)]
        gyro[i] = [0.001 * np.sin(t), 0.001 * np.cos(t), 0.002 * np.sin(t)]

    # Phase 4: High dynamic turning & acceleration (200..300)
    for i in range(200, 300):
        t = i * 0.1
        accel[i] = [1.5 + 0.5 * np.sin(0.2 * t), 0.8 * np.sin(0.3 * t), 9.80665 + 0.4 * np.cos(t)]
        gyro[i] = [0.02 * np.sin(t), 0.03 * np.cos(t), 0.15 * np.sin(0.5 * t)]

    # 1. Feature Extraction (warmup through full window)
    f57_all = extract_sliding_window_features(
        accel, gyro, window_size=10, stride=1, dt=0.1,
        include_temporal_extrema=False, include_jerk=True, include_frequency=False
    )
    f45_all = f57_all[:, indices]

    # 2. Vibration Filter step-by-step
    v_filt = VibrationPotholeFilter(dt=0.1)
    clean_acc_all = []
    clean_gyr_all = []
    pothole_detected = []
    idling_detected = []
    pothole_counts = []
    vib_energies = []

    for i in range(N):
        ca, cg, st = v_filt.process(accel[i], gyro[i])
        clean_acc_all.append(ca)
        clean_gyr_all.append(cg)
        pothole_detected.append(1 if st.is_pothole_detected else 0)
        idling_detected.append(1 if st.is_engine_idling else 0)
        pothole_counts.append(st.pothole_event_count)
        vib_energies.append(st.vibration_energy)

    clean_acc_all = np.array(clean_acc_all, dtype=np.float64)
    clean_gyr_all = np.array(clean_gyr_all, dtype=np.float64)
    pothole_detected = np.array(pothole_detected, dtype=np.int32)
    idling_detected = np.array(idling_detected, dtype=np.int32)
    pothole_counts = np.array(pothole_counts, dtype=np.int32)
    vib_energies = np.array(vib_energies, dtype=np.float64)

    os.makedirs("tests/cross_language", exist_ok=True)
    bin_path = "tests/cross_language/feat_vib_golden.bin"

    with open(bin_path, "wb") as f:
        # N
        f.write(struct.pack("<I", N))
        # raw accel (N, 3), raw gyro (N, 3)
        f.write(accel.astype(np.float64).tobytes())
        f.write(gyro.astype(np.float64).tobytes())
        # f57 (N, 57), f45 (N, 45)
        f.write(f57_all.astype(np.float64).tobytes())
        f.write(f45_all.astype(np.float64).tobytes())
        # clean_acc (N, 3), clean_gyr (N, 3)
        f.write(clean_acc_all.tobytes())
        f.write(clean_gyr_all.tobytes())
        # flags and metrics
        f.write(pothole_detected.tobytes())
        f.write(idling_detected.tobytes())
        f.write(pothole_counts.tobytes())
        f.write(vib_energies.tobytes())

    print(f"[+] Wrote golden binary for Gate C -> {bin_path} ({N} steps)")

if __name__ == "__main__":
    generate()
