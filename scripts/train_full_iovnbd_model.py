"""
scripts/train_full_iovnbd_model.py
==================================
Trains the AI Speed Regressor (GBDT) on the FULL authentic IO-VNBD dataset
(964,771 synchronized multi-modal sensor frames across 71+ drives and 26.8 hours).

Features:
- Mount-invariant dynamic gravity decomposition.
- Road-roughness normalization (ZCR, vibration ratios, envelope entropy).
- Balanced per-drive resampling (mitigating highway sample dominance).
- Robust Huber loss regression (suppressing cobblestone impulse artifacts).
- Saves model checkpoint to models/gbdt_speed_model_iovnbd.joblib.

Author: Autonomous Navigation Engineering Team
"""

import os
import sys
import glob
import time
import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

# Ensure workspace root is in python path
sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import _find_column
from idr_pipeline.dataset_loader import extract_sliding_window_features, FEATURE_NAMES_39


def load_and_preprocess_pair(s_path: str, v_path: str, stride: int = 5):
    """Loads and extracts features from a synchronized CSV pair."""
    try:
        s_df = pd.read_csv(s_path, encoding='latin1', low_memory=False)
        v_df = pd.read_csv(v_path, encoding='latin1', low_memory=False)
    except Exception as e:
        print(f"[!] Error reading {s_path} or {v_path}: {e}")
        return None, None

    s_df.columns = [str(c).strip() for c in s_df.columns]
    v_df.columns = [str(c).strip() for c in v_df.columns]

    # Match IMU columns
    ax_col = _find_column(s_df.columns, r'accel.*(\bx\b|[_\s]x)')
    ay_col = _find_column(s_df.columns, r'accel.*(\by\b|[_\s]y)')
    az_col = _find_column(s_df.columns, r'accel.*(\bz\b|[_\s]z)')

    gx_col = _find_column(s_df.columns, r'gyro.*(roll|\b[xX]\b)')
    gy_col = _find_column(s_df.columns, r'gyro.*(pitch|\b[yY]\b)')
    gz_col = _find_column(s_df.columns, r'gyro.*(yaw|\b[zZ]\b)')

    v_spd_col = _find_column(v_df.columns, r'velo')

    # Restrict to common length
    N = min(len(s_df), len(v_df))
    if N < 50:
        return None, None

    def get_clean_col(df, col_name, default_val=0.0):
        if col_name and col_name in df.columns:
            s = pd.to_numeric(df[col_name].iloc[:N], errors='coerce').fillna(default_val).values
            return s
        return np.full(N, default_val)

    accel = np.column_stack([
        get_clean_col(s_df, ax_col, 0.0),
        get_clean_col(s_df, ay_col, 0.0),
        get_clean_col(s_df, az_col, 9.80665)
    ])

    gyro = np.column_stack([
        get_clean_col(s_df, gx_col, 0.0),
        get_clean_col(s_df, gy_col, 0.0),
        get_clean_col(s_df, gz_col, 0.0)
    ])

    v_speed_kmh = get_clean_col(v_df, v_spd_col, 0.0)
    v_speed_ms = v_speed_kmh / 3.6

    # Stage 1: In-Vehicle Alignment (Gravity Leveling + Longitudinal PCA)
    from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
    R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, gyro, v_speed_ms)
    accel_body = (R_p2b @ accel.T).T
    gyro_body = (R_p2b @ gyro.T).T

    # Extract 39 features in the aligned vehicle body frame
    window_size = 10
    X = extract_sliding_window_features(accel_body, gyro_body, window_size=window_size, stride=stride)

    # Target speeds corresponding to window ends (exact sample-to-sample synchronization)
    y = v_speed_ms[0 : N : stride]

    min_len = min(len(X), len(y))
    return X[:min_len], y[:min_len]



def train_full_model(max_samples_per_drive: int = 2000):
    print("=" * 78)
    print("AI SPEED REGRESSOR TRAINING: FULL AUTHENTIC IO-VNBD-ACTUAL DATASET")
    print("=" * 78)

    base_dir = "IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    if not os.path.exists(base_dir):
        base_dir = "synchronised_iovnbd_data/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    if not os.path.exists(base_dir):
        raise FileNotFoundError(f"Directory not found: {base_dir}")

    print(f"[*] Training dataset source: {base_dir}")

    # Discover all S files recursively
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Discovered {len(s_files)} smartphone log files across IO-VNBD.")


    # Held-out drives for rigorous multi-driver / multi-surface evaluation
    held_out_drive_ids = {'S3b', 'Vta01b', 'Vta04', 'Vtb01', 'Vw02'}

    X_train_list, y_train_list = [], []
    held_out_test_dict = {}

    total_drives_processed = 0
    t0 = time.time()

    category_stats = {}

    for s_path in s_files:
        dir_name = os.path.dirname(s_path)
        v_candidates = glob.glob(os.path.join(dir_name, "V-*.csv"))
        if not v_candidates:
            continue
        v_path = v_candidates[0]

        # Extract category and drive identifier
        rel_path = os.path.relpath(s_path, base_dir)
        parts = rel_path.split(os.sep)
        cat = parts[0]
        if len(parts) >= 3:
            drive_id = parts[1]
        else:
            drive_id = os.path.splitext(os.path.basename(s_path))[0].replace("S-", "")

        is_held_out = drive_id in held_out_drive_ids

        # Extract features with stride=5 (0.5 s step)
        X, y = load_and_preprocess_pair(s_path, v_path, stride=5)
        if X is None or len(X) == 0:
            continue

        if cat not in category_stats:
            category_stats[cat] = {'drives': 0, 'raw_windows': 0, 'train_windows': 0, 'test_windows': 0}
        category_stats[cat]['drives'] += 1
        category_stats[cat]['raw_windows'] += len(X)

        if is_held_out:
            held_out_test_dict[drive_id] = (X, y, cat)
            category_stats[cat]['test_windows'] += len(X)
        else:
            # Per-drive balanced resampling: uniform downsampling if length exceeds cap
            if len(X) > max_samples_per_drive:
                sub_idx = np.linspace(0, len(X) - 1, max_samples_per_drive, dtype=int)
                X_sampled = X[sub_idx]
                y_sampled = y[sub_idx]
            else:
                X_sampled = X
                y_sampled = y

            X_train_list.append(X_sampled)
            y_train_list.append(y_sampled)
            category_stats[cat]['train_windows'] += len(X_sampled)

        total_drives_processed += 1

    print("\n[*] Category Ingestion Breakdown (with balanced sampling cap):")
    for cat, stat in sorted(category_stats.items()):
        print(f"    - {cat:20s}: {stat['drives']:2d} drives | Raw: {stat['raw_windows']:6,d} | Train: {stat['train_windows']:6,d} | Test: {stat['test_windows']:5,d}")

    X_train = np.vstack(X_train_list)
    y_train = np.concatenate(y_train_list)

    X_test_all = np.vstack([t[0] for t in held_out_test_dict.values()])
    y_test_all = np.concatenate([t[1] for t in held_out_test_dict.values()])

    print(f"\n[*] Balanced Training Set: {len(X_train):,d} windows | Features: {X_train.shape[1]}")
    print(f"    Speed Range: {np.min(y_train)*3.6:.1f} km/h -> {np.max(y_train)*3.6:.1f} km/h (mean {np.mean(y_train)*3.6:.1f} km/h)")
    print(f"[*] Combined Held-Out Test Set: {len(X_test_all):,d} windows across {len(held_out_test_dict)} trips")

    # Standardize features
    print("\n[*] Standardizing features across training windows...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_all_scaled = scaler.transform(X_test_all)

    # Train GBDT with robust Huber loss to eliminate cobblestone/shock distortion
    print("[*] Training Gradient Boosting Regressor (Huber loss, n_estimators=120, max_depth=5, lr=0.08)...")
    t_train = time.time()
    model = GradientBoostingRegressor(
        loss='huber',
        alpha=0.90,
        n_estimators=120,
        max_depth=5,
        learning_rate=0.08,
        subsample=0.85,
        random_state=42,
        verbose=1
    )
    model.fit(X_train_scaled, y_train)
    print(f"[+] Model converged in {time.time() - t_train:.1f} s!")

    # Overall Metrics
    train_preds = np.clip(model.predict(X_train_scaled), 0.0, 35.0)
    test_preds = np.clip(model.predict(X_test_all_scaled), 0.0, 35.0)

    train_rmse = float(np.sqrt(mean_squared_error(y_train, train_preds)))
    train_mae = float(mean_absolute_error(y_train, train_preds))
    train_r2 = float(r2_score(y_train, train_preds))

    test_rmse = float(np.sqrt(mean_squared_error(y_test_all, test_preds)))
    test_mae = float(mean_absolute_error(y_test_all, test_preds))
    test_r2 = float(r2_score(y_test_all, test_preds))

    print("\n" + "=" * 78)
    print("GLOBAL PERFORMANCE METRICS:")
    print("=" * 78)
    print(f"  TRAIN ({len(X_train):,d} samples):")
    print(f"    - RMSE : {train_rmse:.3f} m/s ({train_rmse*3.6:.2f} km/h)")
    print(f"    - MAE  : {train_mae:.3f} m/s ({train_mae*3.6:.2f} km/h)")
    print(f"    - R²   : {train_r2:.4f}")
    print(f"  HELD-OUT TEST SUITE ({len(X_test_all):,d} samples across unseen trips):")
    print(f"    - RMSE : {test_rmse:.3f} m/s ({test_rmse*3.6:.2f} km/h)")
    print(f"    - MAE  : {test_mae:.3f} m/s ({test_mae*3.6:.2f} km/h)")
    print(f"    - R²   : {test_r2:.4f}")
    print("=" * 78)

    # Per-Trip Breakdown for Held-Out Evaluation
    print("\nHELD-OUT TEST SUITE TRIP-BY-TRIP BREAKDOWN:")
    print(f"{'Drive ID':<10s} | {'Category':<16s} | {'Windows':<7s} | {'Mean km/h':<9s} | {'RMSE (m/s)':<10s} | {'MAE (m/s)':<10s} | {'R²':<7s}")
    print("-" * 78)
    for drive_id, (X_d, y_d, cat_d) in sorted(held_out_test_dict.items()):
        X_d_scaled = scaler.transform(X_d)
        p_d = np.clip(model.predict(X_d_scaled), 0.0, 35.0)
        d_rmse = float(np.sqrt(mean_squared_error(y_d, p_d)))
        d_mae = float(mean_absolute_error(y_d, p_d))
        d_r2 = float(r2_score(y_d, p_d)) if np.var(y_d) > 1e-4 else 0.0
        print(f"{drive_id:<10s} | {cat_d:<16s} | {len(y_d):<7,d} | {np.mean(y_d)*3.6:<9.1f} | {d_rmse:<10.3f} | {d_mae:<10.3f} | {d_r2:<7.4f}")
    print("-" * 78)

    # Top Features
    importances = model.feature_importances_
    top_indices = np.argsort(importances)[::-1][:10]
    print("\n[*] Top 10 Most Predictive Physical Features:")
    for rank, idx in enumerate(top_indices, 1):
        name = FEATURE_NAMES_39[idx] if idx < len(FEATURE_NAMES_39) else f"feat_{idx}"
        print(f"    {rank:2d}. {name:<20s}: {importances[idx]*100:5.2f}%")

    # Save Checkpoint
    os.makedirs("models", exist_ok=True)
    checkpoint_path = "models/gbdt_speed_model_iovnbd.joblib"
    checkpoint = {
        "model": model,
        "scaler": scaler,
        "features": FEATURE_NAMES_39,
        "train_rmse": train_rmse,
        "test_rmse": test_rmse,
        "train_r2": train_r2,
        "test_r2": test_r2,
        "n_train_samples": len(X_train),
        "n_test_samples": len(X_test_all),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    joblib.dump(checkpoint, checkpoint_path)
    print(f"\n[+] Updated model checkpoint successfully saved to: {checkpoint_path}")
    print(f"    File size: {os.path.getsize(checkpoint_path) / (1024*1024):.2f} MB")


if __name__ == "__main__":
    train_full_model()
