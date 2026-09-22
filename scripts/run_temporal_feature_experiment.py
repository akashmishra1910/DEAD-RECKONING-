"""
scripts/run_temporal_feature_experiment.py
=========================================
Executes a rigorous BEFORE vs. AFTER experiment comparing:
- BEFORE: Current GBDT model (39 features)
- AFTER:  Temporal-feature GBDT model (51 features: 39 + 12 newly added extrema)

Identical experimental conditions:
- Dataset: Authentic IO-VNBD (72 synchronized trips)
- Train/Test Split: 67 training drives vs 5 held-out test drives (S3b, Vta01b, Vta04, Vtb01, Vw02)
- Window length: 1.0s (10 samples @ 10 Hz)
- Preprocessing: Stage 1 In-Vehicle Mounting Calibration (PCA) + StandardScaler
- Hyperparameters: Huber loss (alpha=0.90), n_estimators=120, max_depth=5, lr=0.08, subsample=0.85, random_state=42
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

sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import _find_column
from idr_pipeline.dataset_loader import (
    extract_sliding_window_features,
    FEATURE_NAMES_39,
    FEATURE_NAMES_51,
    NEW_TEMPORAL_FEATURES_12
)
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine


def load_drive_features(s_path: str, v_path: str, stride: int = 5):
    """Loads a drive pair and extracts both 39-feature and 51-feature representations."""
    try:
        s_df = pd.read_csv(s_path, encoding='latin1', low_memory=False)
        v_df = pd.read_csv(v_path, encoding='latin1', low_memory=False)
    except Exception as e:
        print(f"[!] Error reading {s_path}: {e}")
        return None, None

    s_df.columns = [str(c).strip() for c in s_df.columns]
    v_df.columns = [str(c).strip() for c in v_df.columns]

    ax_col = _find_column(s_df.columns, r'accel.*(\bx\b|[_\s]x)')
    ay_col = _find_column(s_df.columns, r'accel.*(\by\b|[_\s]y)')
    az_col = _find_column(s_df.columns, r'accel.*(\bz\b|[_\s]z)')

    gx_col = _find_column(s_df.columns, r'gyro.*(roll|\b[xX]\b)')
    gy_col = _find_column(s_df.columns, r'gyro.*(pitch|\b[yY]\b)')
    gz_col = _find_column(s_df.columns, r'gyro.*(yaw|\b[zZ]\b)')

    v_spd_col = _find_column(v_df.columns, r'velo')

    N = min(len(s_df), len(v_df))
    if N < 50:
        return None, None

    def get_clean_col(df, col_name, default_val=0.0):
        if col_name and col_name in df.columns:
            return pd.to_numeric(df[col_name].iloc[:N], errors='coerce').fillna(default_val).values
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

    # Stage 1: In-vehicle mounting calibration (same for both)
    R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, gyro, v_speed_ms)
    accel_b = (R_p2b @ accel.T).T
    gyro_b = (R_p2b @ gyro.T).T

    # Extract 51 features (first 39 are identical to baseline)
    X_51 = extract_sliding_window_features(accel_b, gyro_b, window_size=10, stride=stride, include_temporal_extrema=True)
    y = v_speed_ms[0 : N : stride]

    min_len = min(len(X_51), len(y))
    return X_51[:min_len], y[:min_len]


def run_experiment(max_samples_per_drive: int = 2000):
    print("=" * 80)
    print("EXPERIMENT: EVALUATING GBDT TEMPORAL/STATISTICAL FEATURE ENHANCEMENT")
    print("=" * 80)

    base_dir = "IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Total discovered trips: {len(s_files)}")

    held_out_drive_ids = {'S3b', 'Vta01b', 'Vta04', 'Vtb01', 'Vw02'}

    X_train_51_list, y_train_list = [], []
    held_out_test_dict = {}

    print("[*] Ingesting and extracting features from drives...")
    t_ingest = time.time()
    for s_path in s_files:
        dir_name = os.path.dirname(s_path)
        v_candidates = glob.glob(os.path.join(dir_name, "V-*.csv"))
        if not v_candidates:
            continue
        v_path = v_candidates[0]

        rel_path = os.path.relpath(s_path, base_dir)
        parts = rel_path.split(os.sep)
        cat = parts[0]
        drive_id = parts[1] if len(parts) >= 3 else os.path.splitext(os.path.basename(s_path))[0].replace("S-", "")

        X_d_51, y_d = load_drive_features(s_path, v_path, stride=5)
        if X_d_51 is None or len(X_d_51) == 0:
            continue

        if drive_id in held_out_drive_ids:
            held_out_test_dict[drive_id] = (X_d_51, y_d, cat)
        else:
            if len(X_d_51) > max_samples_per_drive:
                sub_idx = np.linspace(0, len(X_d_51) - 1, max_samples_per_drive, dtype=int)
                X_sampled = X_d_51[sub_idx]
                y_sampled = y_d[sub_idx]
            else:
                X_sampled = X_d_51
                y_sampled = y_d
            X_train_51_list.append(X_sampled)
            y_train_list.append(y_sampled)

    print(f"[+] Ingestion completed in {time.time() - t_ingest:.1f} s")

    X_train_51 = np.vstack(X_train_51_list)
    y_train = np.concatenate(y_train_list)

    X_test_51 = np.vstack([t[0] for t in held_out_test_dict.values()])
    y_test = np.concatenate([t[1] for t in held_out_test_dict.values()])

    # Split into 39 vs 51 feature representations
    X_train_39 = X_train_51[:, :39]
    X_test_39 = X_test_51[:, :39]

    print(f"[*] Training Windows: {len(y_train):,d} | Test Windows: {len(y_test):,d}")
    print(f"[*] Baseline Feature Dim: {X_train_39.shape[1]} | Augmented Feature Dim: {X_train_51.shape[1]}")

    # Standardizers
    scaler_39 = StandardScaler()
    X_train_39_scaled = scaler_39.fit_transform(X_train_39)
    X_test_39_scaled = scaler_39.transform(X_test_39)

    scaler_51 = StandardScaler()
    X_train_51_scaled = scaler_51.fit_transform(X_train_51)
    X_test_51_scaled = scaler_51.transform(X_test_51)

    gbdt_kwargs = dict(
        loss='huber',
        alpha=0.90,
        n_estimators=120,
        max_depth=5,
        learning_rate=0.08,
        subsample=0.85,
        random_state=42
    )

    # =========================================================================
    # MODEL 1: BEFORE (39 Features)
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL 1: BEFORE (Existing 39 Features)")
    print("=" * 50)
    model_39 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_39 = time.time()
    model_39.fit(X_train_39_scaled, y_train)
    train_time_39 = time.time() - t0_39
    print(f"[+] Model 1 (39 feats) trained in {train_time_39:.2f} s")

    # Benchmarking inference latency
    warmup = model_39.predict(X_test_39_scaled[:100])
    t_lat0 = time.time()
    for _ in range(10):
        _ = model_39.predict(X_test_39_scaled[:1000])
    lat_39_us = (time.time() - t_lat0) / (10 * 1000) * 1e6

    # Predictions
    pred_train_39 = np.clip(model_39.predict(X_train_39_scaled), 0.0, 35.0)
    pred_test_39 = np.clip(model_39.predict(X_test_39_scaled), 0.0, 35.0)

    rmse_train_39 = np.sqrt(mean_squared_error(y_train, pred_train_39))
    mae_train_39 = mean_absolute_error(y_train, pred_train_39)
    r2_train_39 = r2_score(y_train, pred_train_39)

    rmse_test_39 = np.sqrt(mean_squared_error(y_test, pred_test_39))
    mae_test_39 = mean_absolute_error(y_test, pred_test_39)
    r2_test_39 = r2_score(y_test, pred_test_39)

    # 0-60 km/h subset evaluation (< 16.67 m/s)
    mask_60_train = y_train <= (60.0 / 3.6)
    mask_60_test = y_test <= (60.0 / 3.6)

    rmse_60_train_39 = np.sqrt(mean_squared_error(y_train[mask_60_train], pred_train_39[mask_60_train]))
    r2_60_train_39 = r2_score(y_train[mask_60_train], pred_train_39[mask_60_train])
    rmse_60_test_39 = np.sqrt(mean_squared_error(y_test[mask_60_test], pred_test_39[mask_60_test]))
    r2_60_test_39 = r2_score(y_test[mask_60_test], pred_test_39[mask_60_test])

    # Save temp checkpoint to measure file size
    joblib.dump(model_39, "models/temp_model_39.joblib")
    size_39_kb = os.path.getsize("models/temp_model_39.joblib") / 1024.0

    # =========================================================================
    # MODEL 2: AFTER (51 Features: 39 + 12 Temporal Extrema)
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL 2: AFTER (Augmented 51 Features)")
    print("=" * 50)
    model_51 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_51 = time.time()
    model_51.fit(X_train_51_scaled, y_train)
    train_time_51 = time.time() - t0_51
    print(f"[+] Model 2 (51 feats) trained in {train_time_51:.2f} s")

    # Benchmarking inference latency
    warmup = model_51.predict(X_test_51_scaled[:100])
    t_lat1 = time.time()
    for _ in range(10):
        _ = model_51.predict(X_test_51_scaled[:1000])
    lat_51_us = (time.time() - t_lat1) / (10 * 1000) * 1e6

    # Predictions
    pred_train_51 = np.clip(model_51.predict(X_train_51_scaled), 0.0, 35.0)
    pred_test_51 = np.clip(model_51.predict(X_test_51_scaled), 0.0, 35.0)

    rmse_train_51 = np.sqrt(mean_squared_error(y_train, pred_train_51))
    mae_train_51 = mean_absolute_error(y_train, pred_train_51)
    r2_train_51 = r2_score(y_train, pred_train_51)

    rmse_test_51 = np.sqrt(mean_squared_error(y_test, pred_test_51))
    mae_test_51 = mean_absolute_error(y_test, pred_test_51)
    r2_test_51 = r2_score(y_test, pred_test_51)

    rmse_60_train_51 = np.sqrt(mean_squared_error(y_train[mask_60_train], pred_train_51[mask_60_train]))
    r2_60_train_51 = r2_score(y_train[mask_60_train], pred_train_51[mask_60_train])
    rmse_60_test_51 = np.sqrt(mean_squared_error(y_test[mask_60_test], pred_test_51[mask_60_test]))
    r2_60_test_51 = r2_score(y_test[mask_60_test], pred_test_51[mask_60_test])

    joblib.dump(model_51, "models/temp_model_51.joblib")
    size_51_kb = os.path.getsize("models/temp_model_51.joblib") / 1024.0

    # Clean up temp models
    if os.path.exists("models/temp_model_39.joblib"):
        os.remove("models/temp_model_39.joblib")
    if os.path.exists("models/temp_model_51.joblib"):
        os.remove("models/temp_model_51.joblib")

    # =========================================================================
    # COMPARISON TABLE
    # =========================================================================
    print("\n" + "=" * 80)
    print("MODEL COMPARISON RESULTS: BEFORE (39 Feats) vs AFTER (51 Feats)")
    print("=" * 80)
    print(f"{'Metric':<35s} | {'Existing GBDT (39)':<20s} | {'Temporal GBDT (51)':<20s} | {'Change':<15s}")
    print("-" * 80)

    rows = [
        ("Feature Count", f"{39}", f"{51}", f"+12 features"),
        ("Training Time", f"{train_time_39:.2f} s", f"{train_time_51:.2f} s", f"{train_time_51 - train_time_39:+.2f} s"),
        ("Model Size (joblib)", f"{size_39_kb:.1f} KB", f"{size_51_kb:.1f} KB", f"{size_51_kb - size_39_kb:+.1f} KB"),
        ("Inference Latency", f"{lat_39_us:.2f} us/sample", f"{lat_51_us:.2f} us/sample", f"{lat_51_us - lat_39_us:+.2f} us"),
        ("Train Speed RMSE", f"{rmse_train_39:.3f} m/s", f"{rmse_train_51:.3f} m/s", f"{rmse_train_51 - rmse_train_39:+.3f} m/s"),
        ("Train Speed MAE", f"{mae_train_39:.3f} m/s", f"{mae_train_51:.3f} m/s", f"{mae_train_51 - mae_train_39:+.3f} m/s"),
        ("Train Speed R2", f"{r2_train_39:.4f}", f"{r2_train_51:.4f}", f"{r2_train_51 - r2_train_39:+.4f}"),
        ("Held-Out Test RMSE (All)", f"{rmse_test_39:.3f} m/s", f"{rmse_test_51:.3f} m/s", f"{rmse_test_51 - rmse_test_39:+.3f} m/s"),
        ("Held-Out Test MAE (All)", f"{mae_test_39:.3f} m/s", f"{mae_test_51:.3f} m/s", f"{mae_test_51 - mae_test_39:+.3f} m/s"),
        ("Held-Out Test R2 (All)", f"{r2_test_39:.4f}", f"{r2_test_51:.4f}", f"{r2_test_51 - r2_test_39:+.4f}"),
        ("0-60 km/h Test RMSE", f"{rmse_60_test_39:.3f} m/s", f"{rmse_60_test_51:.3f} m/s", f"{rmse_60_test_51 - rmse_60_test_39:+.3f} m/s"),
        ("0-60 km/h Test R2", f"{r2_60_test_39:.4f}", f"{r2_60_test_51:.4f}", f"{r2_60_test_51 - r2_60_test_39:+.4f}")
    ]

    for name, m39, m51, chg in rows:
        print(f"{name:<35s} | {m39:<20s} | {m51:<20s} | {chg:<15s}")
    print("-" * 80)

    # Per-Trip Breakdown Comparison
    print("\nPER-TRIP HELD-OUT SUITE BREAKDOWN (Test RMSE in m/s):")
    print(f"{'Drive ID':<10s} | {'Category':<16s} | {'Windows':<7s} | {'Existing 39 RMSE':<18s} | {'Temporal 51 RMSE':<18s} | {'Delta':<10s}")
    print("-" * 85)
    for drive_id, (X_d_51, y_d, cat_d) in sorted(held_out_test_dict.items()):
        X_d_39_sc = scaler_39.transform(X_d_51[:, :39])
        X_d_51_sc = scaler_51.transform(X_d_51)
        p39 = np.clip(model_39.predict(X_d_39_sc), 0.0, 35.0)
        p51 = np.clip(model_51.predict(X_d_51_sc), 0.0, 35.0)
        r39 = np.sqrt(mean_squared_error(y_d, p39))
        r51 = np.sqrt(mean_squared_error(y_d, p51))
        delta = r51 - r39
        print(f"{drive_id:<10s} | {cat_d:<16s} | {len(y_d):<7,d} | {r39:<18.3f} | {r51:<18.3f} | {delta:+.3f}")
    print("-" * 85)

    # Feature Importance of 51 features
    importances = model_51.feature_importances_
    sorted_idx = np.argsort(importances)[::-1]

    print("\n" + "=" * 80)
    print("FEATURE IMPORTANCE RANKING (51-FEATURE MODEL)")
    print("=" * 80)
    print(f"{'Rank':<5s} | {'Feature Name':<20s} | {'Importance %':<14s} | {'Type':<18s}")
    print("-" * 65)

    new_feat_set = set(NEW_TEMPORAL_FEATURES_12)
    for rank, idx in enumerate(sorted_idx, 1):
        name = FEATURE_NAMES_51[idx]
        imp_pct = importances[idx] * 100.0
        f_type = "NEW TEMPORAL" if name in new_feat_set else "Existing (39)"
        if rank <= 15 or name in new_feat_set:
            print(f"{rank:<5d} | {name:<20s} | {imp_pct:6.2f}%       | {f_type:<18s}")
    print("-" * 65)

    print("\nNEWLY ADDED TEMPORAL FEATURES SUMMARY:")
    new_imp_total = 0.0
    for name in NEW_TEMPORAL_FEATURES_12:
        idx = FEATURE_NAMES_51.index(name)
        imp = importances[idx] * 100.0
        rank = list(sorted_idx).index(idx) + 1
        new_imp_total += imp
        print(f"  - {name:<10s}: Rank {rank:2d}/51 | Importance: {imp:5.2f}%")
    print(f"\nTotal Importance of 12 New Temporal Features: {new_imp_total:.2f}%")

    # If the new model improves performance, save it as updated checkpoint
    if rmse_test_51 <= rmse_test_39:
        print("\n[+] The 51-feature model achieved improved/equal performance! Saving checkpoint...")
        checkpoint_path = "models/gbdt_speed_model_iovnbd.joblib"
        checkpoint = {
            "model": model_51,
            "scaler": scaler_51,
            "features": FEATURE_NAMES_51,
            "train_rmse": rmse_train_51,
            "test_rmse": rmse_test_51,
            "train_r2": r2_train_51,
            "test_r2": r2_test_51,
            "n_train_samples": len(y_train),
            "n_test_samples": len(y_test),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        joblib.dump(checkpoint, checkpoint_path)
        print(f"[+] Successfully saved updated checkpoint to {checkpoint_path}")
    else:
        print("\n[*] Baseline 39-feature model was superior or equivalent. Keeping baseline checkpoint.")


if __name__ == "__main__":
    run_experiment()
