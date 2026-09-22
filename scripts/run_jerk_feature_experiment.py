"""
scripts/run_jerk_feature_experiment.py
======================================
Executes Step 2 of the GBDT Feature Ablation Experiment:
Evaluating temporal derivative (jerk and angular rate derivative) features.

MODELS:
- Model A: Baseline GBDT (39 features)
- Model B: Jerk-Augmented GBDT (57 features: 39 baseline + 18 jerk/rate derivative)

EXPERIMENTAL RIGOR:
- Dataset: Authentic IO-VNBD (72 synchronized trips)
- Train/Test Split: 67 training drives vs 5 held-out test drives (S3b, Vta01b, Vta04, Vtb01, Vw02)
- Window: 1.0s (10 samples @ 10 Hz), stride=5 (0.5s step)
- Preprocessing: Stage 1 Mounting Calibration (PCA) + StandardScaler (fitted strictly on train)
- Hyperparameters: Huber loss (alpha=0.90), n_estimators=120, max_depth=5, lr=0.08, subsample=0.85, random_state=42
- Supervised Target: Vehicle reference forward speed (m/s)
- Feature Source: Smartphone IMU only (no GPS, no OBD speed input, no map, no ESKF)
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
    FEATURE_NAMES_57,
    JERK_FEATURE_NAMES_18
)
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine


def load_drive_features(s_path: str, v_path: str, stride: int = 5, dt: float = 0.1):
    """Loads a drive pair and extracts 57 features (first 39 are identical to baseline)."""
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

    # Stage 1: In-vehicle mounting calibration (same for both models)
    R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, gyro, v_speed_ms)
    accel_b = (R_p2b @ accel.T).T
    gyro_b = (R_p2b @ gyro.T).T

    # Extract 57 features (first 39 identical to baseline, next 18 are jerk/rate derivative)
    X_57 = extract_sliding_window_features(
        accel_b, gyro_b, window_size=10, stride=stride, dt=dt, include_jerk=True
    )
    y = v_speed_ms[0 : N : stride]

    min_len = min(len(X_57), len(y))
    return X_57[:min_len], y[:min_len]


def run_jerk_experiment(max_samples_per_drive: int = 2000):
    print("=" * 80)
    print("STEP 2 EXPERIMENT: EVALUATING JERK / TEMPORAL-DIFFERENCE FEATURES")
    print("=" * 80)

    base_dir = "IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Total discovered trips: {len(s_files)}")

    held_out_drive_ids = {'S3b', 'Vta01b', 'Vta04', 'Vtb01', 'Vw02'}

    X_train_57_list, y_train_list = [], []
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

        X_d_57, y_d = load_drive_features(s_path, v_path, stride=5, dt=0.1)
        if X_d_57 is None or len(X_d_57) == 0:
            continue

        if drive_id in held_out_drive_ids:
            held_out_test_dict[drive_id] = (X_d_57, y_d, cat)
        else:
            if len(X_d_57) > max_samples_per_drive:
                sub_idx = np.linspace(0, len(X_d_57) - 1, max_samples_per_drive, dtype=int)
                X_sampled = X_d_57[sub_idx]
                y_sampled = y_d[sub_idx]
            else:
                X_sampled = X_d_57
                y_sampled = y_d
            X_train_57_list.append(X_sampled)
            y_train_list.append(y_sampled)

    print(f"[+] Ingestion completed in {time.time() - t_ingest:.1f} s")

    X_train_57 = np.vstack(X_train_57_list)
    y_train = np.concatenate(y_train_list)

    X_test_57 = np.vstack([t[0] for t in held_out_test_dict.values()])
    y_test = np.concatenate([t[1] for t in held_out_test_dict.values()])

    # Split into 39 vs 57 feature representations
    X_train_39 = X_train_57[:, :39]
    X_test_39 = X_test_57[:, :39]

    # Verify parity: first 39 features
    print(f"[*] Training Windows: {len(y_train):,d} | Test Windows: {len(y_test):,d}")
    print(f"[*] Model A Feature Dim: {X_train_39.shape[1]} | Model B Feature Dim: {X_train_57.shape[1]}")

    # =========================================================================
    # STEP 8: NUMERICAL STABILITY CHECK
    # =========================================================================
    print("\n" + "=" * 80)
    print("STEP 8: NUMERICAL INTEGRITY & STABILITY AUDIT OF JERK FEATURES")
    print("=" * 80)
    all_X_57 = np.vstack([X_train_57, X_test_57])
    nan_count = int(np.isnan(all_X_57).sum())
    inf_count = int(np.isinf(all_X_57).sum())
    print(f"  - Total NaN values across all samples/features: {nan_count}")
    print(f"  - Total Inf values across all samples/features: {inf_count}")
    assert nan_count == 0, "ERROR: Found NaN in features!"
    assert inf_count == 0, "ERROR: Found Inf in features!"

    print("\n  Summary Statistics for Newly Added 18 Jerk Features (across 73,581 windows):")
    print(f"  {'Feature':<20s} | {'Min':>10s} | {'Max':>10s} | {'Mean':>10s} | {'Std':>10s} | {'p1':>10s} | {'p99':>10s}")
    print("  " + "-" * 88)
    jerk_stats = {}
    for i, name in enumerate(JERK_FEATURE_NAMES_18):
        col_idx = 39 + i
        col_data = all_X_57[:, col_idx]
        c_min = float(np.min(col_data))
        c_max = float(np.max(col_data))
        c_mean = float(np.mean(col_data))
        c_std = float(np.std(col_data))
        p1 = float(np.percentile(col_data, 1))
        p99 = float(np.percentile(col_data, 99))
        jerk_stats[name] = dict(min=c_min, max=c_max, mean=c_mean, std=c_std, p1=p1, p99=p99)
        print(f"  {name:<20s} | {c_min:10.3f} | {c_max:10.3f} | {c_mean:10.3f} | {c_std:10.3f} | {p1:10.3f} | {p99:10.3f}")
    print("  " + "-" * 88)

    # Standardizers
    scaler_39 = StandardScaler()
    X_train_39_scaled = scaler_39.fit_transform(X_train_39)
    X_test_39_scaled = scaler_39.transform(X_test_39)

    scaler_57 = StandardScaler()
    X_train_57_scaled = scaler_57.fit_transform(X_train_57)
    X_test_57_scaled = scaler_57.transform(X_test_57)

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
    # MODEL A: BASELINE 39 FEATURES
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL A: BASELINE (39 Features)")
    print("=" * 50)
    model_39 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_39 = time.time()
    model_39.fit(X_train_39_scaled, y_train)
    train_time_39 = time.time() - t0_39
    print(f"[+] Model A (39 feats) trained in {train_time_39:.2f} s")

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

    joblib.dump(model_39, "models/temp_model_39.joblib")
    size_39_kb = os.path.getsize("models/temp_model_39.joblib") / 1024.0

    # =========================================================================
    # MODEL B: AUGMENTED 57 FEATURES (39 Baseline + 18 Jerk)
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL B: JERK-AUGMENTED (57 Features)")
    print("=" * 50)
    model_57 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_57 = time.time()
    model_57.fit(X_train_57_scaled, y_train)
    train_time_57 = time.time() - t0_57
    print(f"[+] Model B (57 feats) trained in {train_time_57:.2f} s")

    # Benchmarking inference latency
    warmup = model_57.predict(X_test_57_scaled[:100])
    t_lat1 = time.time()
    for _ in range(10):
        _ = model_57.predict(X_test_57_scaled[:1000])
    lat_57_us = (time.time() - t_lat1) / (10 * 1000) * 1e6

    # Predictions
    pred_train_57 = np.clip(model_57.predict(X_train_57_scaled), 0.0, 35.0)
    pred_test_57 = np.clip(model_57.predict(X_test_57_scaled), 0.0, 35.0)

    rmse_train_57 = np.sqrt(mean_squared_error(y_train, pred_train_57))
    mae_train_57 = mean_absolute_error(y_train, pred_train_57)
    r2_train_57 = r2_score(y_train, pred_train_57)

    rmse_test_57 = np.sqrt(mean_squared_error(y_test, pred_test_57))
    mae_test_57 = mean_absolute_error(y_test, pred_test_57)
    r2_test_57 = r2_score(y_test, pred_test_57)

    rmse_60_train_57 = np.sqrt(mean_squared_error(y_train[mask_60_train], pred_train_57[mask_60_train]))
    r2_60_train_57 = r2_score(y_train[mask_60_train], pred_train_57[mask_60_train])
    rmse_60_test_57 = np.sqrt(mean_squared_error(y_test[mask_60_test], pred_test_57[mask_60_test]))
    r2_60_test_57 = r2_score(y_test[mask_60_test], pred_test_57[mask_60_test])

    # Save experimental model checkpoint strictly under separate filename
    jerk_ckpt_path = "models/gbdt_speed_model_iovnbd_jerk_experiment.joblib"
    checkpoint_jerk = {
        "model": model_57,
        "scaler": scaler_57,
        "features": FEATURE_NAMES_57,
        "train_rmse": rmse_train_57,
        "test_rmse": rmse_test_57,
        "train_r2": r2_train_57,
        "test_r2": r2_test_57,
        "n_train_samples": len(y_train),
        "n_test_samples": len(y_test),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    joblib.dump(checkpoint_jerk, jerk_ckpt_path)
    size_57_kb = os.path.getsize(jerk_ckpt_path) / 1024.0
    print(f"[+] Saved experimental model checkpoint to: {jerk_ckpt_path} ({size_57_kb:.1f} KB)")

    if os.path.exists("models/temp_model_39.joblib"):
        os.remove("models/temp_model_39.joblib")

    # =========================================================================
    # COMPARISON TABLE
    # =========================================================================
    print("\n" + "=" * 80)
    print("MODEL COMPARISON RESULTS: BASELINE (39 Feats) vs JERK MODEL (57 Feats)")
    print("=" * 80)
    print(f"{'Metric':<35s} | {'Baseline 39':<20s} | {'Jerk Model (57)':<20s} | {'Delta':<15s}")
    print("-" * 80)

    rows = [
        ("Feature Count", f"{39}", f"{57}", f"+18 features"),
        ("Training Time", f"{train_time_39:.2f} s", f"{train_time_57:.2f} s", f"{train_time_57 - train_time_39:+.2f} s"),
        ("Model Size (joblib)", f"{size_39_kb:.1f} KB", f"{size_57_kb:.1f} KB", f"{size_57_kb - size_39_kb:+.1f} KB"),
        ("Inference Latency", f"{lat_39_us:.2f} us/sample", f"{lat_57_us:.2f} us/sample", f"{lat_57_us - lat_39_us:+.2f} us"),
        ("Train Speed RMSE", f"{rmse_train_39:.3f} m/s", f"{rmse_train_57:.3f} m/s", f"{rmse_train_57 - rmse_train_39:+.3f} m/s"),
        ("Train Speed MAE", f"{mae_train_39:.3f} m/s", f"{mae_train_57:.3f} m/s", f"{mae_train_57 - mae_train_39:+.3f} m/s"),
        ("Train Speed R2", f"{r2_train_39:.4f}", f"{r2_train_57:.4f}", f"{r2_train_57 - r2_train_39:+.4f}"),
        ("Held-Out Test RMSE (All)", f"{rmse_test_39:.3f} m/s", f"{rmse_test_57:.3f} m/s", f"{rmse_test_57 - rmse_test_39:+.3f} m/s"),
        ("Held-Out Test MAE (All)", f"{mae_test_39:.3f} m/s", f"{mae_test_57:.3f} m/s", f"{mae_test_57 - mae_test_39:+.3f} m/s"),
        ("Held-Out Test R2 (All)", f"{r2_test_39:.4f}", f"{r2_test_57:.4f}", f"{r2_test_57 - r2_test_39:+.4f}"),
        ("0-60 km/h Test RMSE", f"{rmse_60_test_39:.3f} m/s", f"{rmse_60_test_57:.3f} m/s", f"{rmse_60_test_57 - rmse_60_test_39:+.3f} m/s"),
        ("0-60 km/h Test R2", f"{r2_60_test_39:.4f}", f"{r2_60_test_57:.4f}", f"{r2_60_test_57 - r2_60_test_39:+.4f}")
    ]

    for name, m39, m57, chg in rows:
        print(f"{name:<35s} | {m39:<20s} | {m57:<20s} | {chg:<15s}")
    print("-" * 80)

    # Per-Trip Breakdown Comparison
    print("\nPER-DRIVE HELD-OUT SUITE BREAKDOWN (Test RMSE in m/s):")
    print(f"{'Drive ID':<10s} | {'Category':<16s} | {'Windows':<7s} | {'Baseline RMSE':<18s} | {'Jerk RMSE':<18s} | {'Delta':<10s}")
    print("-" * 85)
    for drive_id, (X_d_57, y_d, cat_d) in sorted(held_out_test_dict.items()):
        X_d_39_sc = scaler_39.transform(X_d_57[:, :39])
        X_d_57_sc = scaler_57.transform(X_d_57)
        p39 = np.clip(model_39.predict(X_d_39_sc), 0.0, 35.0)
        p57 = np.clip(model_57.predict(X_d_57_sc), 0.0, 35.0)
        r39 = np.sqrt(mean_squared_error(y_d, p39))
        r57 = np.sqrt(mean_squared_error(y_d, p57))
        delta = r57 - r39
        print(f"{drive_id:<10s} | {cat_d:<16s} | {len(y_d):<7,d} | {r39:<18.3f} | {r57:<18.3f} | {delta:+.3f}")
    print("-" * 85)

    # Feature Importance of 57 features
    importances = model_57.feature_importances_
    sorted_idx = np.argsort(importances)[::-1]

    print("\n" + "=" * 80)
    print("FEATURE IMPORTANCE RANKING (57-FEATURE JERK MODEL)")
    print("=" * 80)
    print(f"{'Rank':<5s} | {'Feature Name':<22s} | {'Importance %':<14s} | {'Type':<18s}")
    print("-" * 65)

    jerk_feat_set = set(JERK_FEATURE_NAMES_18)
    for rank, idx in enumerate(sorted_idx, 1):
        name = FEATURE_NAMES_57[idx]
        imp_pct = importances[idx] * 100.0
        f_type = "JERK FEATURE" if name in jerk_feat_set else "Existing (39)"
        if rank <= 15 or name in jerk_feat_set:
            print(f"{rank:<5d} | {name:<22s} | {imp_pct:6.2f}%       | {f_type:<18s}")
    print("-" * 65)

    print("\nNEWLY ADDED JERK FEATURES SUMMARY:")
    jerk_imp_total = 0.0
    jerk_ranked_list = []
    for name in JERK_FEATURE_NAMES_18:
        idx = FEATURE_NAMES_57.index(name)
        imp = importances[idx] * 100.0
        rank = list(sorted_idx).index(idx) + 1
        jerk_imp_total += imp
        jerk_ranked_list.append((rank, name, imp))
        print(f"  - {name:<20s}: Rank {rank:2d}/57 | Importance: {imp:5.2f}%")

    jerk_ranked_list.sort(key=lambda x: x[0])
    most_imp = jerk_ranked_list[0]
    print(f"\nTotal Importance of 18 Jerk Features: {jerk_imp_total:.2f}%")
    print(f"Most Important Jerk Feature: {most_imp[1]} (Rank {most_imp[0]}/57, {most_imp[2]:.2f}%)")


if __name__ == "__main__":
    run_jerk_experiment()
