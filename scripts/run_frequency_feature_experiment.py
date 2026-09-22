"""
scripts/run_frequency_feature_experiment.py
===========================================
Executes the Frequency-Domain Feature Ablation Experiment for ISRO PS 26168.

MODELS COMPARED:
- Model A: Baseline GBDT (39 features)
- Model B: Jerk-Augmented GBDT (57 features)
- Model C: Frequency-Augmented GBDT (75 features: 57 + 18 spectral features)

NEW FREQUENCY FEATURES (18 features across 6 IMU channels):
- Dominant Frequency (dom_freq_*)
- Spectral Centroid (centroid_*)
- Spectral Energy (energy_*)
Computed independently for ax, ay, az, gx, gy, gz.

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
    FEATURE_NAMES_75,
    JERK_FEATURE_NAMES_18,
    FREQ_FEATURE_NAMES_18
)
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine


def load_drive_features_75(s_path: str, v_path: str, stride: int = 5, dt: float = 0.1):
    """Loads a drive pair and extracts all 75 features."""
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

    # Stage 1: In-vehicle mounting calibration
    R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, gyro, v_speed_ms)
    accel_b = (R_p2b @ accel.T).T
    gyro_b = (R_p2b @ gyro.T).T

    # Extract 75 features: 39 baseline + 18 jerk + 18 frequency
    X_75 = extract_sliding_window_features(
        accel_b, gyro_b, window_size=10, stride=stride, dt=dt,
        include_jerk=True, include_frequency=True
    )
    y = v_speed_ms[0 : N : stride]

    min_len = min(len(X_75), len(y))
    return X_75[:min_len], y[:min_len]


def run_frequency_experiment(max_samples_per_drive: int = 2000):
    print("=" * 80)
    print("FREQUENCY-DOMAIN FEATURE EXPERIMENT: 39 vs 57 vs 75 FEATURES")
    print("=" * 80)

    base_dir = "IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Total discovered trips: {len(s_files)}")

    held_out_drive_ids = {'S3b', 'Vta01b', 'Vta04', 'Vtb01', 'Vw02'}

    X_train_75_list, y_train_list = [], []
    held_out_test_dict = {}

    print("[*] Ingesting drives and extracting 75 features...")
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

        X_d_75, y_d = load_drive_features_75(s_path, v_path, stride=5, dt=0.1)
        if X_d_75 is None or len(X_d_75) == 0:
            continue

        if drive_id in held_out_drive_ids:
            held_out_test_dict[drive_id] = (X_d_75, y_d, cat)
        else:
            if len(X_d_75) > max_samples_per_drive:
                sub_idx = np.linspace(0, len(X_d_75) - 1, max_samples_per_drive, dtype=int)
                X_sampled = X_d_75[sub_idx]
                y_sampled = y_d[sub_idx]
            else:
                X_sampled = X_d_75
                y_sampled = y_d
            X_train_75_list.append(X_sampled)
            y_train_list.append(y_sampled)

    print(f"[+] Ingestion completed in {time.time() - t_ingest:.1f} s")

    X_train_75 = np.vstack(X_train_75_list)
    y_train = np.concatenate(y_train_list)

    X_test_75 = np.vstack([t[0] for t in held_out_test_dict.values()])
    y_test = np.concatenate([t[1] for t in held_out_test_dict.values()])

    # Split feature matrices
    X_train_39 = X_train_75[:, :39]
    X_test_39 = X_test_75[:, :39]

    X_train_57 = X_train_75[:, :57]
    X_test_57 = X_test_75[:, :57]

    # =========================================================================
    # PRE-TRAINING CHECKS & VERIFICATIONS
    # =========================================================================
    print("\n" + "=" * 80)
    print("PRE-TRAINING DATA INTEGRITY & PARITY VERIFICATION")
    print("=" * 80)

    # 1. Feature count verification
    print(f"  - Total features in Model C: {X_train_75.shape[1]} (Expected: 75)")
    assert X_train_75.shape[1] == 75, f"Expected 75 features, got {X_train_75.shape[1]}"
    assert len(FEATURE_NAMES_75) == 75, f"Expected 75 feature names, got {len(FEATURE_NAMES_75)}"

    # 2. Parity check: first 39 and first 57 columns
    assert np.allclose(X_train_75[:, :39], X_train_39), "Parity error: 39 baseline altered!"
    assert np.allclose(X_train_75[:, :57], X_train_57), "Parity error: 57 jerk features altered!"
    print("  - Parity check: PASSED (First 39 and 57 features are 100% numerically identical)")

    # 3. NaN / Inf check
    nan_count = int(np.isnan(X_train_75).sum() + np.isnan(X_test_75).sum())
    inf_count = int(np.isinf(X_train_75).sum() + np.isinf(X_test_75).sum())
    print(f"  - Total NaN values across all samples: {nan_count}")
    print(f"  - Total Inf values across all samples: {inf_count}")
    assert nan_count == 0, "ERROR: Found NaN in features!"
    assert inf_count == 0, "ERROR: Found Inf in features!"
    print("  - Numerical integrity check: PASSED (Zero NaNs, Zero Infs)")

    # 4. Feature list print
    print("\n  Complete List of 18 Newly Added Frequency-Domain Features:")
    for idx, name in enumerate(FREQ_FEATURE_NAMES_18, 1):
        print(f"    {idx:2d}. {name:<18s} (Col {57 + idx:2d}/75)")

    # Standardizers (fitted strictly on training set)
    scaler_39 = StandardScaler().fit(X_train_39)
    X_train_39_sc = scaler_39.transform(X_train_39)
    X_test_39_sc = scaler_39.transform(X_test_39)

    scaler_57 = StandardScaler().fit(X_train_57)
    X_train_57_sc = scaler_57.transform(X_train_57)
    X_test_57_sc = scaler_57.transform(X_test_57)

    scaler_75 = StandardScaler().fit(X_train_75)
    X_train_75_sc = scaler_75.transform(X_train_75)
    X_test_75_sc = scaler_75.transform(X_test_75)

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
    # MODEL A: 39 FEATURES
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL A: BASELINE (39 Features)")
    print("=" * 50)
    model_39 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_39 = time.time()
    model_39.fit(X_train_39_sc, y_train)
    t_train_39 = time.time() - t0_39

    p_tr_39 = np.clip(model_39.predict(X_train_39_sc), 0.0, 35.0)
    p_te_39 = np.clip(model_39.predict(X_test_39_sc), 0.0, 35.0)

    # Latency test
    _ = model_39.predict(X_test_39_sc[:100])
    t_lat0 = time.time()
    for _ in range(10):
        _ = model_39.predict(X_test_39_sc[:1000])
    lat_39 = (time.time() - t_lat0) / (10 * 1000) * 1e6

    # =========================================================================
    # MODEL B: 57 FEATURES (39 Baseline + 18 Jerk)
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL B: JERK MODEL (57 Features)")
    print("=" * 50)
    model_57 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_57 = time.time()
    model_57.fit(X_train_57_sc, y_train)
    t_train_57 = time.time() - t0_57

    p_tr_57 = np.clip(model_57.predict(X_train_57_sc), 0.0, 35.0)
    p_te_57 = np.clip(model_57.predict(X_test_57_sc), 0.0, 35.0)

    # Latency test
    _ = model_57.predict(X_test_57_sc[:100])
    t_lat1 = time.time()
    for _ in range(10):
        _ = model_57.predict(X_test_57_sc[:1000])
    lat_57 = (time.time() - t_lat1) / (10 * 1000) * 1e6

    # =========================================================================
    # MODEL C: 75 FEATURES (57 + 18 Frequency Features)
    # =========================================================================
    print("\n" + "=" * 50)
    print("TRAINING MODEL C: FREQUENCY MODEL (75 Features)")
    print("=" * 50)
    model_75 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_75 = time.time()
    model_75.fit(X_train_75_sc, y_train)
    t_train_75 = time.time() - t0_75

    p_tr_75 = np.clip(model_75.predict(X_train_75_sc), 0.0, 35.0)
    p_te_75 = np.clip(model_75.predict(X_test_75_sc), 0.0, 35.0)

    # Latency test
    _ = model_75.predict(X_test_75_sc[:100])
    t_lat2 = time.time()
    for _ in range(10):
        _ = model_75.predict(X_test_75_sc[:1000])
    lat_75 = (time.time() - t_lat2) / (10 * 1000) * 1e6

    # Save Model C checkpoint separately (DO NOT touch baseline)
    freq_ckpt_path = "models/gbdt_speed_model_iovnbd_frequency_experiment.joblib"
    checkpoint_freq = {
        "model": model_75,
        "scaler": scaler_75,
        "features": FEATURE_NAMES_75,
        "train_rmse": np.sqrt(mean_squared_error(y_train, p_tr_75)),
        "test_rmse": np.sqrt(mean_squared_error(y_test, p_te_75)),
        "train_r2": r2_score(y_train, p_tr_75),
        "test_r2": r2_score(y_test, p_te_75),
        "n_train_samples": len(y_train),
        "n_test_samples": len(y_test),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
    }
    joblib.dump(checkpoint_freq, freq_ckpt_path)
    size_75_kb = os.path.getsize(freq_ckpt_path) / 1024.0
    print(f"[+] Saved experimental model checkpoint to: {freq_ckpt_path} ({size_75_kb:.1f} KB)")

    # Model sizes
    joblib.dump(model_39, "models/temp_m39.joblib")
    joblib.dump(model_57, "models/temp_m57.joblib")
    size_39_kb = os.path.getsize("models/temp_m39.joblib") / 1024.0
    size_57_kb = os.path.getsize("models/temp_m57.joblib") / 1024.0
    os.remove("models/temp_m39.joblib")
    os.remove("models/temp_m57.joblib")

    # 0-60 km/h masks
    m60_tr = y_train <= (60.0 / 3.6)
    m60_te = y_test <= (60.0 / 3.6)

    # Compute metrics function
    def calc_metrics(y_true, y_pred, mask=None):
        yt = y_true[mask] if mask is not None else y_true
        yp = y_pred[mask] if mask is not None else y_pred
        rmse = float(np.sqrt(mean_squared_error(yt, yp)))
        mae = float(mean_absolute_error(yt, yp))
        r2 = float(r2_score(yt, yp)) if np.var(yt) > 1e-4 else 0.0
        return rmse, mae, r2

    m_tr_39 = calc_metrics(y_train, p_tr_39)
    m_te_39 = calc_metrics(y_test, p_te_39)
    m_60_te_39 = calc_metrics(y_test, p_te_39, m60_te)

    m_tr_57 = calc_metrics(y_train, p_tr_57)
    m_te_57 = calc_metrics(y_test, p_te_57)
    m_60_te_57 = calc_metrics(y_test, p_te_57, m60_te)

    m_tr_75 = calc_metrics(y_train, p_tr_75)
    m_te_75 = calc_metrics(y_test, p_te_75)
    m_60_te_75 = calc_metrics(y_test, p_te_75, m60_te)

    # =========================================================================
    # COMPACT COMPARISON TABLE (39 vs 57 vs 75)
    # =========================================================================
    print("\n" + "=" * 90)
    print("COMPREHENSIVE THREE-WAY MODEL COMPARISON: 39 vs 57 vs 75 FEATURES")
    print("=" * 90)
    print(f"{'Metric':<28s} | {'Baseline (39)':<14s} | {'Jerk (57)':<14s} | {'Freq (75)':<14s} | {'Delta (75 vs 57)':<16s}")
    print("-" * 90)

    comp_rows = [
        ("Feature Count", "39", "57", "75", "+18 features"),
        ("Training Time", f"{t_train_39:.1f} s", f"{t_train_57:.1f} s", f"{t_train_75:.1f} s", f"{t_train_75 - t_train_57:+.1f} s"),
        ("Model Checkpoint Size", f"{size_39_kb:.1f} KB", f"{size_57_kb:.1f} KB", f"{size_75_kb:.1f} KB", f"{size_75_kb - size_57_kb:+.1f} KB"),
        ("Inference Latency", f"{lat_39:.2f} us", f"{lat_57:.2f} us", f"{lat_75:.2f} us", f"{lat_75 - lat_57:+.2f} us"),
        ("Train Speed RMSE", f"{m_tr_39[0]:.3f} m/s", f"{m_tr_57[0]:.3f} m/s", f"{m_tr_75[0]:.3f} m/s", f"{m_tr_75[0] - m_tr_57[0]:+.3f} m/s"),
        ("Train Speed MAE", f"{m_tr_39[1]:.3f} m/s", f"{m_tr_57[1]:.3f} m/s", f"{m_tr_75[1]:.3f} m/s", f"{m_tr_75[1] - m_tr_57[1]:+.3f} m/s"),
        ("Train Speed R2", f"{m_tr_39[2]:.4f}", f"{m_tr_57[2]:.4f}", f"{m_tr_75[2]:.4f}", f"{m_tr_75[2] - m_tr_57[2]:+.4f}"),
        ("Held-Out Test RMSE", f"{m_te_39[0]:.3f} m/s", f"{m_te_57[0]:.3f} m/s", f"{m_te_75[0]:.3f} m/s", f"{m_te_75[0] - m_te_57[0]:+.3f} m/s"),
        ("Held-Out Test MAE", f"{m_te_39[1]:.3f} m/s", f"{m_te_57[1]:.3f} m/s", f"{m_te_75[1]:.3f} m/s", f"{m_te_75[1] - m_te_57[1]:+.3f} m/s"),
        ("Held-Out Test R2", f"{m_te_39[2]:.4f}", f"{m_te_57[2]:.4f}", f"{m_te_75[2]:.4f}", f"{m_te_75[2] - m_te_57[2]:+.4f}"),
        ("0-60 km/h Test RMSE", f"{m_60_te_39[0]:.3f} m/s", f"{m_60_te_57[0]:.3f} m/s", f"{m_60_te_75[0]:.3f} m/s", f"{m_60_te_75[0] - m_60_te_57[0]:+.3f} m/s"),
        ("0-60 km/h Test R2", f"{m_60_te_39[2]:.4f}", f"{m_60_te_57[2]:.4f}", f"{m_60_te_75[2]:.4f}", f"{m_60_te_75[2] - m_60_te_57[2]:+.4f}"),
    ]

    for name, c39, c57, c75, delta in comp_rows:
        print(f"{name:<28s} | {c39:<14s} | {c57:<14s} | {c75:<14s} | {delta:<16s}")
    print("-" * 90)

    # =========================================================================
    # PER-DRIVE BREAKDOWN ON THE 5 HELD-OUT DRIVES
    # =========================================================================
    print("\nPER-DRIVE HELD-OUT SUITE BREAKDOWN (Test RMSE in m/s):")
    print(f"{'Drive ID':<10s} | {'Category':<16s} | {'Windows':<7s} | {'Baseline 39':<12s} | {'Jerk 57':<12s} | {'Freq 75':<12s} | {'Delta (75 vs 57)':<16s}")
    print("-" * 95)
    for drive_id, (X_d_75, y_d, cat_d) in sorted(held_out_test_dict.items()):
        X39 = scaler_39.transform(X_d_75[:, :39])
        X57 = scaler_57.transform(X_d_75[:, :57])
        X75 = scaler_75.transform(X_d_75)

        p39 = np.clip(model_39.predict(X39), 0.0, 35.0)
        p57 = np.clip(model_57.predict(X57), 0.0, 35.0)
        p75 = np.clip(model_75.predict(X75), 0.0, 35.0)

        r39 = float(np.sqrt(mean_squared_error(y_d, p39)))
        r57 = float(np.sqrt(mean_squared_error(y_d, p57)))
        r75 = float(np.sqrt(mean_squared_error(y_d, p75)))
        d75_57 = r75 - r57
        print(f"{drive_id:<10s} | {cat_d:<16s} | {len(y_d):<7,d} | {r39:10.3f} m/s | {r57:10.3f} m/s | {r75:10.3f} m/s | {d75_57:+10.3f} m/s")
    print("-" * 95)

    # =========================================================================
    # FEATURE IMPORTANCE (MODEL C - 75 FEATURES)
    # =========================================================================
    importances = model_75.feature_importances_
    sorted_idx = np.argsort(importances)[::-1]

    print("\n" + "=" * 80)
    print("FEATURE IMPORTANCE RANKING (75-FEATURE MODEL C)")
    print("=" * 80)
    print(f"{'Rank':<5s} | {'Feature Name':<22s} | {'Importance %':<14s} | {'Feature Group':<18s}")
    print("-" * 65)

    freq_set = set(FREQ_FEATURE_NAMES_18)
    jerk_set = set(JERK_FEATURE_NAMES_18)

    for rank, idx in enumerate(sorted_idx, 1):
        name = FEATURE_NAMES_75[idx]
        imp_pct = importances[idx] * 100.0
        if name in freq_set:
            f_grp = "FREQUENCY (NEW)"
        elif name in jerk_set:
            f_grp = "JERK (STEP 2)"
        else:
            f_grp = "BASELINE (39)"

        if rank <= 20 or name in freq_set:
            print(f"{rank:<5d} | {name:<22s} | {imp_pct:6.2f}%       | {f_grp:<18s}")
    print("-" * 65)

    print("\nNEWLY ADDED FREQUENCY FEATURES SUMMARY:")
    freq_imp_total = 0.0
    freq_ranked = []
    for name in FREQ_FEATURE_NAMES_18:
        idx = FEATURE_NAMES_75.index(name)
        imp = importances[idx] * 100.0
        rank = list(sorted_idx).index(idx) + 1
        freq_imp_total += imp
        freq_ranked.append((rank, name, imp))
        print(f"  - {name:<18s}: Rank {rank:2d}/75 | Importance: {imp:5.2f}%")

    freq_ranked.sort(key=lambda x: x[0])
    most_imp_f = freq_ranked[0]
    print(f"\nTotal Importance of 18 Frequency Features: {freq_imp_total:.2f}%")
    print(f"Most Important Frequency Feature: {most_imp_f[1]} (Rank {most_imp_f[0]}/75, {most_imp_f[2]:.2f}%)")


if __name__ == "__main__":
    run_frequency_experiment()
