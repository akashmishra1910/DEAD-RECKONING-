"""
scripts/run_feature_selection_experiment.py
===========================================
Executes Checkpoint 3D: Feature Selection Experiment for ISRO PS 26168.
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


def load_drive_features_57(s_path: str, v_path: str, stride: int = 5, dt: float = 0.1):
    """Loads a drive pair and extracts all 57 features (39 baseline + 18 jerk)."""
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

    # Stage 1: In-vehicle mounting calibration (PCA leveling)
    R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, gyro, v_speed_ms)
    accel_b = (R_p2b @ accel.T).T
    gyro_b = (R_p2b @ gyro.T).T

    # Extract 57 features (39 baseline + 18 jerk)
    X_57 = extract_sliding_window_features(
        accel_b, gyro_b, window_size=10, stride=stride, dt=dt,
        include_jerk=True, include_frequency=False
    )
    y = v_speed_ms[0 : N : stride]

    min_len = min(len(X_57), len(y))
    return X_57[:min_len], y[:min_len]


def calc_metrics(y_true, y_pred, mask=None):
    yt = y_true[mask] if mask is not None else y_true
    yp = y_pred[mask] if mask is not None else y_pred
    rmse = float(np.sqrt(mean_squared_error(yt, yp)))
    mae = float(mean_absolute_error(yt, yp))
    r2 = float(r2_score(yt, yp)) if np.var(yt) > 1e-4 else 0.0
    return rmse, mae, r2


def measure_inference_latency(model, X_sample, n_warmup=100, n_iter=10, batch_size=1000):
    _ = model.predict(X_sample[:n_warmup])
    t0 = time.time()
    for _ in range(n_iter):
        _ = model.predict(X_sample[:batch_size])
    return (time.time() - t0) / (n_iter * batch_size) * 1e6


def run_feature_selection_experiment(max_samples_per_drive: int = 2000):
    print("=" * 85)
    print("CHECKPOINT 3D: FEATURE SELECTION EXPERIMENT (57 vs TOP 45 vs TOP 35 vs TOP 25)")
    print("=" * 85)

    base_dir = "IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Total discovered trips: {len(s_files)}")

    held_out_drive_ids = {'S3b', 'Vta01b', 'Vta04', 'Vtb01', 'Vw02'}

    X_train_57_list, y_train_list = [], []
    held_out_test_dict = {}

    print("[*] Ingesting drives and extracting 57 features (10 Hz, 1.0s window, stride 5)...")
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

        X_d_57, y_d = load_drive_features_57(s_path, v_path, stride=5, dt=0.1)
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

    print(f"[*] Training samples: {len(y_train):,d} across 67 drives")
    print(f"[*] Held-out test samples: {len(y_test):,d} across 5 drives ({', '.join(sorted(held_out_drive_ids))})")

    # Pre-training integrity checks
    print("\n" + "=" * 85)
    print("DATA INTEGRITY VERIFICATION")
    print("=" * 85)
    assert X_train_57.shape[1] == 57, f"Expected 57 features, got {X_train_57.shape[1]}"
    assert X_test_57.shape[1] == 57, f"Expected 57 features, got {X_test_57.shape[1]}"
    assert len(FEATURE_NAMES_57) == 57, f"Expected 57 feature names, got {len(FEATURE_NAMES_57)}"

    nan_count = int(np.isnan(X_train_57).sum() + np.isnan(X_test_57).sum())
    inf_count = int(np.isinf(X_train_57).sum() + np.isinf(X_test_57).sum())
    print(f"  - Total NaN values across all samples: {nan_count}")
    print(f"  - Total Inf values across all samples: {inf_count}")
    assert nan_count == 0, "ERROR: Found NaN in features!"
    assert inf_count == 0, "ERROR: Found Inf in features!"
    print("  - Numerical integrity check: PASSED (Zero NaNs, Zero Infs)")

    # Anti-leakage rule: StandardScalers fitted strictly on training data
    scaler_57 = StandardScaler().fit(X_train_57)
    X_train_57_sc = scaler_57.transform(X_train_57)
    X_test_57_sc = scaler_57.transform(X_test_57)

    gbdt_kwargs = dict(
        loss='huber',
        alpha=0.90,
        n_estimators=120,
        max_depth=5,
        learning_rate=0.08,
        subsample=0.85,
        random_state=42
    )

    # 1. Train 57-feature model on training set
    print("\n" + "=" * 85)
    print("1. TRAINING FULL 57-FEATURE MODEL (REFERENCE)")
    print("=" * 85)
    model_57 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_57 = time.time()
    model_57.fit(X_train_57_sc, y_train)
    t_train_57 = time.time() - t0_57
    print(f"[+] Trained 57-feature GBDT in {t_train_57:.1f} s")

    # 2. Feature ranking computed exclusively on training data
    print("\n" + "=" * 85)
    print("2. FEATURE RANKING COMPUTED STRICTLY ON TRAINING DATA (ANTI-LEAKAGE)")
    print("=" * 85)
    importances = model_57.feature_importances_
    sorted_indices = np.argsort(importances)[::-1]

    ranked_features = [FEATURE_NAMES_57[i] for i in sorted_indices]
    ranked_importances = [importances[i] for i in sorted_indices]

    print(f"{'Rank':<5s} | {'Feature Name':<22s} | {'Importance %':<14s} | {'Cumulative %':<14s} | {'Category':<15s}")
    print("-" * 80)
    cum_imp = 0.0
    jerk_set = set(JERK_FEATURE_NAMES_18)
    for rank, (name, imp) in enumerate(zip(ranked_features, ranked_importances), 1):
        cum_imp += imp * 100.0
        cat = "JERK (18)" if name in jerk_set else "BASELINE (39)"
        print(f"{rank:<5d} | {name:<22s} | {imp*100:6.3f}%       | {cum_imp:6.2f}%       | {cat:<15s}")
    print("-" * 80)

    # Subsets strictly from the training-only ranking
    top45_indices = sorted_indices[:45]
    top35_indices = sorted_indices[:35]
    top25_indices = sorted_indices[:25]

    top45_features = [FEATURE_NAMES_57[i] for i in top45_indices]
    top35_features = [FEATURE_NAMES_57[i] for i in top35_indices]
    top25_features = [FEATURE_NAMES_57[i] for i in top25_indices]

    print(f"\n[+] Subset definitions:")
    print(f"    - Full 57: 57 features (100.00% training importance)")
    print(f"    - Top 45: 45 features ({sum(ranked_importances[:45])*100:.2f}% training importance)")
    print(f"    - Top 35: 35 features ({sum(ranked_importances[:35])*100:.2f}% training importance)")
    print(f"    - Top 25: 25 features ({sum(ranked_importances[:25])*100:.2f}% training importance)")

    # 3. Train subset models (Top 45, Top 35, Top 25)
    X_tr_45_raw = X_train_57[:, top45_indices]
    X_te_45_raw = X_test_57[:, top45_indices]

    X_tr_35_raw = X_train_57[:, top35_indices]
    X_te_35_raw = X_test_57[:, top35_indices]

    X_tr_25_raw = X_train_57[:, top25_indices]
    X_te_25_raw = X_test_57[:, top25_indices]

    # StandardScalers fitted strictly on training subsets
    scaler_45 = StandardScaler().fit(X_tr_45_raw)
    X_tr_45_sc = scaler_45.transform(X_tr_45_raw)
    X_te_45_sc = scaler_45.transform(X_te_45_raw)

    scaler_35 = StandardScaler().fit(X_tr_35_raw)
    X_tr_35_sc = scaler_35.transform(X_tr_35_raw)
    X_te_35_sc = scaler_35.transform(X_te_35_raw)

    scaler_25 = StandardScaler().fit(X_tr_25_raw)
    X_tr_25_sc = scaler_25.transform(X_tr_25_raw)
    X_te_25_sc = scaler_25.transform(X_te_25_raw)

    # Train Top 45
    print("\n" + "=" * 50)
    print("TRAINING TOP 45 MODEL")
    print("=" * 50)
    model_45 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_45 = time.time()
    model_45.fit(X_tr_45_sc, y_train)
    t_train_45 = time.time() - t0_45
    print(f"[+] Trained Top 45 GBDT in {t_train_45:.1f} s")

    # Train Top 35
    print("\n" + "=" * 50)
    print("TRAINING TOP 35 MODEL")
    print("=" * 50)
    model_35 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_35 = time.time()
    model_35.fit(X_tr_35_sc, y_train)
    t_train_35 = time.time() - t0_35
    print(f"[+] Trained Top 35 GBDT in {t_train_35:.1f} s")

    # Train Top 25
    print("\n" + "=" * 50)
    print("TRAINING TOP 25 MODEL")
    print("=" * 50)
    model_25 = GradientBoostingRegressor(**gbdt_kwargs)
    t0_25 = time.time()
    model_25.fit(X_tr_25_sc, y_train)
    t_train_25 = time.time() - t0_25
    print(f"[+] Trained Top 25 GBDT in {t_train_25:.1f} s")

    # 4. Predictions & Evaluations
    models = {
        "57 Features": (model_57, scaler_57, sorted_indices, FEATURE_NAMES_57, t_train_57, X_train_57_sc, X_test_57_sc),
        "Top 45": (model_45, scaler_45, top45_indices, top45_features, t_train_45, X_tr_45_sc, X_te_45_sc),
        "Top 35": (model_35, scaler_35, top35_indices, top35_features, t_train_35, X_tr_35_sc, X_te_35_sc),
        "Top 25": (model_25, scaler_25, top25_indices, top25_features, t_train_25, X_tr_25_sc, X_te_25_sc),
    }

    m60_te = y_test <= (60.0 / 3.6)
    m60_tr = y_train <= (60.0 / 3.6)

    ckpt_paths = {
        "Top 45": "models/gbdt_speed_model_iovnbd_top45.joblib",
        "Top 35": "models/gbdt_speed_model_iovnbd_top35.joblib",
        "Top 25": "models/gbdt_speed_model_iovnbd_top25.joblib",
    }

    results = {}
    print("\n" + "=" * 85)
    print("EVALUATION & BENCHMARKING ACROSS ALL CANDIDATES")
    print("=" * 85)

    for name, (m, sc, feat_idx, feat_names, t_train, X_tr_sc, X_te_sc) in models.items():
        p_tr = np.clip(m.predict(X_tr_sc), 0.0, 35.0)
        p_te = np.clip(m.predict(X_te_sc), 0.0, 35.0)

        tr_rmse, tr_mae, tr_r2 = calc_metrics(y_train, p_tr)
        te_rmse, te_mae, te_r2 = calc_metrics(y_test, p_te)
        te_60_rmse, te_60_mae, te_60_r2 = calc_metrics(y_test, p_te, m60_te)

        lat = measure_inference_latency(m, X_te_sc)

        if name in ckpt_paths:
            ckpt_p = ckpt_paths[name]
            ckpt_obj = {
                "model": m,
                "scaler": sc,
                "features": feat_names,
                "indices": [int(idx) for idx in feat_idx],
                "train_rmse": tr_rmse,
                "test_rmse": te_rmse,
                "train_r2": tr_r2,
                "test_r2": te_r2,
                "n_train_samples": len(y_train),
                "n_test_samples": len(y_test),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            joblib.dump(ckpt_obj, ckpt_p)
            size_kb = os.path.getsize(ckpt_p) / 1024.0
            print(f"[+] Saved checkpoint for {name} -> {ckpt_p} ({size_kb:.1f} KB)")
        else:
            joblib.dump(m, "models/temp_eval.joblib")
            size_kb = os.path.getsize("models/temp_eval.joblib") / 1024.0
            os.remove("models/temp_eval.joblib")

        # Per-drive test metrics
        drive_rmses = {}
        for drive_id, (X_d_57, y_d, cat_d) in sorted(held_out_test_dict.items()):
            if name == "57 Features":
                X_d_sub = X_d_57
            elif name == "Top 45":
                X_d_sub = X_d_57[:, top45_indices]
            elif name == "Top 35":
                X_d_sub = X_d_57[:, top35_indices]
            elif name == "Top 25":
                X_d_sub = X_d_57[:, top25_indices]

            X_d_sc = sc.transform(X_d_sub)
            p_d = np.clip(m.predict(X_d_sc), 0.0, 35.0)
            drive_rmses[drive_id] = float(np.sqrt(mean_squared_error(y_d, p_d)))

        results[name] = {
            "n_features": len(feat_names),
            "t_train": t_train,
            "size_kb": size_kb,
            "lat": lat,
            "tr_rmse": tr_rmse,
            "tr_mae": tr_mae,
            "tr_r2": tr_r2,
            "te_rmse": te_rmse,
            "te_mae": te_mae,
            "te_r2": te_r2,
            "te_60_rmse": te_60_rmse,
            "te_60_r2": te_60_r2,
            "drive_rmses": drive_rmses,
            "features": feat_names
        }

    # Summary Report Table
    print("\n" + "=" * 105)
    print("FEATURE SELECTION ABLATION RESULTS: 57 vs TOP 45 vs TOP 35 vs TOP 25")
    print("=" * 105)
    hdr = f"{'Metric':<25s} | {'57 Features':<16s} | {'Top 45':<16s} | {'Top 35':<16s} | {'Top 25':<16s}"
    print(hdr)
    print("-" * 105)

    r57 = results["57 Features"]
    r45 = results["Top 45"]
    r35 = results["Top 35"]
    r25 = results["Top 25"]

    rows = [
        ("Feature Count", f"{r57['n_features']}", f"{r45['n_features']}", f"{r35['n_features']}", f"{r25['n_features']}"),
        ("Training Time", f"{r57['t_train']:.1f} s", f"{r45['t_train']:.1f} s", f"{r35['t_train']:.1f} s", f"{r25['t_train']:.1f} s"),
        ("Model Checkpoint Size", f"{r57['size_kb']:.1f} KB", f"{r45['size_kb']:.1f} KB", f"{r35['size_kb']:.1f} KB", f"{r25['size_kb']:.1f} KB"),
        ("Inference Latency", f"{r57['lat']:.2f} us", f"{r45['lat']:.2f} us", f"{r35['lat']:.2f} us", f"{r25['lat']:.2f} us"),
        ("Train Speed RMSE", f"{r57['tr_rmse']:.3f} m/s", f"{r45['tr_rmse']:.3f} m/s", f"{r35['tr_rmse']:.3f} m/s", f"{r25['tr_rmse']:.3f} m/s"),
        ("Train Speed MAE", f"{r57['tr_mae']:.3f} m/s", f"{r45['tr_mae']:.3f} m/s", f"{r35['tr_mae']:.3f} m/s", f"{r25['tr_mae']:.3f} m/s"),
        ("Train Speed R2", f"{r57['tr_r2']:.4f}", f"{r45['tr_r2']:.4f}", f"{r35['tr_r2']:.4f}", f"{r25['tr_r2']:.4f}"),
        ("Held-Out Test RMSE", f"{r57['te_rmse']:.3f} m/s", f"{r45['te_rmse']:.3f} m/s", f"{r35['te_rmse']:.3f} m/s", f"{r25['te_rmse']:.3f} m/s"),
        ("Held-Out Test MAE", f"{r57['te_mae']:.3f} m/s", f"{r45['te_mae']:.3f} m/s", f"{r35['te_mae']:.3f} m/s", f"{r25['te_mae']:.3f} m/s"),
        ("Held-Out Test R2", f"{r57['te_r2']:.4f}", f"{r45['te_r2']:.4f}", f"{r35['te_r2']:.4f}", f"{r25['te_r2']:.4f}"),
        ("0-60 km/h Test RMSE", f"{r57['te_60_rmse']:.3f} m/s", f"{r45['te_60_rmse']:.3f} m/s", f"{r35['te_60_rmse']:.3f} m/s", f"{r25['te_60_rmse']:.3f} m/s"),
        ("0-60 km/h Test R2", f"{r57['te_60_r2']:.4f}", f"{r45['te_60_r2']:.4f}", f"{r35['te_60_r2']:.4f}", f"{r25['te_60_r2']:.4f}"),
    ]
    for row in rows:
        print(f"{row[0]:<25s} | {row[1]:<16s} | {row[2]:<16s} | {row[3]:<16s} | {row[4]:<16s}")
    print("-" * 105)

    print("\nPER-DRIVE HELD-OUT SUITE BREAKDOWN (Test RMSE in m/s):")
    print(f"{'Drive ID':<10s} | {'57 Features':<16s} | {'Top 45':<16s} | {'Top 35':<16s} | {'Top 25':<16s}")
    print("-" * 85)
    for d in sorted(held_out_drive_ids):
        print(f"{d:<10s} | {r57['drive_rmses'][d]:12.3f} m/s | {r45['drive_rmses'][d]:12.3f} m/s | {r35['drive_rmses'][d]:12.3f} m/s | {r25['drive_rmses'][d]:12.3f} m/s")
    print("-" * 85)

    print("\n" + "=" * 85)
    print("EXACT SELECTED FEATURE NAMES PER SUBSET")
    print("=" * 85)
    print(f"\n[Top 25 Features] (Retained {sum(ranked_importances[:25])*100:.2f}% training importance):")
    for i, fn in enumerate(top25_features, 1):
        print(f"  {i:2d}. {fn}")

    print(f"\n[Features in Top 35 but not Top 25] (+10 features):")
    for i, fn in enumerate(top35_features[25:], 26):
        print(f"  {i:2d}. {fn}")

    print(f"\n[Features in Top 45 but not Top 35] (+10 features):")
    for i, fn in enumerate(top45_features[35:], 36):
        print(f"  {i:2d}. {fn}")

    print(f"\n[Features in 57 but pruned from Top 45] (12 pruned features, {sum(ranked_importances[45:])*100:.2f}% training importance):")
    for i, fn in enumerate(ranked_features[45:], 46):
        print(f"  {i:2d}. {fn} ({ranked_importances[i-1]*100:.3f}%)")


if __name__ == "__main__":
    run_feature_selection_experiment()
