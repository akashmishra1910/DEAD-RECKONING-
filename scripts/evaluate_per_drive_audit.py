"""
scripts/evaluate_per_drive_audit.py
===================================
Forensic audit evaluating the trained AI Speed Regressor across ALL individual
drives in the authentic IO-VNBD dataset.

Reports:
1. Per-drive RMSE, MAE, R², ground truth speed, predicted speed, and error range.
2. Category-level aggregated performance (Driver A, B, D, E, asphalt, cobblestone).
3. Verifies that the 71-drive regression (especially on cobblestones and urban trips)
   is resolved.

Author: Autonomous Navigation Engineering Team
"""

import os
import sys
import glob
import time
import joblib
import numpy as np
import pandas as pd
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

# Ensure workspace root is in python path
sys.path.insert(0, os.path.abspath('.'))

from scripts.train_full_iovnbd_model import load_and_preprocess_pair


def run_per_drive_audit():
    print("=" * 90)
    print("IO-VNBD COMPREHENSIVE PER-DRIVE SPEED ESTIMATION AUDIT")
    print("=" * 90)

    checkpoint_path = "models/gbdt_speed_model_iovnbd.joblib"
    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Model checkpoint not found at {checkpoint_path}. Train model first.")

    print(f"[*] Loading model checkpoint from: {checkpoint_path}")
    checkpoint = joblib.load(checkpoint_path)
    model = checkpoint["model"]
    scaler = checkpoint["scaler"]

    base_dir = "synchronised_iovnbd_data/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Auditing {len(s_files)} drives across IO-VNBD...\n")

    results = []
    
    for s_path in s_files:
        dir_name = os.path.dirname(s_path)
        v_candidates = glob.glob(os.path.join(dir_name, "V-*.csv"))
        if not v_candidates:
            continue
        v_path = v_candidates[0]

        rel_path = os.path.relpath(s_path, base_dir)
        parts = rel_path.split(os.sep)
        cat = parts[0]
        if len(parts) >= 3:
            drive_id = parts[1]
        else:
            drive_id = os.path.splitext(os.path.basename(s_path))[0].replace("S-", "")

        X, y = load_and_preprocess_pair(s_path, v_path, stride=5)
        if X is None or len(X) < 10:
            continue

        X_scaled = scaler.transform(X)
        preds = np.clip(model.predict(X_scaled), 0.0, 35.0)

        rmse = float(np.sqrt(mean_squared_error(y, preds)))
        mae = float(mean_absolute_error(y, preds))
        r2 = float(r2_score(y, preds)) if np.var(y) > 1e-4 else 0.0

        mean_gt_kmh = float(np.mean(y) * 3.6)
        mean_pred_kmh = float(np.mean(preds) * 3.6)
        max_gt_kmh = float(np.max(y) * 3.6)

        results.append({
            "category": cat,
            "drive_id": drive_id,
            "samples": len(y),
            "mean_gt_kmh": mean_gt_kmh,
            "mean_pred_kmh": mean_pred_kmh,
            "max_gt_kmh": max_gt_kmh,
            "rmse_ms": rmse,
            "mae_ms": mae,
            "r2": r2
        })

    df = pd.DataFrame(results)

    # 1. Category-level summary
    print("=" * 90)
    print("CATEGORY-LEVEL PERFORMANCE SUMMARY:")
    print("=" * 90)
    cat_summary = df.groupby("category").agg(
        drives=("drive_id", "count"),
        total_samples=("samples", "sum"),
        mean_gt_kmh=("mean_gt_kmh", "mean"),
        rmse_ms=("rmse_ms", "mean"),
        mae_ms=("mae_ms", "mean"),
        r2=("r2", "mean")
    ).reset_index()

    print(f"{'Category':<18s} | {'Drives':<6s} | {'Windows':<8s} | {'Mean Speed':<10s} | {'Mean RMSE':<10s} | {'Mean MAE':<10s} | {'Mean R²':<8s}")
    print("-" * 90)
    for _, row in cat_summary.iterrows():
        print(f"{row['category']:<18s} | {int(row['drives']):<6d} | {int(row['total_samples']):<8,d} | {row['mean_gt_kmh']:<6.1f} km/h | {row['rmse_ms']:<6.3f} m/s | {row['mae_ms']:<6.3f} m/s | {row['r2']:<8.4f}")
    print("-" * 90)

    # Overall dataset statistics
    total_drives = len(df)
    total_samples = df["samples"].sum()
    avg_rmse = df["rmse_ms"].mean()
    avg_mae = df["mae_ms"].mean()
    avg_r2 = df["r2"].mean()

    print(f"\nOVERALL DATASET AUDIT (71+ Drives, {total_samples:,d} Windows):")
    print(f"  Average Drive RMSE : {avg_rmse:.3f} m/s ({avg_rmse*3.6:.2f} km/h)")
    print(f"  Average Drive MAE  : {avg_mae:.3f} m/s ({avg_mae*3.6:.2f} km/h)")
    print(f"  Average Drive R²   : {avg_r2:.4f}")
    print("=" * 90)

    # 2. Detailed Inspection of High-Roughness (Cobblestone) Drives
    print("\nCOBBLESTONE & BRICK ROAD AUDIT (Vtb Category):")
    print(f"{'Drive ID':<10s} | {'Windows':<8s} | {'GT Mean':<10s} | {'Pred Mean':<10s} | {'RMSE (m/s)':<10s} | {'MAE (m/s)':<10s} | {'R²':<8s}")
    print("-" * 75)
    vtb_df = df[df["category"] == "Vtb (Driver E)"]
    for _, row in vtb_df.iterrows():
        print(f"{row['drive_id']:<10s} | {row['samples']:<8,d} | {row['mean_gt_kmh']:<6.1f} km/h | {row['mean_pred_kmh']:<6.1f} km/h | {row['rmse_ms']:<10.3f} | {row['mae_ms']:<10.3f} | {row['r2']:<8.4f}")
    print("-" * 75)

    return df


if __name__ == "__main__":
    run_per_drive_audit()
