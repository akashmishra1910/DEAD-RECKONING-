#!/usr/bin/env python3
"""
scripts/audit_group_a_speed_model.py

Audits the baseline GBDT forward speed regressor on authentic IO-VNBD drives.
Measures MAE, RMSE, Bias, R², and P95 stratified by physical motion regimes.
"""

import os
import sys
import math
import numpy as np
import pandas as pd
import joblib

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine


def main():
    print("=" * 80)
    print("EXPERIMENT GROUP A: SPEED REGRESSION PHYSICAL STRATIFICATION AUDIT")
    print("=" * 80)

    data_dir = "real_iovnbd_data"
    drives = RealIOVNBDLoader.load_all_drives(data_dir)

    ckpt_path = "models/production/gbdt_speed_model_iovnbd_top45.joblib"
    ckpt = joblib.load(ckpt_path)
    gbdt_model = ckpt["model"]
    gbdt_scaler = ckpt["scaler"]
    gbdt_indices = ckpt["indices"]

    all_gt_spd = []
    all_pred_spd = []
    all_ax = []
    all_wz = []
    all_vib = []
    all_drives = []

    for name, traj in drives.items():
        dt = traj.dt
        i_calib = min(150, len(traj.time))
        R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            traj.accel[:i_calib],
            traj.gyro[:i_calib],
            traj.gt_speed[:i_calib]
        )
        accel_b = (R_p2b @ traj.accel.T).T
        gyro_b = (R_p2b @ traj.gyro.T).T

        X_57 = extract_sliding_window_features(accel_b, gyro_b, window_size=10, dt=dt, include_jerk=True, include_frequency=False)
        X_45 = X_57[:, gbdt_indices]
        v_ai = np.clip(gbdt_model.predict(gbdt_scaler.transform(X_45)), 0.0, 35.0)

        gt = traj.gt_speed
        N = min(len(gt), len(v_ai))
        all_gt_spd.extend(gt[:N])
        all_pred_spd.extend(v_ai[:N])
        all_ax.extend(accel_b[:N, 0])
        all_wz.extend(gyro_b[:N, 2])

        # Vibration proxy: norm of diff from rolling mean
        from scipy.ndimage import uniform_filter1d
        smooth_acc = uniform_filter1d(accel_b[:N], size=10, axis=0)
        vib = np.linalg.norm(accel_b[:N] - smooth_acc, axis=1)
        all_vib.extend(vib)
        all_drives.extend([name] * N)

    gt = np.array(all_gt_spd)
    pred = np.array(all_pred_spd)
    ax = np.array(all_ax)
    wz = np.array(all_wz)
    vib = np.array(all_vib)
    err = pred - gt
    abs_err = np.abs(err)

    def calc_metrics(mask, label):
        sub_gt = gt[mask]
        sub_pred = pred[mask]
        sub_err = err[mask]
        sub_abs = abs_err[mask]
        if len(sub_gt) == 0:
            return None
        mae = float(np.mean(sub_abs))
        rmse = float(np.sqrt(np.mean(sub_err**2)))
        bias = float(np.mean(sub_err))
        p95 = float(np.percentile(sub_abs, 95))
        ss_res = np.sum(sub_err**2)
        ss_tot = np.sum((sub_gt - np.mean(sub_gt))**2)
        r2 = float(1.0 - ss_res / max(ss_tot, 1e-12))
        return {
            "Regime": label,
            "Count": len(sub_gt),
            "PctData": f"{len(sub_gt)/len(gt)*100:.1f}%",
            "MAE (m/s)": f"{mae:.3f}",
            "RMSE (m/s)": f"{rmse:.3f}",
            "Bias (m/s)": f"{bias:+.3f}",
            "P95 (m/s)": f"{p95:.3f}",
            "R²": f"{r2:.3f}"
        }

    regimes = [
        (np.ones_like(gt, dtype=bool), "OVERALL FULL DATASET"),
        (gt < 0.5, "Stationary (<0.5 m/s)"),
        ((gt >= 0.5) & (gt < 1.39), "<5 km/h (0.5-1.4 m/s)"),
        ((gt >= 1.39) & (gt < 4.17), "5-15 km/h (1.4-4.2 m/s)"),
        ((gt >= 4.17) & (gt < 8.33), "15-30 km/h (4.2-8.3 m/s)"),
        (gt >= 8.33, ">30 km/h (Highway/Cruise)"),
        (ax > 0.60, "Acceleration (ax > +0.6 m/s²)"),
        (ax < -0.60, "Braking (ax < -0.6 m/s²)"),
        (np.abs(wz) > 0.10, "Cornering (|wz| > 0.10 rad/s)"),
        (vib > 1.5, "High Vibration (rough pavement)"),
    ]

    results = []
    for mask, label in regimes:
        m = calc_metrics(mask, label)
        if m:
            results.append(m)

    df_res = pd.DataFrame(results)
    print(df_res.to_string(index=False))

    # Also per-drive evaluation
    print("\nPER-DRIVE PERFORMANCE:")
    drive_res = []
    drives_arr = np.array(all_drives)
    for dname in np.unique(drives_arr):
        mask = (drives_arr == dname)
        m = calc_metrics(mask, dname)
        if m:
            drive_res.append(m)
    print(pd.DataFrame(drive_res).to_string(index=False))

    df_res.to_csv("reports/part3_gbdt_speed_stratification.csv", index=False)
    print("\nSaved reports/part3_gbdt_speed_stratification.csv")


if __name__ == "__main__":
    main()
