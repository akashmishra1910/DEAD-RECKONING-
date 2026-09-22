"""
scripts/run_step4c_iovnbd_benchmark.py
======================================
Checkpoint 4C: Fresh IO-VNBD Outage Benchmark & Comparative Navigation Evaluation.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import os
import sys
import glob
import time
import json
import joblib
import platform
import numpy as np
import pandas as pd
import sklearn

# Ensure workspace root in path
sys.path.insert(0, os.path.abspath('.'))

# Configure matplotlib for headless environment
os.environ['MPLCONFIGDIR'] = os.path.abspath('scratch/mpl_cache')
os.makedirs('scratch/mpl_cache', exist_ok=True)
os.makedirs('reports', exist_ok=True)
os.makedirs('figures', exist_ok=True)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    rodrigues,
    renormalize_rotation_matrix,
    rotation_matrix_to_euler
)
from idr_pipeline.dataset_loader import extract_sliding_window_features


def compute_along_cross_track(p_est, p_gt, v_gt):
    """Computes along-track and cross-track error components in 2D."""
    diff_2d = p_est[:, :2] - p_gt[:, :2]
    v_norm = np.linalg.norm(v_gt[:, :2], axis=1, keepdims=True)
    valid_v = (v_norm.squeeze() > 0.1)
    
    along_track = np.zeros(len(p_est))
    cross_track = np.zeros(len(p_est))
    
    if np.any(valid_v):
        unit_tangent = np.zeros_like(v_gt[:, :2])
        unit_tangent[valid_v] = v_gt[valid_v, :2] / v_norm[valid_v]
        # For samples with near-zero speed, fallback to finite difference of position
        if not np.all(valid_v):
            dp = np.gradient(p_gt[:, :2], axis=0)
            dp_norm = np.linalg.norm(dp, axis=1, keepdims=True)
            valid_dp = (dp_norm.squeeze() > 1e-4)
            unit_tangent[~valid_v & valid_dp] = dp[~valid_v & valid_dp] / dp_norm[~valid_v & valid_dp]

        along = np.sum(diff_2d * unit_tangent, axis=1)
        cross_vec = diff_2d - along[:, np.newaxis] * unit_tangent
        cross = np.linalg.norm(cross_vec, axis=1)
        
        along_track = along
        cross_track = cross
    else:
        cross_track = np.linalg.norm(diff_2d, axis=1)
        
    return along_track, cross_track


def run_benchmark():
    print("=" * 84)
    print("ESKF STEP 4C: REPRODUCIBLE IO-VNBD OUTAGE BENCHMARK (FLU + ENU UNIFIED)")
    print("=" * 84)

    # 1. Environment and Reproducibility Metadata
    meta = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "python_version": platform.python_version(),
        "numpy_version": np.__version__,
        "pandas_version": pd.__version__,
        "sklearn_version": sklearn.__version__,
        "dataset": "IO-VNBD Authentic Naturalistic Driving Dataset (72 Drives)",
        "model_checkpoint": "models/gbdt_speed_model_iovnbd_top45.joblib",
        "model_feature_count": 45,
        "pre_outage_init_seconds": 20.0,
        "outage_durations_sec": [10.0, 30.0, 60.0, 120.0],
        "held_out_drives": ["S3b", "Vta01b", "Vta04", "Vtb01", "Vw02"]
    }

    print(f"[*] Environment: Python {meta['python_version']} | NumPy {meta['numpy_version']} | Scikit-Learn {meta['sklearn_version']}")
    print(f"[*] Frozen Checkpoint: {meta['model_checkpoint']} (45 Features)")

    # 2. Load Top-45 Model Checkpoint (FROZEN)
    assert os.path.exists(meta['model_checkpoint']), f"Missing {meta['model_checkpoint']}"
    ckpt = joblib.load(meta['model_checkpoint'])
    model = ckpt['model']
    scaler = ckpt['scaler']
    indices_45 = ckpt['indices']
    assert len(indices_45) == 45, "Checkpoint must contain exactly 45 feature indices."

    # 3. Discover Drives
    base_dir = "IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset"
    s_files = sorted(glob.glob(os.path.join(base_dir, "**", "S-*.csv"), recursive=True))
    print(f"[*] Ingesting drives from: {base_dir}")
    print(f"[*] Discovered {len(s_files)} candidate smartphone logs.")

    all_drives = {}
    valid_drives_count = 0
    total_duration_sec = 0.0

    t_load0 = time.time()
    for s_path in s_files:
        dir_name = os.path.dirname(s_path)
        v_candidates = glob.glob(os.path.join(dir_name, "V-*.csv"))
        if not v_candidates:
            continue
        rel_path = os.path.relpath(s_path, base_dir)
        parts = rel_path.split(os.sep)
        cat = parts[0]
        drive_id = parts[1] if len(parts) >= 3 else os.path.splitext(os.path.basename(s_path))[0].replace("S-", "")

        try:
            traj = RealIOVNBDLoader.load_drive(s_path, v_candidates[0], name=drive_id)
            all_drives[drive_id] = (traj, cat)
            valid_drives_count += 1
            total_duration_sec += traj.time[-1]
        except Exception as e:
            print(f"[!] Warning: failed to load {drive_id}: {e}")

    print(f"[+] Successfully parsed {valid_drives_count} / {len(s_files)} drives ({total_duration_sec/3600.0:.2f} total hours) in {time.time()-t_load0:.1f}s")

    # 4. Outage Evaluation Loop
    durations = [10.0, 30.0, 60.0, 120.0]
    configs = [
        "A_Naive_INS",
        "B_AI_Vel_INS",
        "C_AI_Vel_ESKF",
        "D_AI_Vel_ESKF_NHC",
        "E_AI_Vel_ESKF_NHC_ZUPT"
    ]

    records = []
    representative_data = {}  # for plotting

    t_eval0 = time.time()
    for drive_id, (traj, cat) in all_drives.items():
        is_held_out = (drive_id in meta['held_out_drives'])
        dt = traj.dt

        # Check pre-outage requirement: 20s
        t_init = meta['pre_outage_init_seconds']
        i0 = int(t_init / dt)
        if len(traj.time) <= i0 + 10:
            continue

        # In-vehicle alignment matrix estimated from pre-outage leveling & longitudinal transients
        R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            traj.accel[:i0], traj.gyro[:i0], traj.gt_speed[:i0]
        )
        accel_b = (R_p2b @ traj.accel.T).T
        gyro_b = (R_p2b @ traj.gyro.T).T

        # Gyro bias estimated during pre-outage phase
        bg_est = np.mean(gyro_b[:i0], axis=0)

        # Precompute AI forward speeds using Top45 model
        X_57 = extract_sliding_window_features(
            accel_b, gyro_b, window_size=10, dt=dt, include_jerk=True, include_frequency=False
        )
        X_45 = X_57[:, indices_45]
        v_ai_all = np.clip(model.predict(scaler.transform(X_45)), 0.0, 35.0)

        for T in durations:
            i1 = int((t_init + T) / dt)
            if len(traj.time) <= i1:
                continue

            # Ground truth reference trajectory during outage
            gt_pos = traj.gt_pos[i0:i1]
            gt_vel = traj.gt_vel[i0:i1]
            gt_spd = traj.gt_speed[i0:i1]
            gt_hdg = traj.gt_heading[i0:i1]
            time_outage = traj.time[i0:i1] - traj.time[i0]
            N_steps = len(gt_pos)

            dist_traveled = float(np.sum(gt_spd * dt))
            # Require minimum genuine motion for drift percentage
            if dist_traveled < 5.0:
                continue

            p0 = gt_pos[0].copy()
            v0 = gt_vel[0].copy()
            yaw0 = float(gt_hdg[0])
            init_euler = np.array([0.0, 0.0, yaw0])

            # Run Configurations
            for cfg in configs:
                est_pos = np.zeros((N_steps, 3))
                est_vel = np.zeros((N_steps, 3))
                est_hdg = np.zeros(N_steps)

                if cfg == "A_Naive_INS":
                    curr_p = p0.copy()
                    curr_v = v0.copy()
                    curr_R = ErrorStateKalmanFilter(p0, v0, init_euler).R.copy()
                    est_pos[0] = curr_p
                    est_vel[0] = curr_v
                    est_hdg[0] = yaw0

                    for k in range(1, N_steps):
                        global_k = i0 + k
                        fb = accel_b[global_k]
                        wb = gyro_b[global_k] - bg_est
                        fn = curr_R @ fb
                        an = fn + np.array([0.0, 0.0, -9.80665])
                        curr_p += curr_v * dt + 0.5 * an * dt**2
                        curr_v += an * dt
                        curr_R = renormalize_rotation_matrix(curr_R @ rodrigues(wb * dt))
                        est_pos[k] = curr_p
                        est_vel[k] = curr_v
                        est_hdg[k] = rotation_matrix_to_euler(curr_R)[2]

                elif cfg == "B_AI_Vel_INS":
                    curr_p = p0.copy()
                    curr_R = ErrorStateKalmanFilter(p0, v0, init_euler).R.copy()
                    est_pos[0] = curr_p
                    est_vel[0] = v0
                    est_hdg[0] = yaw0

                    for k in range(1, N_steps):
                        global_k = i0 + k
                        wb = gyro_b[global_k] - bg_est
                        curr_R = renormalize_rotation_matrix(curr_R @ rodrigues(wb * dt))
                        vai = v_ai_all[global_k]
                        # Forward axis in nav frame is column 0
                        v_nav = vai * curr_R[:, 0]
                        curr_p += v_nav * dt
                        est_pos[k] = curr_p
                        est_vel[k] = v_nav
                        est_hdg[k] = rotation_matrix_to_euler(curr_R)[2]

                elif cfg in ("C_AI_Vel_ESKF", "D_AI_Vel_ESKF_NHC", "E_AI_Vel_ESKF_NHC_ZUPT"):
                    eskf = ErrorStateKalmanFilter(
                        init_pos=p0,
                        init_vel=v0,
                        init_euler=init_euler,
                        init_bg=bg_est,
                        acc_noise=0.20,
                        gyro_noise=0.01
                    )
                    est_pos[0] = eskf.p.copy()
                    est_vel[0] = eskf.v.copy()
                    est_hdg[0] = eskf.yaw

                    for k in range(1, N_steps):
                        global_k = i0 + k
                        fb = accel_b[global_k]
                        wb = gyro_b[global_k]
                        vai = v_ai_all[global_k]

                        eskf.predict(fb, wb, dt)
                        eskf.update_ai_velocity(vai, r_ai=0.35)

                        if cfg in ("D_AI_Vel_ESKF_NHC", "E_AI_Vel_ESKF_NHC_ZUPT"):
                            eskf.update_nhc(r_lat=0.05, r_vert=0.05)

                        if cfg == "E_AI_Vel_ESKF_NHC_ZUPT":
                            if vai < 0.2:
                                eskf.update_zupt(r_zupt=0.01)

                        est_pos[k] = eskf.p.copy()
                        est_vel[k] = eskf.v.copy()
                        est_hdg[k] = eskf.yaw

                # Metrics Calculation
                pos_err_2d = np.linalg.norm(est_pos[:, :2] - gt_pos[:, :2], axis=1)
                final_err = float(pos_err_2d[-1])
                max_err = float(np.max(pos_err_2d))
                mean_err = float(np.mean(pos_err_2d))
                rmse_err = float(np.sqrt(np.mean(pos_err_2d**2)))
                drift_pct = (final_err / dist_traveled) * 100.0

                # Velocity metrics
                vel_err_norm = np.linalg.norm(est_vel[:, :2] - gt_vel[:, :2], axis=1)
                vel_rmse = float(np.sqrt(np.mean(vel_err_norm**2)))
                vel_mae = float(np.mean(vel_err_norm))

                # Heading error (smallest signed difference)
                hdg_diff = np.arctan2(np.sin(est_hdg - gt_hdg), np.cos(est_hdg - gt_hdg))
                hdg_err_deg = float(np.degrees(np.sqrt(np.mean(hdg_diff**2))))

                # Along-track and cross-track
                along_track, cross_track = compute_along_cross_track(est_pos, gt_pos, gt_vel)
                along_rmse = float(np.sqrt(np.mean(along_track**2)))
                cross_rmse = float(np.sqrt(np.mean(cross_track**2)))

                record = {
                    "drive_id": drive_id,
                    "category": cat,
                    "is_held_out": is_held_out,
                    "config": cfg,
                    "outage_duration_s": T,
                    "outage_start_s": t_init,
                    "distance_traveled_m": round(dist_traveled, 2),
                    "mean_gt_speed_kmh": round((dist_traveled / T) * 3.6, 2),
                    "final_horizontal_error_m": round(final_err, 3),
                    "max_horizontal_error_m": round(max_err, 3),
                    "mean_horizontal_error_m": round(mean_err, 3),
                    "rmse_horizontal_error_m": round(rmse_err, 3),
                    "drift_percent": round(drift_pct, 3),
                    "velocity_rmse_ms": round(vel_rmse, 3),
                    "velocity_mae_ms": round(vel_mae, 3),
                    "heading_rmse_deg": round(hdg_err_deg, 2),
                    "along_track_rmse_m": round(along_rmse, 3),
                    "cross_track_rmse_m": round(cross_rmse, 3)
                }
                records.append(record)

                # Save representative run for plotting (Vta01b @ 60s or Vta04 @ 60s)
                if drive_id in ("Vta01b", "Vta04") and abs(T - 60.0) < 1.0:
                    key = f"{drive_id}_{T}_{cfg}"
                    representative_data[key] = {
                        "drive_id": drive_id,
                        "config": cfg,
                        "time": time_outage,
                        "gt_pos": gt_pos,
                        "est_pos": est_pos,
                        "err_2d": pos_err_2d,
                        "dist": dist_traveled
                    }

    df_results = pd.DataFrame(records)
    print(f"[+] Benchmark completed in {time.time()-t_eval0:.1f}s. Generated {len(df_results)} evaluation rows.")

    # 5. Save Raw Machine-Readable Data
    csv_out = "reports/step4c_outage_results.csv"
    df_results.to_csv(csv_out, index=False)
    print(f"[+] Raw results written to: {csv_out}")

    # Convert results to JSON structure
    json_out = "reports/step4c_outage_results.json"
    summary_dict = {
        "metadata": meta,
        "results": records
    }
    with open(json_out, "w") as f:
        json.dump(summary_dict, f, indent=2)
    print(f"[+] JSON summary written to: {json_out}")

    # 6. Generate Publication-Grade Figures
    print("\n[*] Generating high-resolution IEEE-style evaluation plots...")
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 10,
        'axes.labelsize': 11,
        'axes.titlesize': 12,
        'legend.fontsize': 9,
        'grid.alpha': 0.35
    })

    # PLOT 1: Ground Truth vs Dead Reckoning Trajectories (Vta01b / Vta04 @ 60s)
    fig, ax = plt.subplots(figsize=(10, 7), dpi=300)
    rep_keys = [k for k in representative_data.keys() if "Vta01b" in k]
    if not rep_keys:
        rep_keys = [k for k in representative_data.keys() if "Vta04" in k]

    if rep_keys:
        sample_k = rep_keys[0]
        gt_p = representative_data[sample_k]["gt_pos"]
        drive_name = representative_data[sample_k]["drive_id"]
        dist_val = representative_data[sample_k]["dist"]

        ax.plot(gt_p[:, 0], gt_p[:, 1], 'k-', lw=2.5, label='Ground Truth (RTK / CAN)')
        ax.plot(gt_p[0, 0], gt_p[0, 1], 'go', markersize=8, label='Outage Start (t=20s)')
        ax.plot(gt_p[-1, 0], gt_p[-1, 1], 'ks', markersize=8, label='Outage End (t=80s)')

        colors = {
            "A_Naive_INS": ('#d62728', '--', '1. Naive Strapdown INS'),
            "B_AI_Vel_INS": ('#ff7f0e', '-.', '2. AI Velocity + INS (Open-loop)'),
            "C_AI_Vel_ESKF": ('#9467bd', ':', '3. AI Velocity + ESKF'),
            "D_AI_Vel_ESKF_NHC": ('#1f77b4', '-', '4. AI Vel + ESKF + NHC'),
            "E_AI_Vel_ESKF_NHC_ZUPT": ('#2ca02c', '-', '5. AI Vel + ESKF + NHC + ZUPT')
        }

        for k in rep_keys:
            cfg_name = representative_data[k]["config"]
            est_p = representative_data[k]["est_pos"]
            c, ls, lbl = colors.get(cfg_name, ('gray', '-', cfg_name))
            ax.plot(est_p[:, 0], est_p[:, 1], color=c, linestyle=ls, lw=1.6, label=lbl)

        ax.set_title(f"Trajectory Evolution during 60s Outage ({drive_name}: {dist_val:.1f} m)")
        ax.set_xlabel("Local East Position (m)")
        ax.set_ylabel("Local North Position (m)")
        ax.legend(loc='best', frameon=True)
        ax.grid(True)
        ax.axis('equal')
        plt.tight_layout()
        fig.savefig("figures/step4c_groundtruth_vs_dr_trajectory.png")
        plt.close(fig)
        print("    - figures/step4c_groundtruth_vs_dr_trajectory.png")

    # PLOT 2: Position Error vs Time
    fig, ax = plt.subplots(figsize=(9, 5), dpi=300)
    if rep_keys:
        for k in rep_keys:
            cfg_name = representative_data[k]["config"]
            t_axis = representative_data[k]["time"]
            err_2d = representative_data[k]["err_2d"]
            c, ls, lbl = colors.get(cfg_name, ('gray', '-', cfg_name))
            ax.plot(t_axis, err_2d, color=c, linestyle=ls, lw=1.8, label=lbl)

        ax.axhline(0.10 * dist_val, color='r', linestyle=':', lw=1.5, label='ISRO 10% Drift Bound')
        ax.set_title(f"Horizontal Position Error vs Outage Elapsed Time ({drive_name})")
        ax.set_xlabel("Elapsed Outage Time (s)")
        ax.set_ylabel("Horizontal Position Error (m)")
        ax.set_yscale('log')
        ax.legend(loc='upper left', frameon=True)
        ax.grid(True, which='both')
        plt.tight_layout()
        fig.savefig("figures/step4c_position_error_vs_time.png")
        plt.close(fig)
        print("    - figures/step4c_position_error_vs_time.png")

    # PLOT 3: Drift Percentage by Outage Duration (Held-Out Test Set)
    df_held = df_results[df_results["is_held_out"] == True]
    if len(df_held) > 0:
        fig, ax = plt.subplots(figsize=(9, 5), dpi=300)
        mean_drift = df_held.groupby(["config", "outage_duration_s"])["drift_percent"].median().unstack()
        for cfg in configs:
            if cfg in mean_drift.index:
                c, ls, lbl = colors.get(cfg, ('gray', '-', cfg))
                ax.plot(mean_drift.columns, mean_drift.loc[cfg], marker='o', color=c, linestyle=ls, lw=2.0, label=lbl)

        ax.axhline(10.0, color='r', linestyle='--', lw=2.0, label='ISRO Benchmark Target (<10%)')
        ax.set_title("Median Drift Percentage vs Outage Duration (Held-Out Test Cohort)")
        ax.set_xlabel("Outage Duration (s)")
        ax.set_ylabel("Position Drift (% of Ground-Truth Distance)")
        ax.set_xticks(durations)
        ax.legend(loc='best', frameon=True)
        ax.grid(True)
        plt.tight_layout()
        fig.savefig("figures/step4c_drift_by_duration.png")
        plt.close(fig)
        print("    - figures/step4c_drift_by_duration.png")

    # PLOT 4: Configuration Comparison Bar Chart (Held-Out @ 60s)
    df_60 = df_held[df_held["outage_duration_s"] == 60.0]
    if len(df_60) > 0:
        fig, ax = plt.subplots(figsize=(10, 5), dpi=300)
        summary_60 = df_60.groupby("config")["drift_percent"].agg(['mean', 'median', 'std']).reindex(configs)
        bars = ax.bar(range(len(configs)), summary_60['median'], yerr=summary_60['std'], capsize=5,
                      color=['#d62728', '#ff7f0e', '#9467bd', '#1f77b4', '#2ca02c'], alpha=0.85, edgecolor='black')
        ax.axhline(10.0, color='r', linestyle='--', lw=1.8, label='ISRO 10% Bound')
        ax.set_xticks(range(len(configs)))
        ax.set_xticklabels(["1. Naive INS", "2. AI Vel + INS", "3. AI + ESKF", "4. + NHC", "5. + NHC + ZUPT"], rotation=15)
        ax.set_title("Configuration Ablation: Median Drift % @ 60s Outage (Held-Out Test Set)")
        ax.set_ylabel("Drift (% of Traveled Distance)")
        ax.legend(loc='upper right')
        ax.grid(True, axis='y')
        plt.tight_layout()
        fig.savefig("figures/step4c_configuration_comparison.png")
        plt.close(fig)
        print("    - figures/step4c_configuration_comparison.png")

    # PLOT 5: Per-Drive Error Distribution Boxplot
    if len(df_held) > 0:
        fig, ax = plt.subplots(figsize=(10, 5), dpi=300)
        data_to_plot = [df_held[(df_held["config"] == cfg) & (df_held["outage_duration_s"] == 60.0)]["final_horizontal_error_m"].dropna() for cfg in configs]
        ax.boxplot(data_to_plot, labels=["1. Naive", "2. AI+INS", "3. AI+ESKF", "4. +NHC", "5. +ZUPT"], patch_artist=True)
        ax.set_title("Distribution of Final Horizontal Error @ 60s Outage across Drives")
        ax.set_ylabel("Final Position Error (m)")
        ax.set_yscale('log')
        ax.grid(True, axis='y', which='both')
        plt.tight_layout()
        fig.savefig("figures/step4c_error_distribution.png")
        plt.close(fig)
        print("    - figures/step4c_error_distribution.png")

    # 7. Print Console Summary Tables
    print("\n" + "=" * 84)
    print("SUMMARY A: HELD-OUT TEST COHORT (S3b, Vta01b, Vta04, Vtb01, Vw02)")
    print("=" * 84)
    print(f"{'Configuration':<26s} | {'10s Drift':<12s} | {'30s Drift':<12s} | {'60s Drift':<12s} | {'120s Drift':<12s}")
    print("-" * 84)
    for cfg in configs:
        row_str = f"{cfg:<26s}"
        for T in durations:
            sub = df_held[(df_held["config"] == cfg) & (df_held["outage_duration_s"] == T)]
            if len(sub) > 0:
                med = sub["drift_percent"].median()
                row_str += f" | {med:6.2f} % ({len(sub)})"
            else:
                row_str += " |    N/A     "
        print(row_str)
    print(f"{'F_Full_System (3D MM)':<26s} | NOT AVAILABLE (OSM graph for real UK routes not implemented)")
    print("=" * 84)

    print("\n" + "=" * 84)
    print("SUMMARY B: ALL VALID IO-VNBD NATURALISTIC DRIVES (UP TO 72 DRIVES)")
    print("=" * 84)
    print(f"{'Configuration':<26s} | {'10s Drift':<12s} | {'30s Drift':<12s} | {'60s Drift':<12s} | {'120s Drift':<12s}")
    print("-" * 84)
    for cfg in configs:
        row_str = f"{cfg:<26s}"
        for T in durations:
            sub = df_results[(df_results["config"] == cfg) & (df_results["outage_duration_s"] == T)]
            if len(sub) > 0:
                med = sub["drift_percent"].median()
                row_str += f" | {med:6.2f} % ({len(sub)})"
            else:
                row_str += " |    N/A     "
        print(row_str)
    print(f"{'F_Full_System (3D MM)':<26s} | NOT AVAILABLE (OSM graph for real UK routes not implemented)")
    print("=" * 84)

    return df_results


if __name__ == "__main__":
    run_benchmark()
