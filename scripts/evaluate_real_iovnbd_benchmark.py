"""
scripts/evaluate_real_iovnbd_benchmark.py
=========================================
SIH 2026 — PS 26168 (ISRO): Intelligent Dead Reckoning Evaluation Harness.
Rigorous multi-baseline benchmark on authentic naturalistic IO-VNBD dataset.

Compares:
1. Naive Strapdown INS (Classical Double-Integration of Accelerometer)
2. Standalone AI Velocity Dead Reckoning (Open-loop AI Speed + Gyro)
3. Proposed 15-State Physics-Constrained ESKF (AI Speed + NHC + ZUPT)
4. Reference Ground Truth (Vehicle CAN / RTK-grade VBOX Logger)

Outputs:
- Publication-quality figure: figures/real_iovnbd_trajectory_plot.png
- Quantitative metrics table with drift percentages and ATE RMSE.

Author: Autonomous Navigation Engineering Team
"""

import os
import sys

# Ensure workspace root is in python module search path
sys.path.insert(0, os.path.abspath('.'))

# Ensure local cache for matplotlib
os.environ['MPLCONFIGDIR'] = os.path.abspath('scratch/mpl_cache')
os.makedirs('scratch/mpl_cache', exist_ok=True)
os.makedirs('figures', exist_ok=True)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.ai_velocity_model import AIVelocityEstimator
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter


def run_benchmark():
    print("=" * 78)
    print("ISRO PS 26168: REAL IO-VNBD MULTI-BASELINE DEAD RECKONING BENCHMARK")
    print("=" * 78)

    # 1. Load and pool real IO-VNBD drives
    data_dir = 'real_iovnbd_data'
    print(f"[*] Ingesting authentic IO-VNBD datasets from: {data_dir}...")
    drives = RealIOVNBDLoader.load_all_drives(data_dir)
    for name, d in drives.items():
        dist = np.sum(d.gt_speed * d.dt)
        print(f"    - {name:15s}: {d.time[-1]:6.1f} s | {dist:7.1f} m | max {np.max(d.gt_speed)*3.6:5.1f} km/h")

    # 2. Extract features and train GBDT speed regressor
    print("\n[*] Extracting sliding-window vibration and kinematic features...")
    # Outage test drive: Vta01b (high-speed straight arterial corridor, 1.25 km)
    test_drive_name = 'Vta01b'
    test_drive = drives[test_drive_name]
    
    # 50-second outage segment: 20s to 70s
    i0 = int(20.0 / test_drive.dt)
    i1 = int(70.0 / test_drive.dt)
    outage_dist = float(np.sum(test_drive.gt_speed[i0:i1] * test_drive.dt))
    outage_duration = float(test_drive.time[i1] - test_drive.time[i0])
    mean_speed_kmh = (outage_dist / outage_duration) * 3.6
    print(f"[*] Benchmark Outage Corridor ({test_drive_name}):")
    print(f"    Time Interval:  {test_drive.time[i0]:.1f} s -> {test_drive.time[i1]:.1f} s ({outage_duration:.1f} s)")
    print(f"    Distance:       {outage_dist:.1f} m")
    print(f"    Mean Speed:     {mean_speed_kmh:.1f} km/h (PS Target: ~50-60 km/h)")

    # Build train set excluding test outage segment
    X_train_list, y_train_list = [], []
    for name, d in drives.items():
        feats = extract_sliding_window_features(d.accel, d.gyro, window_size=10)
        if name == test_drive_name:
            # Include pre-outage (0 to i0) to calibrate vehicle-specific dynamics
            X_train_list.append(feats[:i0])
            y_train_list.append(d.gt_speed[:i0])
            # Include post-outage (i1 to end)
            if len(d.time) > i1:
                X_train_list.append(feats[i1:])
                y_train_list.append(d.gt_speed[i1:])
        else:
            X_train_list.append(feats)
            y_train_list.append(d.gt_speed)

    X_train = np.vstack(X_train_list)
    y_train = np.concatenate(y_train_list)

    print(f"\n[*] Training GBDT Speed Regressor on {len(y_train)} real samples...")
    ai_model = AIVelocityEstimator(model_type='gradient_boosting')
    metrics = ai_model.train(X_train, y_train)
    print(f"    Training RMSE: {metrics['train_rmse']:.3f} m/s | R²: {metrics['train_r2']:.4f} | MAE: {metrics['train_mae']:.3f} m/s")

    # Predict speed on test drive
    test_feats = extract_sliding_window_features(test_drive.accel, test_drive.gyro, window_size=10)
    pred_speeds = ai_model.predict(test_feats)
    outage_pred_speeds = pred_speeds[i0:i1]
    outage_gt_speeds = test_drive.gt_speed[i0:i1]

    spd_rmse = float(np.sqrt(np.mean((outage_gt_speeds - outage_pred_speeds)**2)))
    spd_mae = float(np.mean(np.abs(outage_gt_speeds - outage_pred_speeds)))
    print(f"    Outage Speed Estimation: RMSE = {spd_rmse:.3f} m/s | MAE = {spd_mae:.3f} m/s")

    # 3. Phone-to-Vehicle Alignment
    # Gravity leveling and transient alignment during pre-outage
    f_pre = np.mean(test_drive.accel[:i0], axis=0)
    z_b = f_pre / np.linalg.norm(f_pre)
    x_b_cand = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], z_b) * z_b
    x_b = x_b_cand / np.linalg.norm(x_b_cand)
    y_b = np.cross(z_b, x_b)
    R_p2b = np.vstack([x_b, y_b, z_b])

    # Pre-outage gyro bias estimation
    bg_est = np.mean(R_p2b @ test_drive.gyro[:i0].T, axis=1)

    # 4. Run Multi-Baseline Dead Reckoning during Outage
    p0 = test_drive.gt_pos[i0].copy()
    v0 = test_drive.gt_vel[i0].copy()
    yaw0 = float(test_drive.gt_heading[i0])
    init_euler = np.array([0.0, 0.0, yaw0])

    # Baseline 1: Naive Strapdown INS
    p_naive = p0.copy()
    v_naive = v0.copy()
    yaw_naive = yaw0
    naive_traj = [p_naive.copy()]

    # Baseline 2: Standalone AI Dead Reckoning
    p_ai = p0.copy()
    yaw_ai = yaw0
    ai_traj = [p_ai.copy()]

    # Proposed: 15-State ESKF
    eskf = ErrorStateKalmanFilter(
        init_pos=p0,
        init_vel=v0,
        init_euler=init_euler,
        init_bg=bg_est,
        acc_noise=0.20,
        gyro_noise=0.01
    )
    eskf_traj = [eskf.p.copy()]

    dt = test_drive.dt
    time_outage = test_drive.time[i0:i1] - test_drive.time[i0]

    for k in range(i0 + 1, i1):
        f_b = R_p2b @ test_drive.accel[k]
        w_b = R_p2b @ test_drive.gyro[k]
        idx_rel = k - i0

        # --- Baseline 1: Naive Strapdown ---
        yaw_naive += float(w_b[2] * dt)
        R_n = np.array([
            [np.sin(yaw_naive), np.cos(yaw_naive), 0.0],
            [np.cos(yaw_naive), -np.sin(yaw_naive), 0.0],
            [0.0, 0.0, 1.0]
        ])
        a_nav_naive = R_n @ f_b + np.array([0.0, 0.0, -9.80665])
        p_naive += v_naive * dt + 0.5 * a_nav_naive * dt**2
        v_naive += a_nav_naive * dt
        naive_traj.append(p_naive.copy())

        # --- Baseline 2: Standalone AI DR ---
        yaw_ai += float((w_b[2] - bg_est[2]) * dt)
        v_ai = outage_pred_speeds[idx_rel]
        v_nav_ai = np.array([v_ai * np.sin(yaw_ai), v_ai * np.cos(yaw_ai), 0.0])
        p_ai += v_nav_ai * dt
        ai_traj.append(p_ai.copy())

        # --- Proposed System: 15-State ESKF ---
        eskf.predict(f_b, w_b, dt)
        eskf.update_ai_velocity(v_ai, r_ai=0.35)
        eskf.update_nhc(r_lat=0.05, r_vert=0.05)
        if v_ai < 0.2:
            eskf.update_zupt(r_zupt=0.01)
        eskf_traj.append(eskf.p.copy())

    # Trajectories to numpy arrays
    gt_traj = test_drive.gt_pos[i0:i1]
    naive_traj = np.array(naive_traj)
    ai_traj = np.array(ai_traj)
    eskf_traj = np.array(eskf_traj)

    # 5. Compute Quantitative Metrics
    err_naive = np.linalg.norm(naive_traj[:, :2] - gt_traj[:, :2], axis=1)
    err_ai = np.linalg.norm(ai_traj[:, :2] - gt_traj[:, :2], axis=1)
    err_eskf = np.linalg.norm(eskf_traj[:, :2] - gt_traj[:, :2], axis=1)

    final_err_naive = float(err_naive[-1])
    final_err_ai = float(err_ai[-1])
    final_err_eskf = float(err_eskf[-1])

    drift_naive_pct = (final_err_naive / outage_dist) * 100.0
    drift_ai_pct = (final_err_ai / outage_dist) * 100.0
    drift_eskf_pct = (final_err_eskf / outage_dist) * 100.0

    ate_rmse_naive = float(np.sqrt(np.mean(err_naive**2)))
    ate_rmse_ai = float(np.sqrt(np.mean(err_ai**2)))
    ate_rmse_eskf = float(np.sqrt(np.mean(err_eskf**2)))

    max_err_naive = float(np.max(err_naive))
    max_err_ai = float(np.max(err_ai))
    max_err_eskf = float(np.max(err_eskf))

    print("\n" + "=" * 78)
    print(f"BENCHMARK RESULTS OVER {outage_dist:.1f} m OUTAGE ({outage_duration:.1f} s @ {mean_speed_kmh:.1f} km/h):")
    print("=" * 78)
    print(f"{'Method / Baseline':<32s} | {'Final Err (m)':<13s} | {'Drift (%)':<10s} | {'ATE RMSE (m)':<12s} | {'Max Dev (m)':<11s}")
    print("-" * 78)
    print(f"{'1. Naive Strapdown INS':<32s} | {final_err_naive:10.2f} m   | {drift_naive_pct:7.2f} %  | {ate_rmse_naive:10.2f} m  | {max_err_naive:9.2f} m")
    print(f"{'2. Standalone AI Velocity DR':<32s} | {final_err_ai:10.2f} m   | {drift_ai_pct:7.2f} %  | {ate_rmse_ai:10.2f} m  | {max_err_ai:9.2f} m")
    print(f"{'3. Proposed 15-State ESKF':<32s} | {final_err_eskf:10.2f} m   | {drift_eskf_pct:7.2f} %  | {ate_rmse_eskf:10.2f} m  | {max_err_eskf:9.2f} m")
    print("=" * 78)

    # 6. Generate Publication-Quality Figure
    print("\n[*] Generating high-resolution IEEE-style figure...")
    fig = plt.figure(figsize=(14, 10), dpi=300)
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 11,
        'axes.labelsize': 12,
        'axes.titlesize': 13,
        'legend.fontsize': 10,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'grid.alpha': 0.4
    })

    # Subplot A: 2D Planar Trajectory in Local ENU Frame
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(gt_traj[:, 0] - gt_traj[0, 0], gt_traj[:, 1] - gt_traj[0, 1],
             'k-', linewidth=2.4, label='Ground Truth (CAN / VBOX)')
    ax1.plot(naive_traj[:, 0] - gt_traj[0, 0], naive_traj[:, 1] - gt_traj[0, 1],
             'r--', linewidth=1.5, label=f'Naive INS ({drift_naive_pct:.1f}% drift)')
    ax1.plot(ai_traj[:, 0] - gt_traj[0, 0], ai_traj[:, 1] - gt_traj[0, 1],
             'orange', linestyle='-.', linewidth=1.8, label=f'Standalone AI DR ({drift_ai_pct:.1f}% drift)')
    ax1.plot(eskf_traj[:, 0] - gt_traj[0, 0], eskf_traj[:, 1] - gt_traj[0, 1],
             'b-', linewidth=2.2, label=f'Proposed 15-State ESKF ({drift_eskf_pct:.1f}% drift)')
    ax1.scatter([0], [0], color='green', s=70, zorder=5, marker='o', label='Outage Ingress (t=0s)')
    ax1.scatter(gt_traj[-1, 0] - gt_traj[0, 0], gt_traj[-1, 1] - gt_traj[0, 1],
                color='red', s=80, zorder=5, marker='*', label='True Endpoint')
    ax1.set_xlabel('Local East Position (m)')
    ax1.set_ylabel('Local North Position (m)')
    ax1.set_title('(a) 2D Planar Dead Reckoning Trajectory (IO-VNBD)')
    ax1.legend(loc='best', frameon=True)
    ax1.grid(True, linestyle=':')
    ax1.axis('equal')

    # Subplot B: Cumulative Horizontal Position Error vs Outage Time
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(time_outage, err_naive, 'r--', linewidth=1.8, label='Naive INS (Quartic Divergence)')
    ax2.plot(time_outage, err_ai, 'orange', linestyle='-.', linewidth=1.8, label='Standalone AI DR')
    ax2.plot(time_outage, err_eskf, 'b-', linewidth=2.2, label='Proposed 15-State ESKF')
    # 10% Benchmark Bound
    benchmark_bound = (test_drive.gt_speed[i0:i1] * dt).cumsum() * 0.10
    ax2.plot(time_outage, benchmark_bound, 'k:', linewidth=1.5, label='ISRO 10% Bound (PS Benchmark)')
    ax2.set_xlabel('GNSS Outage Duration (s)')
    ax2.set_ylabel('Horizontal Position Error (m)')
    ax2.set_title('(b) Horizontal Error Divergence Over 50s Outage')
    ax2.set_ylim(0, max(200, min(800, max_err_naive * 1.05)))
    ax2.legend(loc='upper left', frameon=True)
    ax2.grid(True, linestyle=':')

    # Subplot C: AI Forward Speed Regression vs Ground Truth
    ax3 = plt.subplot(2, 2, 3)
    ax3.plot(time_outage, outage_gt_speeds * 3.6, 'k-', linewidth=2.0, label='Ground Truth CAN Speed')
    ax3.plot(time_outage, outage_pred_speeds * 3.6, 'g--', linewidth=1.8, label=f'AI GBDT Regressor (RMSE={spd_rmse*3.6:.1f} km/h)')
    ax3.fill_between(time_outage, (outage_pred_speeds - spd_rmse)*3.6, (outage_pred_speeds + spd_rmse)*3.6,
                     color='green', alpha=0.15, label='±1σ Error Envelope')
    ax3.set_xlabel('GNSS Outage Duration (s)')
    ax3.set_ylabel('Longitudinal Speed (km/h)')
    ax3.set_title('(c) AI Forward Speed Inference on Real IMU Vibration')
    ax3.legend(loc='lower right', frameon=True)
    ax3.grid(True, linestyle=':')

    # Subplot D: Error Decomposition (Along-Track vs Cross-Track)
    ax4 = plt.subplot(2, 2, 4)
    # Compute along-track and cross-track errors
    yaw_ref = test_drive.gt_heading[i0:i1]
    dx = eskf_traj[:, 0] - gt_traj[:, 0]
    dy = eskf_traj[:, 1] - gt_traj[:, 1]
    # Along track: delta_p . [sin(yaw), cos(yaw)]
    along_track = dx * np.sin(yaw_ref) + dy * np.cos(yaw_ref)
    # Cross track: delta_p . [cos(yaw), -sin(yaw)]
    cross_track = dx * np.cos(yaw_ref) - dy * np.sin(yaw_ref)

    ax4.plot(time_outage, along_track, 'b-', linewidth=1.8, label='Proposed ESKF Along-Track Error')
    ax4.plot(time_outage, cross_track, 'm--', linewidth=1.8, label='Proposed ESKF Cross-Track Error')
    ax4.axhline(0, color='gray', linestyle=':', linewidth=1.0)
    ax4.set_xlabel('GNSS Outage Duration (s)')
    ax4.set_ylabel('Orthogonal Error Component (m)')
    ax4.set_title('(d) Error State Decomposition (Along vs Cross-Track)')
    ax4.legend(loc='best', frameon=True)
    ax4.grid(True, linestyle=':')

    plt.tight_layout()
    plot_path = 'figures/real_iovnbd_trajectory_plot.png'
    plt.savefig(plot_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"\n[+] Successfully saved publication plot to: {plot_path}")

    # 7. Save metrics to CSV
    metrics_df = pd.DataFrame([
        {
            'Method': 'Naive Strapdown INS',
            'Final_Error_m': round(final_err_naive, 2),
            'Relative_Drift_pct': round(drift_naive_pct, 2),
            'ATE_RMSE_m': round(ate_rmse_naive, 2),
            'Max_Deviation_m': round(max_err_naive, 2)
        },
        {
            'Method': 'Standalone AI Velocity DR',
            'Final_Error_m': round(final_err_ai, 2),
            'Relative_Drift_pct': round(drift_ai_pct, 2),
            'ATE_RMSE_m': round(ate_rmse_ai, 2),
            'Max_Deviation_m': round(max_err_ai, 2)
        },
        {
            'Method': 'Proposed 15-State ESKF',
            'Final_Error_m': round(final_err_eskf, 2),
            'Relative_Drift_pct': round(drift_eskf_pct, 2),
            'ATE_RMSE_m': round(ate_rmse_eskf, 2),
            'Max_Deviation_m': round(max_err_eskf, 2)
        }
    ])
    metrics_csv_path = 'figures/real_iovnbd_benchmark_metrics.csv'
    metrics_df.to_csv(metrics_csv_path, index=False)
    print(f"[+] Metrics exported to: {metrics_csv_path}")


if __name__ == '__main__':
    run_benchmark()
