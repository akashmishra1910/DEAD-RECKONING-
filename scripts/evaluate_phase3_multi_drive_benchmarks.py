"""
scripts/evaluate_phase3_multi_drive_benchmarks.py
=================================================
Phase 3 Final Validation Pass: Multi-Drive Outage Evaluation & Heading Observability Audit.

Evaluates across representative authentic IO-VNBD drives:
- Vta01b (Highway corridor / High Speed) [Held-Out]
- Vta04 (Suburban route / Dynamic Turns) [Held-Out]
- Driver_A_S3b (Urban route / Stop-and-Go 681s) [Held-Out]
- Vta03 (Suburban Medium Corridor 38s) [Validation/Dev]
- Vta05 (Short Dynamic Corridor 17s) [Validation/Dev]

Evaluates 10s, 30s, 60s, 120s outages where trajectory duration allows.
Compares:
1. Baseline IMU-only ESKF (AI Velocity + NHC + ZUPT, no heading aiding)
2. ESKF + Heading Supervisor (AI Velocity + NHC + ZUPT + Heading Supervisor)

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import os
import sys
import json
import joblib
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, rotation_matrix_to_euler, wrap_pi
from idr_pipeline.heading_supervisor import HeadingSupervisor, HeadingSourceType, HeadingQuality
from idr_pipeline.dataset_loader import extract_sliding_window_features


def compute_along_cross_track(p_est, p_gt, v_gt):
    """Computes signed along-track and cross-track errors in 2D ENU."""
    diff_2d = p_est[:, :2] - p_gt[:, :2]
    v_norm = np.linalg.norm(v_gt[:, :2], axis=1, keepdims=True)
    valid_v = (v_norm.squeeze() > 0.1)
    
    along_track = np.zeros(len(p_est))
    cross_track = np.zeros(len(p_est))
    
    if np.any(valid_v):
        unit_tangent = np.zeros_like(v_gt[:, :2])
        unit_tangent[valid_v] = v_gt[valid_v, :2] / v_norm[valid_v]
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


def evaluate_outage_segment(
    drive,
    pred_speeds,
    accel_b,
    gyro_b,
    R_p2b,
    t_start: float,
    outage_dur: float,
    use_heading_supervisor: bool = False
):
    dt = drive.dt
    N = len(drive.time)
    
    i0 = int(round(t_start / dt))
    i1 = int(round((t_start + outage_dur) / dt))
    
    if i1 >= N:
        return None
        
    outage_dist = float(np.sum(drive.gt_speed[i0:i1] * dt))
    if outage_dist < 1.0:
        return None

    # Warm up filter before outage (from t=0 to t=i0) with GNSS available
    yaw_init = float(drive.gt_heading[0])
    v_init = np.array([
        drive.gt_speed[0] * np.sin(yaw_init),
        drive.gt_speed[0] * np.cos(yaw_init),
        0.0
    ])
    p_init = drive.gt_pos[0].copy()

    eskf = ErrorStateKalmanFilter(
        init_pos=p_init,
        init_vel=v_init,
        init_euler=np.array([0.0, 0.0, yaw_init]),
        acc_noise=0.20,
        gyro_noise=0.01
    )
    
    supervisor = HeadingSupervisor(
        chi2_gate=6.635,
        min_gnss_speed_mps=2.5,
        ref_mag_field_uT=float(np.mean(np.linalg.norm(drive.mag, axis=1))),
        enable_magnetometer=use_heading_supervisor,
        enable_gnss_course=True
    )

    # 1. Warm-up Phase: run with full GNSS fixes
    for k in range(1, i0):
        t_k = float(drive.time[k])
        eskf.predict(accel_b[k], gyro_b[k], dt)
        sp = float(pred_speeds[k])
        eskf.update_ai_velocity(sp, r_ai=1.0)
        if sp < 0.2:
            eskf.update_zupt(r_zupt=0.01)
        else:
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)
            
        # GNSS fix active during warmup
        eskf.update_gnss(drive.gt_pos[k], r_gnss=2.5)
        eskf.update_gnss_velocity(drive.gt_vel[k], r_vel=0.2)
        
        # Heading supervisor with GNSS course
        supervisor.process_step(
            time_s=t_k,
            eskf=eskf,
            R_p2b=R_p2b,
            gnss_vel=drive.gt_vel[k],
            gnss_valid=True,
            mag_raw_p=drive.mag[k],
            current_yaw_rate=float(gyro_b[k, 2]),
            is_forward=True
        )

    # Snapshot state at start of outage
    p_est = np.zeros((i1 - i0, 3))
    v_est = np.zeros((i1 - i0, 3))
    yaw_est = np.zeros(i1 - i0)
    
    p_est[0] = eskf.p
    v_est[0] = eskf.v
    yaw_est[0] = rotation_matrix_to_euler(eskf.R)[2]

    # Tracking source statistics during outage
    gnss_stats = {
        'available': 0,
        'accepted': 0,
        'rejected_low_speed': 0,
        'rejected_gate': 0,
        'reacquisitions': 0
    }
    mag_stats = {
        'available': 0,
        'accepted': 0,
        'rejected_intensity': 0,
        'rejected_gradient': 0,
        'rejected_physical': 0,
        'rejected_gate': 0,
        'reacquisitions': 0
    }

    # 2. Outage Phase: GNSS is completely blocked (gnss_valid=False, gnss_vel=None)
    for step_idx, k in enumerate(range(i0 + 1, i1), start=1):
        t_k = float(drive.time[k])
        
        # Prediction
        eskf.predict(accel_b[k], gyro_b[k], dt)
        
        # AI speed update
        sp = float(pred_speeds[k])
        eskf.update_ai_velocity(sp, r_ai=1.0)
        
        # Kinematic constraints
        if sp < 0.2:
            eskf.update_zupt(r_zupt=0.01)
        else:
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)
            
        # Heading supervision during outage
        if use_heading_supervisor:
            mag_stats['available'] += 1
            telem = supervisor.process_step(
                time_s=t_k,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=None,
                gnss_valid=False,
                mag_raw_p=drive.mag[k],
                current_yaw_rate=float(gyro_b[k, 2]),
                is_forward=True
            )
            if telem.is_updated:
                if telem.selected_source == HeadingSourceType.MAGNETOMETER:
                    mag_stats['accepted'] += 1
                else:
                    gnss_stats['accepted'] += 1
            else:
                if 'INTENSITY' in telem.rejection_reason:
                    mag_stats['rejected_intensity'] += 1
                elif 'GRADIENT' in telem.rejection_reason:
                    mag_stats['rejected_gradient'] += 1
                elif 'INNOVATION_EXCEEDED' in telem.rejection_reason or 'GATE' in telem.rejection_reason:
                    mag_stats['rejected_gate'] += 1
                elif 'PHYSICAL' in telem.rejection_reason or 'INVALID' in telem.rejection_reason:
                    mag_stats['rejected_physical'] += 1

        p_est[step_idx] = eskf.p
        v_est[step_idx] = eskf.v
        yaw_est[step_idx] = rotation_matrix_to_euler(eskf.R)[2]

    # Metrics computation
    p_gt = drive.gt_pos[i0:i1]
    v_gt = drive.gt_vel[i0:i1]
    gt_yaw = drive.gt_heading[i0:i1]
    
    final_pos_err = float(np.linalg.norm(p_est[-1] - p_gt[-1]))
    drift_pct = float(100.0 * final_pos_err / max(outage_dist, 1.0))
    
    along_track, cross_track = compute_along_cross_track(p_est, p_gt, v_gt)
    final_along = float(along_track[-1])
    final_cross = float(cross_track[-1])
    mean_along = float(np.mean(np.abs(along_track)))
    mean_cross = float(np.mean(cross_track))
    
    heading_errors_deg = np.degrees(np.abs(np.array([wrap_pi(y - gy) for y, gy in zip(yaw_est, gt_yaw)])))
    mean_hdg_err = float(np.mean(heading_errors_deg))
    median_hdg_err = float(np.median(heading_errors_deg))
    max_hdg_err = float(np.max(heading_errors_deg))

    return {
        'final_pos_err_m': final_pos_err,
        'drift_pct': drift_pct,
        'outage_dist_m': outage_dist,
        'outage_dur_s': outage_dur,
        'final_along_m': final_along,
        'final_cross_m': final_cross,
        'mean_along_m': mean_along,
        'mean_cross_m': mean_cross,
        'mean_hdg_err_deg': mean_hdg_err,
        'median_hdg_err_deg': median_hdg_err,
        'max_hdg_err_deg': max_hdg_err,
        'gnss_stats': gnss_stats,
        'mag_stats': mag_stats
    }


def main():
    print("=" * 84)
    print("PHASE 3 FINAL VALIDATION PASS: MULTI-DRIVE IO-VNBD HEADING OBSERVABILITY")
    print("=" * 84)
    
    checkpoint_path = "models/gbdt_speed_model_iovnbd_top45.joblib"
    if not os.path.exists(checkpoint_path):
        checkpoint_path = "models/gbdt_speed_model_iovnbd.joblib"
        
    print(f"[*] Loading frozen GBDT model checkpoint: {checkpoint_path}")
    checkpoint = joblib.load(checkpoint_path)
    model = checkpoint["model"]
    scaler = checkpoint["scaler"]
    top_indices = checkpoint.get("indices", None)

    drives = RealIOVNBDLoader.load_all_drives('real_iovnbd_data')
    print(f"[*] Loaded {len(drives)} authentic IO-VNBD drives: {list(drives.keys())}")

    # Dataset split definition
    dataset_split = {
        'Vta01b': {'role': 'HELD_OUT_BENCHMARK', 'type': 'Highway Corridor / High Speed'},
        'Vta04': {'role': 'HELD_OUT_BENCHMARK', 'type': 'Suburban Route / Dynamic Turns'},
        'Driver_A_S3b': {'role': 'HELD_OUT_BENCHMARK', 'type': 'Urban Route / Stop-and-Go (681s)'},
        'Vta03': {'role': 'DEV_VALIDATION', 'type': 'Suburban Medium Corridor (38s)'},
        'Vta05': {'role': 'DEV_VALIDATION', 'type': 'Short Dynamic Corridor (17s)'}
    }

    outage_durations = [10.0, 30.0, 60.0, 120.0]
    results = []
    
    for name, drive in drives.items():
        role_info = dataset_split.get(name, {'role': 'UNKNOWN', 'type': 'Standard Drive'})
        print(f"\n---> Processing Drive: {name:15s} [{role_info['role']}] ({role_info['type']})")
        print(f"     Duration: {drive.time[-1]:.1f}s | Samples: {len(drive.time)} | Max Speed: {drive.gt_speed.max()*3.6:.1f} km/h")
        
        # In-Vehicle Alignment
        init_N = min(len(drive.time), 200)
        R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            drive.accel[:init_N], drive.gyro[:init_N], drive.gt_speed[:init_N]
        )
        accel_b = (R_p2b @ drive.accel.T).T
        gyro_b = (R_p2b @ drive.gyro.T).T
        
        # Feature Extraction
        feats_all = extract_sliding_window_features(
            accel_b, gyro_b, window_size=10,
            include_temporal_extrema=True, include_jerk=True, include_frequency=True
        )
        if top_indices is not None and feats_all.shape[1] > len(top_indices):
            feats_used = feats_all[:, top_indices]
        else:
            feats_used = feats_all
            
        pred_speeds = np.clip(model.predict(scaler.transform(feats_used)), 0.0, 35.0)

        # Evaluate across available outage durations
        for dur in outage_durations:
            # We select a stable outage start window at t = 20s (or 5s for short drives)
            t_start = 20.0 if drive.time[-1] >= 20.0 + dur else 5.0
            if t_start + dur > drive.time[-1]:
                continue
                
            res_base = evaluate_outage_segment(drive, pred_speeds, accel_b, gyro_b, R_p2b, t_start, dur, use_heading_supervisor=False)
            res_aided = evaluate_outage_segment(drive, pred_speeds, accel_b, gyro_b, R_p2b, t_start, dur, use_heading_supervisor=True)
            
            if res_base is None or res_aided is None:
                continue
                
            improvement_factor = res_base['final_pos_err_m'] / max(res_aided['final_pos_err_m'], 1e-3)
            
            row = {
                'drive_name': name,
                'role': role_info['role'],
                'drive_type': role_info['type'],
                'outage_dur_s': dur,
                'outage_dist_m': res_base['outage_dist_m'],
                # Baseline metrics
                'base_pos_err_m': res_base['final_pos_err_m'],
                'base_drift_pct': res_base['drift_pct'],
                'base_cross_m': res_base['final_cross_m'],
                'base_along_m': res_base['final_along_m'],
                'base_mean_hdg_err': res_base['mean_hdg_err_deg'],
                'base_max_hdg_err': res_base['max_hdg_err_deg'],
                # Heading-aided metrics
                'aided_pos_err_m': res_aided['final_pos_err_m'],
                'aided_drift_pct': res_aided['drift_pct'],
                'aided_cross_m': res_aided['final_cross_m'],
                'aided_along_m': res_aided['final_along_m'],
                'aided_mean_hdg_err': res_aided['mean_hdg_err_deg'],
                'aided_median_hdg_err': res_aided['median_hdg_err_deg'],
                'aided_max_hdg_err': res_aided['max_hdg_err_deg'],
                'improvement_factor': improvement_factor,
                'gnss_stats': res_aided['gnss_stats'],
                'mag_stats': res_aided['mag_stats']
            }
            results.append(row)
            print(f"     Outage {dur:3.0f}s ({res_base['outage_dist_m']:5.1f}m): Baseline Drift = {res_base['drift_pct']:5.2f}% ({res_base['final_pos_err_m']:5.2f}m) -> Aided Drift = {res_aided['drift_pct']:5.2f}% ({res_aided['final_pos_err_m']:5.2f}m) [Hdg Err: {res_aided['mean_hdg_err_deg']:4.1f}°] -> {improvement_factor:4.1f}x")

    df_res = pd.DataFrame(results)
    df_res.to_csv('reports/phase3_multi_drive_outage_benchmark.csv', index=False)
    print("\n[+] Saved detailed multi-drive evaluation to reports/phase3_multi_drive_outage_benchmark.csv")

    # Aggregate Statistics per outage duration
    print("\n" + "=" * 84)
    print("AGGREGATE BENCHMARK STATISTICS ACROSS AUTHENTIC IO-VNBD DRIVES")
    print("=" * 84)
    for dur in outage_durations:
        sub = df_res[df_res['outage_dur_s'] == dur]
        if len(sub) == 0:
            continue
        print(f"\n--- Outage Duration: {dur:.0f}s (N = {len(sub)} evaluation runs) ---")
        print(f"  Baseline Pos Error (m):  Median = {sub['base_pos_err_m'].median():6.2f} | Mean = {sub['base_pos_err_m'].mean():6.2f} | P25 = {sub['base_pos_err_m'].quantile(0.25):6.2f} | P75 = {sub['base_pos_err_m'].quantile(0.75):6.2f} | Max = {sub['base_pos_err_m'].max():6.2f}")
        print(f"  Baseline Drift (%):      Median = {sub['base_drift_pct'].median():6.2f}% | Mean = {sub['base_drift_pct'].mean():6.2f}% | P25 = {sub['base_drift_pct'].quantile(0.25):6.2f}% | P75 = {sub['base_drift_pct'].quantile(0.75):6.2f}% | Max = {sub['base_drift_pct'].max():6.2f}%")
        print(f"  Aided Pos Error (m):     Median = {sub['aided_pos_err_m'].median():6.2f} | Mean = {sub['aided_pos_err_m'].mean():6.2f} | P25 = {sub['aided_pos_err_m'].quantile(0.25):6.2f} | P75 = {sub['aided_pos_err_m'].quantile(0.75):6.2f} | Max = {sub['aided_pos_err_m'].max():6.2f}")
        print(f"  Aided Drift (%):         Median = {sub['aided_drift_pct'].median():6.2f}% | Mean = {sub['aided_drift_pct'].mean():6.2f}% | P25 = {sub['aided_drift_pct'].quantile(0.25):6.2f}% | P75 = {sub['aided_drift_pct'].quantile(0.75):6.2f}% | Max = {sub['aided_drift_pct'].max():6.2f}%")
        print(f"  Aided Cross-Track (m):   Median = {sub['aided_cross_m'].median():6.2f} | Mean = {sub['aided_cross_m'].mean():6.2f} | P25 = {sub['aided_cross_m'].quantile(0.25):6.2f} | P75 = {sub['aided_cross_m'].quantile(0.75):6.2f} | Max = {sub['aided_cross_m'].max():6.2f}")
        print(f"  Aided Along-Track (m):   Median = {sub['aided_along_m'].median():6.2f} | Mean = {sub['aided_along_m'].mean():6.2f} | P25 = {sub['aided_along_m'].quantile(0.25):6.2f} | P75 = {sub['aided_along_m'].quantile(0.75):6.2f} | Max = {sub['aided_along_m'].max():6.2f}")
        print(f"  Aided Heading Error (°): Median = {sub['aided_median_hdg_err'].median():6.2f}° | Mean = {sub['aided_mean_hdg_err'].mean():6.2f}° | Max = {sub['aided_max_hdg_err'].max():6.2f}°")


if __name__ == '__main__':
    main()
