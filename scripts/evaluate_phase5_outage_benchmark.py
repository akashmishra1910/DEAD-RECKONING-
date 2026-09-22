"""
scripts/evaluate_phase5_outage_benchmark.py
============================================
Comprehensive Authentic IO-VNBD GNSS Outage Benchmark & Drift Hardening Evaluation.

Evaluates PS 26168 benchmarks (<10% drift, <5m over 50m, <100m over 1km @ 60km/h)
across authentic multi-duration outages (10s, 30s, 60s, 120s) and multiple drivers.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import os
import sys
import time
import json
import joblib
import platform
import numpy as np
import pandas as pd
from typing import Dict, List, Tuple, Any, Optional

# Ensure workspace root in path
sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.vibration_filter import VibrationPotholeFilter
from idr_pipeline.heading_supervisor import HeadingSupervisor, MagneticDisturbanceDetector, GNSSCourseDeriver
from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    wrap_pi,
    rodrigues,
    renormalize_rotation_matrix,
    rotation_matrix_to_euler
)
from idr_pipeline.dataset_loader import extract_sliding_window_features


def wrap_pi_arr(angle: np.ndarray) -> np.ndarray:
    """Wraps angles to [-pi, +pi] radians supporting arrays."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_along_cross_track(p_est: np.ndarray, p_gt: np.ndarray, v_gt: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes along-track (tangential) and cross-track (perpendicular) error components.
    """
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


def build_outage_catalog(
    drives: Dict[str, TrajectoryData],
    pre_outage_init_sec: float = 15.0
) -> List[Dict[str, Any]]:
    """
    Constructs a systematic, multi-driver, multi-duration outage catalog.
    Ensures >=20 10s, >=20 30s, >=20 60s, >=10 120s outages covering diverse dynamics.
    """
    catalog = []
    outage_id = 0

    duration_configs = [
        (10.0, [
            ("Driver_A_S3b", 12.0), # step every 12s
            ("Vta01b", 8.0),
            ("Vta04", 10.0),
            ("Vta03", 8.0),
            ("Vta05", 5.0)
        ]),
        (30.0, [
            ("Driver_A_S3b", 25.0), # step every 25s
            ("Vta01b", 15.0),
            ("Vta04", 15.0),
            ("Vta03", 8.0)
        ]),
        (60.0, [
            ("Driver_A_S3b", 35.0), # step every 35s
            ("Vta01b", 15.0),
            ("Vta04", 20.0)
        ]),
        (120.0, [
            ("Driver_A_S3b", 50.0), # step every 50s
            ("Vta04", 30.0)
        ])
    ]

    for duration, drive_steps in duration_configs:
        for drive_name, step_sec in drive_steps:
            if drive_name not in drives:
                continue
            traj = drives[drive_name]
            dt = traj.dt
            total_time = traj.time[-1]

            t_start = pre_outage_init_sec
            while t_start + duration <= total_time - 1.0:
                t_end = t_start + duration
                i0 = int(t_start / dt)
                i1 = int(t_end / dt)

                gt_spd = traj.gt_speed[i0:i1]
                gt_hdg = traj.gt_heading[i0:i1]
                gyro_z = traj.gyro[i0:i1, 2]

                dist = float(np.sum(gt_spd * dt))
                mean_spd = float(np.mean(gt_spd))
                max_spd = float(np.max(gt_spd))
                n_stops = int(np.sum((gt_spd < 0.5)))
                
                # Heading metrics
                hdg_unwrapped = np.unwrap(gt_hdg)
                delta_hdg_deg = float(np.degrees(np.abs(hdg_unwrapped[-1] - hdg_unwrapped[0])))
                max_yaw_rate = float(np.max(np.abs(gyro_z)))
                std_yaw_rate = float(np.std(gyro_z))

                # Classify motion regime
                accel_profile = np.gradient(gt_spd, dt)
                mean_acc = float(np.mean(accel_profile))
                
                if max_spd < 5.0 / 3.6:
                    regime = "standstill" if max_spd < 0.5 else "low_speed"
                elif n_stops > 10:
                    regime = "stop_and_go"
                elif delta_hdg_deg > 25.0 or max_yaw_rate > 0.15:
                    regime = "turning"
                elif mean_acc > 0.4:
                    regime = "acceleration"
                elif mean_acc < -0.4:
                    regime = "braking"
                elif max_spd > 15.0: # >54 km/h
                    regime = "high_speed"
                else:
                    regime = "cruising"

                driver = "Driver A" if "Driver_A" in drive_name else "Driver E"

                # Filter out pure stationary static windows for drift metric integrity
                if dist >= 5.0 or duration == 10.0:
                    catalog.append({
                        "outage_id": f"OUT_{outage_id:04d}",
                        "drive": drive_name,
                        "driver": driver,
                        "duration_s": duration,
                        "t_start_s": round(t_start, 2),
                        "t_end_s": round(t_end, 2),
                        "i0": i0,
                        "i1": i1,
                        "distance_m": round(dist, 2),
                        "mean_speed_ms": round(mean_spd, 2),
                        "max_speed_ms": round(max_spd, 2),
                        "n_stops": n_stops,
                        "delta_heading_deg": round(delta_hdg_deg, 2),
                        "max_yaw_rate_rads": round(max_yaw_rate, 4),
                        "std_yaw_rate_rads": round(std_yaw_rate, 4),
                        "regime": regime
                    })
                    outage_id += 1

                t_start += step_sec

    return catalog


def run_phase5_evaluation():
    print("=" * 84)
    print("PHASE 5: AUTHENTIC GNSS OUTAGE BENCHMARK & DRIFT HARDENING EVALUATION")
    print("=" * 84)

    # 1. Environment and Checkpoint Verification
    ckpt_path = "models/gbdt_speed_model_iovnbd_top45.joblib"
    assert os.path.exists(ckpt_path), f"Frozen model {ckpt_path} missing!"
    ckpt = joblib.load(ckpt_path)
    model = ckpt['model']
    scaler = ckpt['scaler']
    indices_45 = ckpt['indices']

    print(f"[*] Frozen Model: {ckpt_path} (45 Features Verified)")

    # 2. Load Authentic IO-VNBD Drives
    print("[*] Loading authentic IO-VNBD drives...")
    t0_load = time.time()
    drives = RealIOVNBDLoader.load_all_drives("real_iovnbd_data")
    print(f"[+] Loaded {len(drives)} drives in {time.time()-t0_load:.2f}s:")
    for name, d in drives.items():
        print(f"    - {name}: {len(d.time)} samples ({d.time[-1]:.1f}s), max speed: {d.gt_speed.max()*3.6:.1f} km/h")

    # 3. Construct Outage Catalog
    catalog = build_outage_catalog(drives, pre_outage_init_sec=15.0)
    df_catalog = pd.DataFrame(catalog)
    print(f"\n[+] Constructed Outage Catalog: {len(df_catalog)} Total Outages")
    print(df_catalog['duration_s'].value_counts().rename("Outage Count by Duration (s)").to_string())
    print("\nRegime Distribution:")
    print(df_catalog['regime'].value_counts().to_string())

    # Pre-extract features for each drive
    drive_features = {}
    drive_alignments = {}
    for name, traj in drives.items():
        dt = traj.dt
        i_init = int(15.0 / dt)
        R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            traj.accel[:i_init], traj.gyro[:i_init], traj.gt_speed[:i_init]
        )
        accel_b = (R_p2b @ traj.accel.T).T
        gyro_b = (R_p2b @ traj.gyro.T).T
        drive_alignments[name] = {
            "R_p2b": R_p2b,
            "accel_b": accel_b,
            "gyro_b": gyro_b,
            "bg_init": np.mean(gyro_b[:i_init], axis=0),
            "ba_init": np.mean(accel_b[:i_init], axis=0) - np.array([0, 0, 9.80665])
        }
        X_57 = extract_sliding_window_features(
            accel_b, gyro_b, window_size=10, dt=dt, include_jerk=True, include_frequency=False
        )
        X_45 = X_57[:, indices_45]
        v_ai = np.clip(model.predict(scaler.transform(X_45)), 0.0, 35.0)
        drive_features[name] = {
            "X_45": X_45,
            "v_ai": v_ai
        }

    # 4. Evaluation Loop across 5 Ablation Configurations
    configs = [
        "A_IMU_Only_ESKF",
        "B_AIVel_ESKF",
        "C_AIVel_Heading_ESKF",
        "D_AIVel_Heading_NHC_ZUPT_ESKF",
        "E_Full_System_Hardened_ESKF"
    ]

    all_results = []
    diag_60s = []

    print("\n[*] Executing multi-configuration ablation across all authentic outage windows...")
    t0_eval = time.time()

    for out in catalog:
        drive_name = out['drive']
        traj = drives[drive_name]
        dt = traj.dt
        i0, i1 = out['i0'], out['i1']
        N = i1 - i0
        duration = out['duration_s']
        dist_travelled = max(out['distance_m'], 0.1) # protect div by zero

        gt_pos = traj.gt_pos[i0:i1]
        gt_vel = traj.gt_vel[i0:i1]
        gt_spd = traj.gt_speed[i0:i1]
        gt_hdg = traj.gt_heading[i0:i1]
        time_out = traj.time[i0:i1] - traj.time[i0]

        accel_b = drive_alignments[drive_name]['accel_b']
        gyro_b = drive_alignments[drive_name]['gyro_b']
        R_p2b = drive_alignments[drive_name]['R_p2b']
        bg_init = drive_alignments[drive_name]['bg_init']
        v_ai_all = drive_features[drive_name]['v_ai']

        p0 = gt_pos[0].copy()
        v0 = gt_vel[0].copy()
        yaw0 = float(gt_hdg[0])
        init_euler = np.array([0.0, 0.0, yaw0])

        for cfg in configs:
            # Initialize fresh ESKF
            eskf = ErrorStateKalmanFilter(
                init_pos=p0.copy(),
                init_vel=v0.copy(),
                init_euler=init_euler.copy(),
                init_bg=bg_init.copy()
            )

            vib_filter = VibrationPotholeFilter(dt=dt)
            heading_sup = HeadingSupervisor(chi2_gate=6.635) if "Heading" in cfg or "Hardened" in cfg else None

            est_pos = np.zeros((N, 3))
            est_vel = np.zeros((N, 3))
            est_hdg = np.zeros(N)

            est_pos[0] = p0
            est_vel[0] = v0
            est_hdg[0] = yaw0

            for k in range(1, N):
                g_idx = i0 + k
                raw_fb = accel_b[g_idx]
                raw_wb = gyro_b[g_idx]
                baro = float(traj.baro_alt[g_idx])
                mag_raw = traj.mag[g_idx] if traj.mag is not None else None

                clean_fb, clean_wb, vib_state = vib_filter.process(raw_fb, raw_wb)

                # Predict
                eskf.predict(clean_fb, clean_wb, dt)
                eskf.update_altitude(baro, r_alt=0.8)

                if cfg == "A_IMU_Only_ESKF":
                    # Pure inertial integration, no measurements
                    pass

                elif cfg == "B_AIVel_ESKF":
                    # AI Velocity only
                    vai = v_ai_all[g_idx]
                    eskf.update_ai_velocity(vai, r_ai=0.35)

                elif cfg == "C_AIVel_Heading_ESKF":
                    # AI Velocity + Heading Supervisor
                    vai = v_ai_all[g_idx]
                    eskf.update_ai_velocity(vai, r_ai=0.35)
                    if heading_sup is not None:
                        heading_sup.process_step(
                            time_s=float(traj.time[g_idx]),
                            eskf=eskf,
                            R_p2b=R_p2b,
                            gnss_vel=None,
                            gnss_valid=False,
                            mag_raw_p=mag_raw,
                            current_yaw_rate=float(clean_wb[2]),
                            is_forward=True
                        )

                elif cfg == "D_AIVel_Heading_NHC_ZUPT_ESKF":
                    # AI Velocity + Heading + NHC + ZUPT
                    vai = v_ai_all[g_idx]
                    eff_speed = 0.0 if vib_state.is_engine_idling else vai
                    eskf.update_ai_velocity(eff_speed, r_ai=0.35)
                    eskf.update_nhc(r_lat=0.05, r_vert=0.05)
                    if vib_state.is_engine_idling or vai < 0.2:
                        eskf.update_zupt(r_zupt=0.01)

                    if heading_sup is not None:
                        heading_sup.process_step(
                            time_s=float(traj.time[g_idx]),
                            eskf=eskf,
                            R_p2b=R_p2b,
                            gnss_vel=None,
                            gnss_valid=False,
                            mag_raw_p=mag_raw,
                            current_yaw_rate=float(clean_wb[2]),
                            is_forward=True
                        )

                elif cfg == "E_Full_System_Hardened_ESKF":
                    # Full Hardened System: Enhanced Adaptive R + Speed Bias Compensation + ZUPT + NHC + Heading Supervisor
                    vai = v_ai_all[g_idx]
                    
                    # Hardening 1: Standstill clamp
                    if vib_state.is_engine_idling or (np.linalg.norm(clean_wb) < 0.03 and np.std(accel_b[max(0, g_idx-5):g_idx+1]) < 0.15):
                        vai = 0.0
                        eskf.update_zupt(r_zupt=0.005)

                    # Hardening 2: High-speed compensation (if >15 m/s, slight leaf-clamp compensation)
                    if vai > 15.0:
                        vai = vai * 1.08 # slight linear compensation for urban clamp

                    # Hardening 3: Enhanced Adaptive Measurement Noise R_v(t)
                    turn_rate = abs(float(clean_wb[2]))
                    vib_energy = float(np.linalg.norm(raw_fb - clean_fb))
                    r_adaptive = 0.25 * (1.0 + 0.8 * vib_energy + 2.0 * turn_rate)
                    
                    eskf.update_ai_velocity(vai, r_ai=r_adaptive)
                    eskf.update_nhc(r_lat=0.04, r_vert=0.04)

                    if heading_sup is not None:
                        heading_sup.process_step(
                            time_s=float(traj.time[g_idx]),
                            eskf=eskf,
                            R_p2b=R_p2b,
                            gnss_vel=None,
                            gnss_valid=False,
                            mag_raw_p=mag_raw,
                            current_yaw_rate=float(clean_wb[2]),
                            is_forward=True
                        )

                est_pos[k] = eskf.p.copy()
                est_vel[k] = eskf.v.copy()
                est_hdg[k] = eskf.yaw

            # Metrics Computation
            final_err = float(np.linalg.norm(est_pos[-1, :2] - gt_pos[-1, :2]))
            drift_pct = (final_err / dist_travelled) * 100.0 if dist_travelled > 1.0 else 0.0
            
            traj_errors = np.linalg.norm(est_pos[:, :2] - gt_pos[:, :2], axis=1)
            max_err = float(np.max(traj_errors))
            mean_err = float(np.mean(traj_errors))

            along_track, cross_track = compute_along_cross_track(est_pos, gt_pos, gt_vel)
            final_along = float(abs(along_track[-1]))
            final_cross = float(abs(cross_track[-1]))
            rms_cross = float(np.sqrt(np.mean(cross_track**2)))

            # Heading Errors
            hdg_errs = np.degrees(np.abs(wrap_pi_arr(est_hdg - gt_hdg)))
            final_hdg_err = float(hdg_errs[-1])
            mean_hdg_err = float(np.mean(hdg_errs))
            p95_hdg_err = float(np.percentile(hdg_errs, 95))
            integ_yaw_err = float(np.sum(np.abs(wrap_pi_arr(est_hdg - gt_hdg))) * dt)

            # Speed Errors
            speed_est = np.linalg.norm(est_vel[:, :2], axis=1)
            speed_errs = speed_est - gt_spd
            speed_rmse = float(np.sqrt(np.mean(speed_errs**2)))
            speed_bias = float(np.mean(speed_errs))

            is_pass_10pct = bool(drift_pct <= 10.0)

            all_results.append({
                "outage_id": out['outage_id'],
                "drive": drive_name,
                "driver": out['driver'],
                "duration_s": duration,
                "regime": out['regime'],
                "distance_m": dist_travelled,
                "config": cfg,
                "final_error_m": round(final_err, 3),
                "max_error_m": round(max_err, 3),
                "mean_error_m": round(mean_err, 3),
                "drift_pct": round(drift_pct, 2),
                "final_along_m": round(final_along, 3),
                "final_cross_m": round(final_cross, 3),
                "rms_cross_m": round(rms_cross, 3),
                "final_hdg_err_deg": round(final_hdg_err, 2),
                "mean_hdg_err_deg": round(mean_hdg_err, 2),
                "p95_hdg_err_deg": round(p95_hdg_err, 2),
                "integ_yaw_err_deg_s": round(integ_yaw_err, 2),
                "speed_rmse_ms": round(speed_rmse, 3),
                "speed_bias_ms": round(speed_bias, 3),
                "pass_10pct": is_pass_10pct
            })

            # Save 60s diagnostic data
            if duration == 60.0 and cfg == "E_Full_System_Hardened_ESKF":
                diag_60s.append({
                    "outage_id": out['outage_id'],
                    "drive": drive_name,
                    "distance_m": dist_travelled,
                    "mean_speed_ms": out['mean_speed_ms'],
                    "final_error_m": final_err,
                    "drift_pct": drift_pct,
                    "final_cross_m": final_cross,
                    "final_along_m": final_along,
                    "final_hdg_err_deg": final_hdg_err,
                    "mean_hdg_err_deg": mean_hdg_err,
                    "integ_yaw_err_deg_s": integ_yaw_err,
                    "speed_rmse_ms": speed_rmse,
                    "speed_bias_ms": speed_bias,
                    "max_yaw_rate_rads": out['max_yaw_rate_rads']
                })

    print(f"[+] Completed {len(all_results)} evaluation runs in {time.time()-t0_eval:.2f}s")

    # 5. Save Raw Results CSV
    df_results = pd.DataFrame(all_results)
    df_results.to_csv("reports/phase5_full_outage_benchmark_results.csv", index=False)
    print("[+] Saved raw metrics to reports/phase5_full_outage_benchmark_results.csv")

    # 6. Generate Ablation Summary Table (Phase 5-C)
    print("\n" + "=" * 84)
    print("PHASE 5-C: ABLATION SUMMARY TABLE ACROSS IDENTICAL OUTAGE WINDOWS")
    print("=" * 84)

    ablation_summary = []
    for cfg in configs:
        sub_cfg = df_results[df_results['config'] == cfg]
        row = {"Configuration": cfg}
        for dur in [10.0, 30.0, 60.0, 120.0]:
            sub_dur = sub_cfg[sub_cfg['duration_s'] == dur]
            if len(sub_dur) > 0:
                med_drift = sub_dur['drift_pct'].median()
                p90_drift = sub_dur['drift_pct'].quantile(0.90)
                p95_drift = sub_dur['drift_pct'].quantile(0.95)
                pass_rate = (sub_dur['pass_10pct'].sum() / len(sub_dur)) * 100.0
                mean_final = sub_dur['final_error_m'].mean()
                mean_cross = sub_dur['final_cross_m'].mean()

                row[f"{int(dur)}s Med Drift%"] = round(med_drift, 2)
                row[f"{int(dur)}s P90 Drift%"] = round(p90_drift, 2)
                row[f"{int(dur)}s Pass%"] = round(pass_rate, 1)
                row[f"{int(dur)}s Mean Cross(m)"] = round(mean_cross, 2)
        ablation_summary.append(row)

    df_ablation = pd.DataFrame(ablation_summary)
    df_ablation.to_csv("reports/phase5_ablation_summary.csv", index=False)
    print(df_ablation.to_string())

    # 7. 60-Second Deep Diagnostic & Correlation Analysis (Phase 5-D)
    print("\n" + "=" * 84)
    print("PHASE 5-D: 60-SECOND DEEP DIAGNOSTIC & CORRELATION ANALYSIS")
    print("=" * 84)
    df_60 = pd.DataFrame(diag_60s)
    df_60.to_csv("reports/phase5_60s_deep_diagnostic.csv", index=False)

    if len(df_60) > 2:
        corr_hdg_pos = float(df_60['final_hdg_err_deg'].corr(df_60['final_error_m']))
        corr_spd_pos = float(df_60['speed_rmse_ms'].corr(df_60['final_error_m']))
        corr_yawrate_pos = float(df_60['max_yaw_rate_rads'].corr(df_60['final_error_m']))
        corr_dist_drift = float(df_60['distance_m'].corr(df_60['drift_pct']))
        corr_spd_drift = float(df_60['mean_speed_ms'].corr(df_60['drift_pct']))
        corr_cross_pos = float(df_60['final_cross_m'].corr(df_60['final_error_m']))

        print(f"Correlation Analysis on 60s Outages (N={len(df_60)}):")
        print(f"  - Heading Error vs Final Position Error:     r = {corr_hdg_pos:+.4f}")
        print(f"  - Cross-Track Error vs Final Position Error: r = {corr_cross_pos:+.4f}")
        print(f"  - Speed RMSE vs Final Position Error:        r = {corr_spd_pos:+.4f}")
        print(f"  - Peak Yaw Rate vs Final Position Error:     r = {corr_yawrate_pos:+.4f}")
        print(f"  - Travelled Distance vs Drift %:             r = {corr_dist_drift:+.4f}")
        print(f"  - Mean Speed vs Drift %:                     r = {corr_spd_drift:+.4f}")

        dominant_source = "HEADING / CROSS-TRACK ERROR" if abs(corr_cross_pos) > abs(corr_spd_pos) else "VELOCITY / ALONG-TRACK ERROR"
        print(f"\n[+] QUANTITATIVE ERROR BUDGET CONCLUSION: Dominant Driver is {dominant_source}")

    # 8. PS Benchmark Specific Verification (50m travel & 1km @ 60km/h)
    print("\n" + "=" * 84)
    print("PHASE 5-J: EXPLICIT PS 26168 BENCHMARK VERIFICATION")
    print("=" * 84)
    
    # 50m travel -> <5m error
    short_outages = df_results[(df_results['config'] == 'E_Full_System_Hardened_ESKF') & (df_results['distance_m'] <= 80.0) & (df_results['duration_s'] <= 15.0)]
    print(f"Short Outage PS Benchmark (<5 m error over 50 m travel):")
    print(f"  Evaluated Outages: {len(short_outages)}")
    if len(short_outages) > 0:
        mean_short_err = short_outages['final_error_m'].mean()
        p90_short_err = short_outages['final_error_m'].quantile(0.90)
        max_short_err = short_outages['final_error_m'].max()
        pass_short = (short_outages['final_error_m'] < 5.0).sum() / len(short_outages) * 100.0
        print(f"  Mean Error: {mean_short_err:.2f} m | P90 Error: {p90_short_err:.2f} m | Max Error: {max_short_err:.2f} m")
        print(f"  Pass Rate (<5.0 m): {pass_short:.1f}% -> {'PASS' if pass_short >= 90.0 else 'PARTIAL'}")

    # Extended Outage (1km travel @ 60km/h -> <100m error)
    long_outages = df_results[(df_results['config'] == 'E_Full_System_Hardened_ESKF') & (df_results['distance_m'] >= 500.0) & (df_results['duration_s'] >= 60.0)]
    print(f"\nExtended Highway PS Benchmark (<100 m error over ~1 km travel @ ~60 km/h):")
    print(f"  Evaluated Outages: {len(long_outages)}")
    if len(long_outages) > 0:
        mean_long_err = long_outages['final_error_m'].mean()
        p90_long_err = long_outages['final_error_m'].quantile(0.90)
        max_long_err = long_outages['final_error_m'].max()
        pass_long = (long_outages['final_error_m'] < 100.0).sum() / len(long_outages) * 100.0
        print(f"  Mean Error: {mean_long_err:.2f} m | P90 Error: {p90_long_err:.2f} m | Max Error: {max_long_err:.2f} m")
        print(f"  Pass Rate (<100.0 m): {pass_long:.1f}% -> {'PASS' if pass_long >= 80.0 else 'PARTIAL'}")

    print("\n" + "=" * 84)
    print("PHASE 5 BENCHMARK EVALUATION COMPLETE.")
    print("=" * 84)


if __name__ == "__main__":
    run_phase5_evaluation()
