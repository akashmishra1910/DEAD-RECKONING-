r"""
scripts/run_phase10b_residual_investigation.py
==============================================
Phase 10B: Residual Error Isolation & Causal Attribution.

ISRO / Space Applications Centre (SAC) — Problem Statement PS 26168
AI-ML Based Intelligent Dead Reckoning System

Implements:
-----------
Experiment A: Perfect Gravity / Grade Compensation
  - A0: C1 (Perfect Attitude Oracle baseline, 32.74% drift)
  - A1: C1 + Perfect Ground-Truth Gravity/Kinematic Accel Compensation

Experiment B: Non-Holonomic Constraint (NHC) Ablation
  - B0: C1 + Normal NHC (r_lat=0.05, r_vert=0.05)
  - B1: C1 + NHC Disabled
  - B2: C1 + True Lateral/Vertical Velocity Constraint (v_y^b = v_{y,gt}^b, v_z^b = v_{z,gt}^b)

Experiment C: Map Hypothesis Analysis
  - C0: Baseline Authoritative Estimator (76.34%)
  - C8: Causal Road Heading Aiding (69.22%)
  - C_multi: Multi-Hypothesis Causal Map Tracker (Top-3 Bayesian weighted)

Experiment D: Combined Oracle Matrix
  - D0: C1 (Attitude Oracle alone)
  - D1: C1 + Gravity Oracle
  - D2: C1 + True NHC Correction
  - D3: C1 + Gravity + True NHC
  - D4: C1 + Gravity + True NHC + Multi-Hypothesis Map

Author: Autonomous Navigation Software Lead
"""

import os
import sys
import math
import copy
import numpy as np
import pandas as pd
from typing import Dict, List, Any, Tuple, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi, euler_to_rotation_matrix
from idr_pipeline.offline_osm_graph import OfflineOSMGraph, OSMEdge
from idr_pipeline.map_matcher_3d import Causal3DMapMatcher, MapMatchConfidence
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine, NavigationMode
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog, OutageSegment


def run_single_outage_10b(
    traj: TrajectoryData,
    outage: OutageSegment,
    osm_graph: OfflineOSMGraph,
    exp_code: str = "A0"
) -> Optional[Dict[str, Any]]:
    dt = traj.dt
    i0 = int(round(outage.start_time / dt))
    i1 = int(round(outage.end_time / dt))
    i0 = max(0, min(i0, len(traj.time) - 1))
    i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
    N = i1 - i0

    p0 = traj.gt_pos[i0].copy()
    v0 = traj.gt_vel[i0].copy()
    yaw0 = float(traj.gt_heading[i0])
    pitch0 = float(traj.gt_pitch[i0])
    roll0 = float(traj.gt_roll[i0])

    engine = SeamlessGNSSDeficitEngine(
        init_pos=p0,
        init_vel=v0,
        init_euler=np.array([roll0, pitch0, yaw0]),
        dt=dt,
        enable_heading_supervisor=True,
        enable_map_matching=True,
        osm_graph=osm_graph,
        enable_speed_calibration=True,
        enable_longitudinal_map_constraint=True
    )

    pos_history = []
    heading_history = []
    heading_err_history = []
    speed_err_history = []
    lateral_acc_history = []
    turn_rate_history = []
    map_cand_counts = []
    map_selected_edges = []
    map_chi2_values = []

    for k in range(N):
        g_idx = i0 + k
        t = float(traj.time[g_idx])
        baro = float(traj.baro_alt[g_idx])
        mag_raw = traj.mag[g_idx] if traj.mag is not None else None

        accel_in = traj.accel[g_idx].copy()
        gyro_in = traj.gyro[g_idx].copy()
        ai_spd = float(traj.gt_speed[g_idx])

        # Kinematic turn rate and lateral accel tracking
        turn_rate = abs(float(gyro_in[2]))
        turn_rate_history.append(turn_rate)
        lat_acc = abs(float(accel_in[1]))
        lateral_acc_history.append(lat_acc)

        # ---------------------------------------------------------------------
        # EXPERIMENT A / D: Perfect Gravity / Kinematic Acceleration Oracle
        # ---------------------------------------------------------------------
        if exp_code in ["A1", "D1", "D3", "D4"]:
            # True kinematic acceleration in navigation frame: a_n = dv_gt / dt
            if k < N - 1:
                dv_n = (traj.gt_vel[g_idx + 1] - traj.gt_vel[g_idx]) / dt
            elif k > 0:
                dv_n = (traj.gt_vel[g_idx] - traj.gt_vel[g_idx - 1]) / dt
            else:
                dv_n = np.zeros(3)

            # Convert to body specific force: f_b = R_b^n^T (a_n - g_n)
            R_gt = euler_to_rotation_matrix(float(traj.gt_roll[g_idx]), float(traj.gt_pitch[g_idx]), float(traj.gt_heading[g_idx]))
            g_n = np.array([0.0, 0.0, -9.80665])
            f_b_true = R_gt.T @ (dv_n - g_n)

            # Transform back to phone frame for engine ingestion
            R_p2b = engine.alignment_engine.R_p2b
            accel_in = R_p2b.T @ f_b_true

        # ---------------------------------------------------------------------
        # STEP PROPAGATION
        # ---------------------------------------------------------------------
        telemetry = engine.process_step(
            time_s=t,
            accel_raw_p=accel_in,
            gyro_raw_p=gyro_in,
            baro_alt=baro,
            gnss_pos=None,
            gnss_valid=False,
            ai_speed=ai_spd,
            gnss_vel=None,
            mag_raw_p=mag_raw,
            enable_road_constraints=(exp_code not in ["B1"])
        )

        # ---------------------------------------------------------------------
        # ATTITUDE RESET (For all C1-based experiments)
        # ---------------------------------------------------------------------
        if exp_code in ["A0", "A1", "B0", "B1", "B2", "D0", "D1", "D2", "D3", "D4"]:
            gt_y = float(traj.gt_heading[g_idx])
            gt_p = float(traj.gt_pitch[g_idx])
            gt_r = float(traj.gt_roll[g_idx])
            engine.eskf.roll = gt_r
            engine.eskf.pitch = gt_p
            engine.eskf.yaw = gt_y
            engine.eskf.R = euler_to_rotation_matrix(gt_r, gt_p, gt_y)

        # ---------------------------------------------------------------------
        # EXPERIMENT B / D: NHC Ablation and True Lateral Velocity Constraint
        # ---------------------------------------------------------------------
        if exp_code == "B1":
            # NHC is disabled in B1 (we overwrite / do not constrain lateral velocity)
            pass
        elif exp_code in ["B2", "D2", "D3", "D4"]:
            # Update lateral and vertical body velocity to true kinematic ground truth
            # v_b_true = R_gt^T @ v_n_gt
            R_gt = engine.eskf.R
            v_b_gt = R_gt.T @ traj.gt_vel[g_idx]
            # True lateral velocity update: v_y^b = v_b_gt[1], v_z^b = v_b_gt[2]
            H_lat = np.zeros((2, 15))
            H_lat[0, 3:6] = R_gt[:, 1]  # Left unit vector
            H_lat[1, 3:6] = R_gt[:, 2]  # Up unit vector
            z_lat = np.array([v_b_gt[1], v_b_gt[2]])
            z_pred = np.array([float(R_gt[:, 1] @ engine.eskf.v), float(R_gt[:, 2] @ engine.eskf.v)])
            dz = z_lat - z_pred
            R_nhc_true = np.eye(2) * (0.02**2)
            S = H_lat @ engine.eskf.P @ H_lat.T + R_nhc_true
            K = engine.eskf.P @ H_lat.T @ np.linalg.inv(S)
            dx = K @ dz
            engine.eskf.p += dx[0:3]
            engine.eskf.v += dx[3:6]
            engine.eskf.P = (np.eye(15) - K @ H_lat) @ engine.eskf.P

        # ---------------------------------------------------------------------
        # EXPERIMENT C / D: Causal Multi-Hypothesis Map Tracking
        # ---------------------------------------------------------------------
        if exp_code in ["C8", "C_multi", "D4"]:
            pos_curr = engine.eskf.p
            cands = osm_graph.get_candidate_edges(pos_curr, search_radius=75.0)
            map_cand_counts.append(len(cands))
            if cands:
                if exp_code in ["C_multi", "D4"] and len(cands) > 1:
                    # Multi-hypothesis Bayesian weighting across top-3 candidate edges
                    top_cands = cands[:3]
                    weights = []
                    azimuths = []
                    for e in top_cands:
                        d2d, delta_h, snap, tang, az = osm_graph.project_point_to_edge(pos_curr, e)
                        d_psi = abs(wrap_pi(engine.eskf.yaw - az))
                        # Gaussian likelihood: L(edge) = exp(-0.5 * (d2d/10)^2 - 0.5 * (d_psi/0.35)^2)
                        lik = math.exp(-0.5 * (min(d2d, 50.0) / 10.0)**2 - 0.5 * (min(d_psi, 1.0) / 0.35)**2)
                        weights.append(lik)
                        azimuths.append(az)
                    w_sum = sum(weights) + 1e-12
                    norm_weights = [w / w_sum for w in weights]
                    # Compute circular mean azimuth
                    sin_sum = sum(w * math.sin(az) for w, az in zip(norm_weights, azimuths))
                    cos_sum = sum(w * math.cos(az) for w, az in zip(norm_weights, azimuths))
                    fused_az = math.atan2(sin_sum, cos_sum)
                    map_selected_edges.append(top_cands[0].id)
                    d_psi_fused = abs(wrap_pi(engine.eskf.yaw - fused_az))
                    if d_psi_fused < math.radians(45.0):
                        engine.eskf.update_road_heading(fused_az, r_road_yaw=0.087, chi2_gate=25.0)
                else:
                    best_edge = cands[0]
                    map_selected_edges.append(best_edge.id)
                    d2d, delta_h, snap, tang, az = osm_graph.project_point_to_edge(pos_curr, best_edge)
                    d_psi = abs(wrap_pi(engine.eskf.yaw - az))
                    if d_psi < math.radians(45.0):
                        engine.eskf.update_road_heading(az, r_road_yaw=0.087, chi2_gate=25.0)

        curr_pos = engine.eskf.p[0:2].copy()
        pos_history.append(curr_pos)
        curr_yaw = float(engine.eskf.yaw)
        heading_history.append(curr_yaw)

        gt_yaw_k = float(traj.gt_heading[g_idx])
        heading_err_history.append(abs(wrap_pi(curr_yaw - gt_yaw_k)))
        curr_spd = float(np.linalg.norm(engine.eskf.v[0:2]))
        speed_err_history.append(abs(curr_spd - float(traj.gt_speed[g_idx])))

    if len(pos_history) == 0:
        return None

    final_pos = pos_history[-1]
    final_gt = traj.gt_pos[i1 - 1, 0:2]
    err_vec = final_pos - final_gt
    final_err_m = float(np.linalg.norm(err_vec))
    dist_m = max(1.0, float(outage.distance_m))
    drift_pct = (final_err_m / dist_m) * 100.0

    gt_final_yaw = float(traj.gt_heading[i1 - 1])
    t_gt = np.array([math.sin(gt_final_yaw), math.cos(gt_final_yaw)])
    n_gt = np.array([math.cos(gt_final_yaw), -math.sin(gt_final_yaw)])
    e_along = float(np.dot(err_vec, t_gt))
    e_cross = float(np.dot(err_vec, n_gt))

    mean_turn_rate = float(np.mean(turn_rate_history))
    mean_lat_acc = float(np.mean(lateral_acc_history))
    is_curve = mean_turn_rate > 0.05 or mean_lat_acc > 0.50

    return {
        "outage_id": outage.outage_id,
        "drive_name": outage.drive_name,
        "duration_s": outage.duration_s,
        "distance_m": dist_m,
        "experiment": exp_code,
        "final_error_m": final_err_m,
        "drift_pct": drift_pct,
        "along_track_error_m": e_along,
        "cross_track_error_m": e_cross,
        "abs_along_track_m": abs(e_along),
        "abs_cross_track_m": abs(e_cross),
        "mean_heading_err_deg": float(np.mean(heading_err_history)) * (180.0 / math.pi),
        "mean_speed_err_mps": float(np.mean(speed_err_history)),
        "mean_turn_rate_rads": mean_turn_rate,
        "mean_lateral_acc_ms2": mean_lat_acc,
        "is_curve_regime": is_curve,
        "pass_10pct": drift_pct < 10.0
    }


def run_phase10b_investigation():
    print("=" * 90)
    print("PHASE 10B: RESIDUAL ERROR ISOLATION & CAUSAL ATTRIBUTION")
    print("=" * 90)

    drives_info = [
        ("Driver_A_S3b", os.path.join("real_iovnbd_data", "Driver_A_S3b", "S-S3b.csv"), os.path.join("real_iovnbd_data", "Driver_A_S3b", "V-S3b.csv")),
        ("Vta01b", os.path.join("real_iovnbd_data", "Vta01b", "S-Vta1b.csv"), os.path.join("real_iovnbd_data", "Vta01b", "V-Vta1b.csv")),
        ("Vta04", os.path.join("real_iovnbd_data", "Vta04", "S-Vta4.csv"), os.path.join("real_iovnbd_data", "Vta04", "V-Vta4.csv")),
        ("Vta03", os.path.join("real_iovnbd_data", "Vta03", "S-Vta3.csv"), os.path.join("real_iovnbd_data", "Vta03", "V-Vta3.csv")),
        ("Vta05", os.path.join("real_iovnbd_data", "Vta05", "S-Vta5.csv"), os.path.join("real_iovnbd_data", "Vta05", "V-Vta5.csv")),
    ]

    loaded_drives = {}
    for name, s_path, v_path in drives_info:
        if os.path.exists(s_path) and os.path.exists(v_path):
            traj = RealIOVNBDLoader.load_drive(s_path, v_path, name=name)
            loaded_drives[name] = traj
            print(f"  Loaded drive {name:15s}: {len(traj.time):5d} samples, duration {traj.time[-1]:.1f} s")

    osm_graph_all = OfflineOSMGraph.build_from_drives(loaded_drives)
    catalog = build_comprehensive_outage_catalog(loaded_drives)
    print(f"\nTotal authentic outages in catalog: {len(catalog)}")

    experiments = [
        # Baseline & Attitudinal Reference
        ("C0",      "Baseline Authoritative Estimator"),
        ("A0",      "C1 Perfect Attitude Oracle (Reference Baseline)"),
        # Experiment A: Gravity / Grade Compensation
        ("A1",      "C1 + Perfect Gravity/Grade Compensation"),
        # Experiment B: NHC Ablation
        ("B0",      "C1 + Normal NHC"),
        ("B1",      "C1 + NHC Disabled"),
        ("B2",      "C1 + True Lateral/Vertical Velocity Constraint"),
        # Experiment C: Map Hypotheses
        ("C8",      "Causal Single-Hypothesis Map Aiding"),
        ("C_multi", "Causal Multi-Hypothesis Map Aiding"),
        # Experiment D: Combined Oracles
        ("D0",      "C1 Attitude Oracle Alone"),
        ("D1",      "C1 + Gravity Oracle"),
        ("D2",      "C1 + True NHC Correction"),
        ("D3",      "C1 + Gravity + True NHC"),
        ("D4",      "C1 + Gravity + True NHC + Multi-Hypothesis Map")
    ]

    all_records = []
    print("\n" + "=" * 90)
    print("RUNNING PHASE 10B EXPERIMENT MATRIX ACROSS 158 OUTAGES")
    print("=" * 90)

    for exp_code, exp_desc in experiments:
        print(f"  Executing {exp_code:<8s}: {exp_desc}...")
        for outage in catalog:
            if outage.drive_name in loaded_drives:
                traj = loaded_drives[outage.drive_name]
                res = run_single_outage_10b(traj, outage, osm_graph_all, exp_code=exp_code)
                if res is not None:
                    all_records.append(res)

    df_records = pd.DataFrame(all_records)
    os.makedirs("reports", exist_ok=True)
    df_records.to_csv("reports/phase10b_trajectory_samples.csv", index=False)
    print(f"\nSaved all trajectory records to reports/phase10b_trajectory_samples.csv")

    # Generate Stratified Summary
    summary_rows = []
    dur_groups = [10.0, 30.0, 60.0, 120.0, "Overall"]

    for exp_code, exp_desc in experiments:
        for dur in dur_groups:
            sub = df_records[df_records["experiment"] == exp_code]
            if dur != "Overall":
                sub = sub[sub["duration_s"] == dur]

            if len(sub) == 0:
                continue

            n = len(sub)
            med_drift = float(sub["drift_pct"].median())
            mean_drift = float(sub["drift_pct"].mean())
            p90_drift = float(np.percentile(sub["drift_pct"], 90))
            p95_drift = float(np.percentile(sub["drift_pct"], 95))
            max_drift = float(sub["drift_pct"].max())
            pass_cnt = int(sub["pass_10pct"].sum())
            pass_rate = (pass_cnt / n) * 100.0

            med_along = float(sub["abs_along_track_m"].median())
            med_cross = float(sub["abs_cross_track_m"].median())
            med_err_m = float(sub["final_error_m"].median())

            summary_rows.append({
                "Experiment": exp_code,
                "Description": exp_desc,
                "Duration": f"{int(dur)}s" if isinstance(dur, float) else dur,
                "N": n,
                "Median_Drift_pct": med_drift,
                "Mean_Drift_pct": mean_drift,
                "P90_Drift_pct": p90_drift,
                "P95_Drift_pct": p95_drift,
                "Max_Drift_pct": max_drift,
                "Pass_Count_10pct": pass_cnt,
                "Pass_Rate_pct": pass_rate,
                "Median_Final_Err_m": med_err_m,
                "Median_Along_m": med_along,
                "Median_Cross_m": med_cross
            })

    df_summary = pd.DataFrame(summary_rows)
    df_summary.to_csv("reports/phase10b_residual_ablation.csv", index=False)
    print(f"Saved complete ablation summary to reports/phase10b_residual_ablation.csv")

    print("\n" + "=" * 90)
    print("PHASE 10B RESIDUAL ERROR ISOLATION SUMMARY (OVERALL ALL 158 OUTAGES)")
    print("=" * 90)
    print(f"{'Exp':<8s} | {'Median Drift':<12s} | {'Mean Drift':<10s} | {'P90 Drift':<10s} | {'P95 Drift':<10s} | {'Pass (<10%)':<12s} | {'Along m':<8s} | {'Cross m':<8s}")
    print("-" * 96)
    for r in summary_rows:
        if r["Duration"] == "Overall":
            print(f"{r['Experiment']:<8s} | {r['Median_Drift_pct']:11.2f}% | {r['Mean_Drift_pct']:9.2f}% | {r['P90_Drift_pct']:9.2f}% | {r['P95_Drift_pct']:9.2f}% | {r['Pass_Count_10pct']:2d}/{r['N']:2d} ({r['Pass_Rate_pct']:5.1f}%) | {r['Median_Along_m']:7.2f}m | {r['Median_Cross_m']:7.2f}m")
    print("-" * 96)


if __name__ == "__main__":
    run_phase10b_investigation()
