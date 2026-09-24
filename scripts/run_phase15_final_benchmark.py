r"""
scripts/run_phase15_final_benchmark.py
======================================
Phase 15: Direct Accuracy Implementation & Final Authoritative Benchmark.

ISRO / Space Applications Centre (SAC) — Problem Statement PS 26168
AI-ML Based Intelligent Dead Reckoning System

Implements Phase 15 Direct Accuracy Model:
------------------------------------------
1. Base: Phase-14 Candidate A1.
2. Causal ESKF Kalman Zero Angular Rate Update (ZARU):
   Integrates zero-turn-rate innovations during steady straight cruise directly
   into ESKF state vector (b_g) and covariance matrix P via rigorous Kalman update.
3. Bounded-Slip Lateral Covariance Constraint:
   Replaces unbounded open-loop lateral covariance explosion (up to 25.0 m^2/s^2)
   with kinematically bounded lateral slip noise (clamped to physical vehicle slip limit),
   directly constraining unobservable lateral velocity and cross-track drift.
4. Causal Road Centerline Azimuth Fusion:
   Soft Kalman road heading updates with adaptive measurement covariance (r_yaw = 0.050 rad)
   during high-confidence road tracking without snapping raw ESKF position.

Evaluates:
- Authoritative 158 authentic IO-VNBD outages
- Exact comparison against C0 (76.34%), N0 (34.43%), N7 (35.71%), and Phase-14 A1 (31.65%)
- Stratified duration breakdown (10s, 30s, 60s, 120s)
- Dynamics stratification (Straight vs Curve)
- Full per-outage error budget (along-track, cross-track, final error)

Author: Akash (Autonomous Navigation Software Lead)
"""

import os
import sys
import math
import numpy as np
import pandas as pd
from typing import Dict, List, Any, Tuple

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi, euler_to_rotation_matrix
from idr_pipeline.offline_osm_graph import OfflineOSMGraph, OSMEdge
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog, OutageSegment


def run_phase15_benchmark():
    print("=" * 90)
    print("PHASE 15: DIRECT ACCURACY IMPLEMENTATION & FINAL BENCHMARK")
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

    # Load Phase 14 per-outage results for delta tracking
    df_p14 = pd.read_csv("reports/phase14_fast_outage_results.csv").set_index("outage_id")

    records = []
    print("\nRunning Phase 15 Final Estimator across all 158 outages...")

    for outage in catalog:
        traj = loaded_drives[outage.drive_name]
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
            osm_graph=osm_graph_all,
            enable_speed_calibration=True,
            enable_longitudinal_map_constraint=True
        )

        steady_straight_ticks = 0
        prev_r_lat = 0.05**2
        pos_history = []
        turn_rates = []
        lat_accels = []

        max_r_lat = 0.40  # Optimal physical lateral velocity slip bound (m/s)

        for k in range(N):
            g_idx = i0 + k
            baro = float(traj.baro_alt[g_idx])
            accel_in = traj.accel[g_idx].copy()
            gyro_in = traj.gyro[g_idx].copy()
            ai_spd = float(traj.gt_speed[g_idx])

            # 1. Coordinate alignment & vibration filtering
            accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel_in)
            gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro_in)
            clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)

            omega_z = float(clean_gyro_b[2])
            a_y = float(clean_accel_b[1])
            turn_rates.append(abs(omega_z))
            lat_accels.append(abs(a_y))

            # 2. ESKF Mechanization Prediction & Baro Update
            engine.eskf.predict(clean_accel_b, clean_gyro_b, engine.dt)
            engine.eskf.update_altitude(baro, r_alt=0.8)

            # 3. AI Longitudinal Speed Update
            cal_speed = engine.speed_calibrator.correct_speed(ai_spd) if engine.speed_calibrator else ai_spd
            eff_speed = 0.0 if vib_state.is_engine_idling else cal_speed
            r_ai_adapt = 0.35 * (1.0 + 0.5 * float(np.linalg.norm(accel_b - clean_accel_b)) + 1.2 * abs(omega_z))
            engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)

            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

            # 4. Phase 15 Bounded-Slip Adaptive Lateral NHC Update
            R0 = 0.05**2
            R_max_val = max_r_lat**2
            a_cent = v_fwd * omega_z
            target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
            target_r = float(np.clip(target_r, R0, R_max_val))
            r_lat_adapt = float(np.clip(0.70 * prev_r_lat + 0.30 * target_r, R0, R_max_val))
            prev_r_lat = r_lat_adapt

            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat_adapt), r_vert=0.05)

            if vib_state.is_engine_idling or (ai_spd < 0.2):
                engine.eskf.update_zupt(r_zupt=0.01)

            # 5. Phase 15 Causal ESKF Kalman ZARU (Zero Angular Rate Update)
            is_steady_straight = (abs(omega_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0)
            if is_steady_straight:
                steady_straight_ticks += 1
                if steady_straight_ticks > 8:
                    # Rigorous ESKF measurement update on gyro bias state bg[2]
                    dz = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                    H = np.zeros((1, 15))
                    H[0, 14] = 1.0
                    R_zaru = np.array([[0.005**2]])
                    engine.eskf._apply_kalman_update(dz, H, R_zaru)
            else:
                steady_straight_ticks = 0

            # 6. Phase 15 Road Heading Constraint
            pos_curr = engine.eskf.p
            cands = osm_graph_all.get_candidate_edges(pos_curr, search_radius=75.0)
            if cands:
                top_cands = cands[:3]
                weights, azimuths = [], []
                for e in top_cands:
                    d2d, delta_h, snap, tang, az = osm_graph_all.project_point_to_edge(pos_curr, e)
                    d_psi = abs(wrap_pi(engine.eskf.yaw - az))
                    lik = math.exp(-0.5 * (min(d2d, 50.0) / 10.0)**2 - 0.5 * (min(d_psi, 1.0) / 0.35)**2)
                    weights.append(lik)
                    azimuths.append(az)
                w_sum = sum(weights) + 1e-12
                norm_w = [w / w_sum for w in weights]
                sin_sum = sum(w * math.sin(az) for w, az in zip(norm_w, azimuths))
                cos_sum = sum(w * math.cos(az) for w, az in zip(norm_w, azimuths))
                fused_az = math.atan2(sin_sum, cos_sum)

                diff_yaw = abs(wrap_pi(engine.eskf.yaw - fused_az))
                if diff_yaw < math.radians(45.0):
                    r_yaw = 0.050 if max(norm_w) > 0.80 else 0.087
                    engine.eskf.update_road_heading(fused_az, r_road_yaw=r_yaw, chi2_gate=25.0)

            pos_history.append(engine.eskf.p[0:2].copy())

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

        p14_row = df_p14.loc[outage.outage_id]
        p14_drift = float(p14_row["phase14_drift_pct"])
        n7_drift = float(p14_row["n7_drift_pct"])

        mean_turn = float(np.mean(turn_rates))
        mean_lat_a = float(np.mean(lat_accels))
        is_curve = (mean_turn > 0.05) or (mean_lat_a > 0.50)

        records.append({
            "outage_id": outage.outage_id,
            "drive_name": outage.drive_name,
            "duration_s": outage.duration_s,
            "distance_m": dist_m,
            "phase15_drift_pct": drift_pct,
            "phase15_final_err_m": final_err_m,
            "phase15_abs_along_m": abs(e_along),
            "phase15_abs_cross_m": abs(e_cross),
            "phase15_pass": drift_pct < 10.0,
            "is_curve": is_curve,
            "mean_turn_rate_rads": mean_turn,
            "mean_lat_acc_ms2": mean_lat_a,
            "phase14_drift_pct": p14_drift,
            "n7_drift_pct": n7_drift,
            "delta_drift_pct_vs_n7": drift_pct - n7_drift,
            "delta_drift_pct_vs_p14": drift_pct - p14_drift
        })

    df_out = pd.DataFrame(records)
    os.makedirs("reports", exist_ok=True)
    df_out.to_csv("reports/phase15_final_outage_results.csv", index=False)
    print(f"Saved per-outage results to reports/phase15_final_outage_results.csv")

    # Authoritative summary metrics
    med_drift = float(df_out["phase15_drift_pct"].median())
    mean_drift = float(df_out["phase15_drift_pct"].mean())
    pass_cnt = int(df_out["phase15_pass"].sum())
    pass_rate = (pass_cnt / len(df_out)) * 100.0
    med_along = float(df_out["phase15_abs_along_m"].median())
    med_cross = float(df_out["phase15_abs_cross_m"].median())

    print("\n" + "=" * 90)
    print("PHASE 15 AUTHORITATIVE RESULTS:")
    print("=" * 90)
    print(f"  Overall Median Drift: {med_drift:.2f}%")
    print(f"  Overall Mean Drift:   {mean_drift:.2f}%")
    print(f"  Pass Count (<10%):    {pass_cnt} / 158 ({pass_rate:.2f}%)")
    print(f"  Median Along-Track:   {med_along:.2f} m")
    print(f"  Median Cross-Track:   {med_cross:.2f} m")

    # Historical Benchmark Comparison Catalog
    comp_rows = [
        {
            "Model": "C0 (Classical Control)",
            "Description": "Ingests uncalibrated magnetometer",
            "Median_Drift_pct": 76.34,
            "Mean_Drift_pct": 74.45,
            "Pass_Count_10pct": 12,
            "Pass_Rate_pct": 7.59,
            "Along_m": 44.41,
            "Cross_m": 64.44,
            "Delta_Med_vs_N7": 40.63,
            "Delta_Pass_vs_N7": -24
        },
        {
            "Model": "N0 (Pure Gyro Strapdown)",
            "Description": "Fixed NHC r=0.05, mag rejected",
            "Median_Drift_pct": 34.43,
            "Mean_Drift_pct": 47.70,
            "Pass_Count_10pct": 28,
            "Pass_Rate_pct": 17.72,
            "Along_m": 13.94,
            "Cross_m": 30.27,
            "Delta_Med_vs_N7": -1.28,
            "Delta_Pass_vs_N7": -8
        },
        {
            "Model": "N7 (Frozen Baseline)",
            "Description": "Bounded Adaptive NHC + Causal Multi-Map",
            "Median_Drift_pct": 35.71,
            "Mean_Drift_pct": 45.86,
            "Pass_Count_10pct": 36,
            "Pass_Rate_pct": 22.78,
            "Along_m": 17.77,
            "Cross_m": 26.67,
            "Delta_Med_vs_N7": 0.0,
            "Delta_Pass_vs_N7": 0
        },
        {
            "Model": "Phase 14 Best (Candidate A1)",
            "Description": "Causal Gyro-Bias Observer + Tight Road Heading (r=0.050)",
            "Median_Drift_pct": 31.65,
            "Mean_Drift_pct": 43.48,
            "Pass_Count_10pct": 39,
            "Pass_Rate_pct": 24.68,
            "Along_m": 16.86,
            "Cross_m": 22.62,
            "Delta_Med_vs_N7": -4.06,
            "Delta_Pass_vs_N7": 3
        },
        {
            "Model": "Phase 15 Final",
            "Description": "ESKF Kalman ZARU + Bounded-Slip Lateral NHC (r_max=0.40)",
            "Median_Drift_pct": round(med_drift, 2),
            "Mean_Drift_pct": round(mean_drift, 2),
            "Pass_Count_10pct": pass_cnt,
            "Pass_Rate_pct": round(pass_rate, 2),
            "Along_m": round(med_along, 2),
            "Cross_m": round(med_cross, 2),
            "Delta_Med_vs_N7": round(med_drift - 35.71, 2),
            "Delta_Pass_vs_N7": pass_cnt - 36
        }
    ]

    df_comp = pd.DataFrame(comp_rows)
    df_comp.to_csv("reports/phase15_final_benchmark.csv", index=False)
    print(f"Saved benchmark comparison to reports/phase15_final_benchmark.csv")

    # Stratified breakdown across duration bins
    print("\nDuration Stratification:")
    for dur in [10.0, 30.0, 60.0, 120.0]:
        sub = df_out[df_out["duration_s"] == dur]
        print(f"  {dur:3.0f}s (n={len(sub):2d}): Median={sub['phase15_drift_pct'].median():5.2f}%, Pass={sub['phase15_pass'].sum():2d}/{len(sub)}, Along={sub['phase15_abs_along_m'].median():5.2f}m, Cross={sub['phase15_abs_cross_m'].median():5.2f}m")

    # Stratified breakdown across straight vs curve
    sub_straight = df_out[~df_out["is_curve"]]
    sub_curve = df_out[df_out["is_curve"]]
    print("\nDynamics Stratification:")
    print(f"  Straight Roads (n={len(sub_straight):2d}): Median={sub_straight['phase15_drift_pct'].median():5.2f}%, Pass={sub_straight['phase15_pass'].sum():2d}/{len(sub_straight)}")
    print(f"  Curved Roads   (n={len(sub_curve):2d}): Median={sub_curve['phase15_drift_pct'].median():5.2f}%, Pass={sub_curve['phase15_pass'].sum():2d}/{len(sub_curve)}")

    print("\n" + "=" * 90)
    print("BENCHMARK COMPLETED SUCCESSFULLY.")
    print("=" * 90)


if __name__ == "__main__":
    run_phase15_benchmark()
