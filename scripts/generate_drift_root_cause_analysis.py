#!/usr/bin/env python3
"""
scripts/generate_drift_root_cause_analysis.py
============================================
Performs complete root-cause decomposition of drift across all 158 authentic outages
under the B1/N8 autonomous baseline (Regime B: strictly zero GT speed).

Generates: DRIFT_ROOT_CAUSE_REPORT.md
"""

import os
import sys
import math
import time
import json
import joblib
import numpy as np
import pandas as pd

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.eskf_fusion import wrap_pi
from idr_pipeline.offline_osm_graph import OfflineOSMGraph
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from scripts.run_phase16_final_benchmark import (
    extract_gnss_doppler_velocity,
    estimate_preoutage_observability
)


def compute_adaptive_r_lat(v_fwd: float, omega_z: float, a_y: float, prev_r_lat: float, r_max: float = 0.16) -> float:
    R0 = 0.05**2
    a_cent = v_fwd * omega_z
    target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
    target_r = float(np.clip(target_r, R0, r_max))
    beta = 0.70
    smoothed_r = beta * prev_r_lat + (1.0 - beta) * target_r
    return float(np.clip(smoothed_r, R0, r_max))


def main():
    print("=" * 80)
    print("EXECUTING IN-DEPTH DRIFT ROOT-CAUSE ANALYSIS ACROSS 158 OUTAGES (B1/N8)")
    print("=" * 80)

    data_dir = "real_iovnbd_data"
    drives = RealIOVNBDLoader.load_all_drives(data_dir)
    catalog = build_comprehensive_outage_catalog(drives)
    print(f"Loaded {len(drives)} drives and {len(catalog)} outages.")

    csv_mapping = {
        "Driver_A_S3b": "real_iovnbd_data/Driver_A_S3b/Smartphone_2019-06-11_12-25-32.csv",
        "Vta01b": "real_iovnbd_data/Vta01b/Smartphone_2019-06-17_15-18-05.csv",
        "Vta04": "real_iovnbd_data/Vta04/Smartphone_2019-06-25_14-38-03.csv",
        "Vta03": "real_iovnbd_data/Vta03/Smartphone_2019-06-25_13-58-29.csv",
        "Vta05": "real_iovnbd_data/Vta05/Smartphone_2019-06-25_15-17-48.csv",
    }
    doppler_vels = {}
    for name, traj in drives.items():
        if name in csv_mapping and os.path.exists(csv_mapping[name]):
            doppler_vels[name] = extract_gnss_doppler_velocity(csv_mapping[name], len(traj.time))
        else:
            doppler_vels[name] = traj.gt_vel.copy()

    ckpt_path = "models/production/gbdt_speed_model_iovnbd_top45.joblib"
    ckpt = joblib.load(ckpt_path)
    gbdt_model = ckpt["model"]
    gbdt_scaler = ckpt["scaler"]
    gbdt_indices = ckpt["indices"]

    gbdt_speeds = {}
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
        gbdt_speeds[name] = v_ai

    osm_graph = OfflineOSMGraph.build_from_drives(drives)

    records = []

    for idx, seg in enumerate(catalog):
        traj = drives[seg.drive_name]
        dt = traj.dt
        i0 = int(round(seg.start_time / dt))
        i1 = int(round(seg.end_time / dt))
        i0 = max(0, min(i0, len(traj.time) - 1))
        i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
        dist_gt = max(1.0, float(seg.distance_m))
        dur_s = float(seg.duration_s)

        engine = SeamlessGNSSDeficitEngine(
            init_pos=traj.gt_pos[i0].copy(),
            init_vel=traj.gt_vel[i0].copy(),
            init_euler=np.array([float(traj.gt_roll[i0]), float(traj.gt_pitch[i0]), float(traj.gt_heading[i0])]),
            dt=dt,
            enable_heading_supervisor=True,
            enable_map_matching=True,
            osm_graph=osm_graph,
            enable_speed_calibration=True,
            enable_longitudinal_map_constraint=True
        )

        gnss_v = doppler_vels[seg.drive_name]
        ba_init, bg_init, P_init = estimate_preoutage_observability(traj, gnss_v, i0, pre_window_s=3.0)
        engine.eskf.ba = ba_init.copy()
        engine.eskf.bg = bg_init.copy()
        engine.eskf.P[6:15, 6:15] = P_init[6:15, 6:15].copy()

        prev_r_lat = 0.05**2
        steady_straight_ticks = 0

        pos_errors = []
        along_errors = []
        cross_errors = []
        vert_errors = []
        yaw_errors = []
        pitch_errors = []
        roll_errors = []
        v_fwd_errors = []
        v_lat_errors = []
        v_vert_errors = []

        omega_z_vals = []
        accel_norm_vals = []
        vib_vals = []
        bg_z_vals = []
        ba_x_vals = []

        stop_ticks = 0
        straight_ticks = 0
        turn_ticks = 0
        high_vib_ticks = 0

        for k in range(i0, i1):
            baro = float(traj.baro_alt[k])
            accel_in = traj.accel[k].copy()
            gyro_in = traj.gyro[k].copy()
            ai_spd = float(gbdt_speeds[seg.drive_name][k])

            accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel_in)
            gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro_in)
            clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)

            omega_z = float(clean_gyro_b[2])
            a_y = float(clean_accel_b[1])
            vib_e = float(np.linalg.norm(accel_b - clean_accel_b))

            omega_z_vals.append(abs(omega_z))
            accel_norm_vals.append(float(np.linalg.norm(clean_accel_b)))
            vib_vals.append(vib_e)
            bg_z_vals.append(float(engine.eskf.bg[2]))
            ba_x_vals.append(float(engine.eskf.ba[0]))

            # ESKF propagation
            engine.eskf.predict(clean_accel_b, clean_gyro_b, engine.dt)
            engine.eskf.update_altitude(baro, r_alt=0.8)

            # Longitudinal speed update
            cal_speed = engine.speed_calibrator.correct_speed(ai_spd) if engine.speed_calibrator else ai_spd
            eff_speed = 0.0 if vib_state.is_engine_idling else cal_speed
            r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_e + 1.2 * abs(omega_z))
            engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)

            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

            # Bounded NHC
            r_lat_adapt = compute_adaptive_r_lat(v_fwd, omega_z, a_y, prev_r_lat, r_max=0.16)
            prev_r_lat = r_lat_adapt
            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat_adapt), r_vert=0.05)

            # ZUPT
            if vib_state.is_engine_idling or (ai_spd < 0.2):
                engine.eskf.update_zupt(r_zupt=0.01)
                stop_ticks += 1

            # In-Flight ZARU
            is_steady_straight = (abs(omega_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0)
            if is_steady_straight:
                straight_ticks += 1
                steady_straight_ticks += 1
                if steady_straight_ticks > 8:
                    dz_w = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                    H_w = np.zeros((1, 15))
                    H_w[0, 14] = 1.0
                    engine.eskf._apply_kalman_update(dz_w, H_w, np.array([[0.005**2]]))
            else:
                steady_straight_ticks = 0
                if abs(omega_z) > 0.05:
                    turn_ticks += 1

            if vib_e > 1.5:
                high_vib_ticks += 1

            # Map heading update
            pos_curr = engine.eskf.p
            cands = osm_graph.get_candidate_edges(pos_curr, search_radius=75.0)
            if cands:
                top_cands = cands[:3]
                weights, azimuths = [], []
                for e in top_cands:
                    d2d, delta_h, snap, tang, az = osm_graph.project_point_to_edge(pos_curr, e)
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

            # Record step errors
            gt_p = traj.gt_pos[k]
            gt_v = traj.gt_vel[k]
            dp = engine.eskf.p - gt_p
            pos_err = float(np.linalg.norm(dp[:2]))
            pos_errors.append(pos_err)
            vert_errors.append(abs(float(dp[2])))

            gt_speed_k = float(traj.gt_speed[k])
            if gt_speed_k > 0.5:
                e_along = np.array([gt_v[0], gt_v[1]]) / gt_speed_k
                e_cross = np.array([-gt_v[1], gt_v[0]]) / gt_speed_k
                along_errors.append(abs(float(np.dot(dp[:2], e_along))))
                cross_errors.append(abs(float(np.dot(dp[:2], e_cross))))
            else:
                along_errors.append(pos_err)
                cross_errors.append(0.0)

            yaw_err = abs(float(wrap_pi(engine.eskf.yaw - traj.gt_heading[k])))
            pitch_err = abs(float(engine.eskf.pitch - traj.gt_pitch[k]))
            roll_err = abs(float(engine.eskf.roll - traj.gt_roll[k]))
            yaw_errors.append(yaw_err)
            pitch_errors.append(pitch_err)
            roll_errors.append(roll_err)

            # Velocity body frame error
            R_est = engine.eskf.R
            v_b_est = R_est.T @ engine.eskf.v
            v_fwd_errors.append(abs(float(v_b_est[0] - gt_speed_k)))
            v_lat_errors.append(abs(float(v_b_est[1])))
            v_vert_errors.append(abs(float(v_b_est[2])))

        final_pos_err = pos_errors[-1]
        drift_pct = (final_pos_err / dist_gt) * 100.0
        along_final = along_errors[-1]
        cross_final = cross_errors[-1]
        yaw_drift_rate_deg_s = math.degrees(yaw_errors[-1]) / max(1.0, dur_s)

        total_steps = len(pos_errors)
        rec = {
            "outage_id": seg.outage_id,
            "drive_name": seg.drive_name,
            "duration_s": dur_s,
            "distance_m": dist_gt,
            "mean_gt_speed": float(np.mean(traj.gt_speed[i0:i1])),
            "drift_pct": drift_pct,
            "final_pos_err_m": final_pos_err,
            "median_pos_err_m": float(np.median(pos_errors)),
            "max_pos_err_m": float(np.max(pos_errors)),
            "along_track_err_m": along_final,
            "cross_track_err_m": cross_final,
            "vert_err_m": vert_errors[-1],
            "yaw_err_deg": math.degrees(yaw_errors[-1]),
            "yaw_drift_rate_deg_s": yaw_drift_rate_deg_s,
            "pitch_err_deg": math.degrees(pitch_errors[-1]),
            "roll_err_deg": math.degrees(roll_errors[-1]),
            "mean_v_fwd_err_mps": float(np.mean(v_fwd_errors)),
            "mean_v_lat_err_mps": float(np.mean(v_lat_errors)),
            "mean_omega_z_rads": float(np.mean(omega_z_vals)),
            "mean_vib_energy": float(np.mean(vib_vals)),
            "final_bg_z_deg_s": math.degrees(bg_z_vals[-1]),
            "final_ba_x_mps2": ba_x_vals[-1],
            "pct_stopped": (stop_ticks / total_steps) * 100.0,
            "pct_straight": (straight_ticks / total_steps) * 100.0,
            "pct_turning": (turn_ticks / total_steps) * 100.0,
            "pct_high_vib": (high_vib_ticks / total_steps) * 100.0,
        }
        records.append(rec)

    df = pd.DataFrame(records)
    df.to_csv("reports/b1_n8_root_cause_decomposition.csv", index=False)
    print(f"Saved root-cause dataset to reports/b1_n8_root_cause_decomposition.csv ({len(df)} rows)")

    # Compute key analytical correlations
    print("\n--- Analytical Correlations with Drift % ---")
    corr_cols = [
        "distance_m", "duration_s", "along_track_err_m", "cross_track_err_m",
        "yaw_err_deg", "mean_v_fwd_err_mps", "mean_v_lat_err_mps",
        "mean_vib_energy", "pct_stopped", "pct_turning"
    ]
    for c in corr_cols:
        corr = df["drift_pct"].corr(df[c])
        print(f"  Corr(drift_pct, {c:20s}) = {corr:+.4f}")

    # Dominant error breakdown: along vs cross
    along_greater = (df["along_track_err_m"] > df["cross_track_err_m"]).sum()
    cross_greater = (df["cross_track_err_m"] >= df["along_track_err_m"]).sum()
    print(f"\nError Dominance: Along-Track dominant in {along_greater}/158 outages ({along_greater/158*100:.1f}%), Cross-Track dominant in {cross_greater}/158 outages ({cross_greater/158*100:.1f}%)")

    # Short distance analysis
    short_dist = df[df["distance_m"] < 15.0]
    print(f"\nShort Distance Outages (D < 15m): {len(short_dist)} outages")
    print(f"  Median Drift % for D < 15m: {short_dist['drift_pct'].median():.2f}%")
    print(f"  Median Drift % for D >= 15m: {df[df['distance_m'] >= 15.0]['drift_pct'].median():.2f}%")
    print(f"  Median Drift % for D >= 50m: {df[df['distance_m'] >= 50.0]['drift_pct'].median():.2f}%")


if __name__ == "__main__":
    main()
