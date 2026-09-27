#!/usr/bin/env python3
"""
scripts/generate_part3_error_budget.py

Generates the exhaustive per-outage physical error budget and failure taxonomy
for all 158 outages of the authentic IO-VNBD dataset under the frozen P2-M7 baseline.

Outputs:
  - reports/part3_authoritative_158_outage_results.csv
  - reports/part3_error_budget.csv
  - reports/part3_failure_modes.csv
"""

import os
import sys
import math
import time
from typing import Dict, List, Any
import numpy as np
import pandas as pd

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi
from idr_pipeline.offline_osm_graph import OfflineOSMGraph
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.map_matcher_3d import Causal3DMapMatcher
from idr_pipeline.integrity_manager import IntegrityManager
from scripts.run_phase16_final_benchmark import (
    extract_gnss_doppler_velocity,
    estimate_preoutage_observability
)
import joblib


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
    print("PART 3 EXHAUSTIVE ERROR BUDGET & FAILURE MODE DECOMPOSITION")
    print("Authentic 158-Outage IO-VNBD Dataset | Frozen P2-M7 Control Baseline")
    print("=" * 80)

    os.makedirs("reports", exist_ok=True)

    # 1. Load drives & catalog
    data_dir = "real_iovnbd_data"
    drives = RealIOVNBDLoader.load_all_drives(data_dir)
    catalog = build_comprehensive_outage_catalog(drives)
    print(f"Loaded {len(drives)} drives and {len(catalog)} outages.")

    # 2. Extract Doppler velocity & load GBDT
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

    # 3. Load GBDT speed model and precompute forward speeds
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

    # 3. Build authentic topological road graph
    osm_graph = OfflineOSMGraph.build_from_drives(drives)

    # 4. Execute 158 outages with fine-grained metric capture
    authoritative_results = []
    error_budget_rows = []
    failure_mode_rows = []

    for seg in catalog:
        drive_name = seg.drive_name
        traj = drives[drive_name]
        dt = traj.dt

        i0 = int(round(seg.start_time / dt))
        i1 = int(round(seg.end_time / dt))
        i0 = max(0, min(i0, len(traj.time) - 1))
        i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
        N_pts = i1 - i0
        dist_gt = max(1.0, float(seg.distance_m))

        gnss_v = doppler_vels[drive_name]
        ba_init, bg_init, P_init = estimate_preoutage_observability(traj, gnss_v, i0, pre_window_s=3.0)

        p_init = traj.gt_pos[i0].copy()
        v_init = traj.gt_vel[i0].copy()
        yaw_init = float(traj.gt_heading[i0])
        euler_init = np.array([float(traj.gt_roll[i0]), float(traj.gt_pitch[i0]), yaw_init])

        engine = SeamlessGNSSDeficitEngine(
            init_pos=p_init,
            init_vel=v_init,
            init_euler=euler_init,
            dt=dt,
            enable_heading_supervisor=True,
            enable_map_matching=False,
            osm_graph=osm_graph
        )
        engine.eskf.ba = ba_init.copy()
        engine.eskf.bg = bg_init.copy()
        engine.eskf.P[6:15, 6:15] = P_init[6:15, 6:15].copy()

        map_matcher = Causal3DMapMatcher(
            graph=osm_graph,
            sigma_d=5.0,
            sigma_h=1.8,
            sigma_psi=0.30,
            beta=5.0,
            elevation_barrier=4.0
        )
        integrity_mgr = IntegrityManager()

        prev_r_lat = 0.05**2
        steady_straight_ticks = 0
        cur_outage_corrections = 0
        cur_outage_rejections = 0
        cur_outage_faults = 0

        baro_offset = float(traj.baro_alt[0]) - float(traj.gt_pos[0, 2]) if hasattr(traj, 'baro_alt') else 0.0

        # Fine-grained timeseries collectors for error budget
        speed_errors = []
        yaw_rates = []
        lat_accels = []
        vib_energies = []
        map_confs = []
        ambiguity_count = 0
        stop_ticks = 0
        turn_ticks = 0

        for k in range(i0, i1):
            t_k = float(traj.time[k])
            accel_raw = traj.accel[k]
            gyro_raw = traj.gyro[k]
            baro_alt = (float(traj.baro_alt[k]) - baro_offset) if hasattr(traj, 'baro_alt') else float(traj.gt_pos[k, 2])

            accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel_raw)
            gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro_raw)

            clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)
            vib_energy = float(np.linalg.norm(accel_b - clean_accel_b))

            ai_spd = float(gbdt_speeds[drive_name][k])
            eff_speed = 0.0 if vib_state.is_engine_idling else ai_spd

            # Record stats
            gt_spd_k = float(traj.gt_speed[k])
            speed_errors.append(eff_speed - gt_spd_k)
            yaw_rates.append(abs(float(clean_gyro_b[2])))
            lat_accels.append(abs(float(clean_accel_b[1])))
            vib_energies.append(vib_energy)
            if gt_spd_k < 0.5:
                stop_ticks += 1
            if abs(float(clean_gyro_b[2])) > 0.10:
                turn_ticks += 1

            # Predict
            engine.eskf.predict(clean_accel_b, clean_gyro_b, dt)
            engine.eskf.update_altitude(baro_alt, r_alt=0.8)

            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))
            omega_z = float(abs(clean_gyro_b[2]))
            a_y = float(clean_accel_b[1])

            # AI Speed Update
            r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_energy + 1.2 * omega_z)
            if eff_speed < 0.5 and vib_energy < 0.20:
                r_ai_adapt = 0.05
            speed_innov = eff_speed - v_fwd
            speed_var = float(engine.eskf.P[3, 3] + r_ai_adapt**2)
            engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)

            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

            # NHC
            r_max_current = 0.09 if (omega_z < 0.02 and abs(a_y) < 0.20) else 0.16
            r_lat = compute_adaptive_r_lat(v_fwd, omega_z, a_y, prev_r_lat, r_max=r_max_current)
            prev_r_lat = r_lat
            r_vert = 0.05**2
            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat), r_vert=math.sqrt(r_vert))

            # ZUPT / ZARU
            if eff_speed < 0.20 and abs(clean_accel_b[0]) < 0.15 and omega_z < 0.015:
                engine.eskf.update_zupt(r_zupt=0.01)
                steady_straight_ticks = 0
            elif omega_z < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0:
                steady_straight_ticks += 1
                if steady_straight_ticks > 8:
                    dz = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                    H = np.zeros((1, 15))
                    H[0, 14] = 1.0
                    engine.eskf._apply_kalman_update(dz, H, np.array([[0.005**2]]))
            else:
                steady_straight_ticks = 0

            # 3D Map Matching Step
            pos_curr = engine.eskf.p
            cands = osm_graph.get_candidate_edges(pos_curr, search_radius=75.0)

            road_hdg_nis = None
            road_cross_nis = None

            if cands:
                hypotheses = []
                best_edge = None
                best_d2d = 999.0
                best_dh = 0.0
                best_az = 0.0
                best_snap = None
                best_tang = None

                weights = []
                results = []
                for e in cands[:3]:
                    d2d, delta_h, snap, tang, az = osm_graph.project_point_to_edge(pos_curr, e)
                    d_psi = abs(wrap_pi(engine.eskf.yaw - az))

                    log_p_dist = -0.5 * (min(d2d, 50.0) / 5.0)**2
                    log_p_psi = -0.5 * (min(d_psi, 1.0) / 0.30)**2
                    log_p_alt = -50.0 - 5.0 * (delta_h - 4.0) if delta_h > 4.0 else -0.5 * (delta_h / 1.8)**2

                    log_p_trans = 0.0
                    if map_matcher.last_matched_edge_id is not None and e.id != map_matcher.last_matched_edge_id:
                        is_connected = (map_matcher.last_matched_edge_id in osm_graph.edges) and (e.id in osm_graph.adjacency.get(osm_graph.edges[map_matcher.last_matched_edge_id].target_id, []))
                        log_p_trans = 0.0 if is_connected else -15.0

                    total_log_p = log_p_dist + log_p_psi + log_p_alt + log_p_trans
                    weights.append(total_log_p)
                    results.append((e, d2d, delta_h, snap, tang, az))

                max_lp = max(weights)
                exp_w = [math.exp(w - max_lp) for w in weights]
                sum_exp_w = sum(exp_w) + 1e-12
                posteriors = [w / sum_exp_w for w in exp_w]

                best_idx = int(np.argmax(posteriors))
                best_edge, best_d2d, best_dh, best_snap, best_tang, best_az = results[best_idx]
                best_lp = weights[best_idx]
                runner_up_lp = sorted(weights, reverse=True)[1] if len(weights) > 1 else -999.0
                score_margin = float(best_lp - runner_up_lp)
                if score_margin < 1.5:
                    ambiguity_count += 1

                    map_matcher.last_matched_edge_id = best_edge.id

                    d_psi_best = abs(wrap_pi(engine.eskf.yaw - best_az))
                    yaw_var = float(engine.eskf.P[8, 8])
                    continuous_conf = integrity_mgr.compute_continuous_map_confidence(
                        d_2d=best_d2d,
                        delta_h=best_dh,
                        d_psi=d_psi_best,
                        score_margin=score_margin,
                        yaw_variance=yaw_var,
                        consecutive_matches=map_matcher.consecutive_valid_steps,
                        elevation_barrier=4.0
                    )
                    map_confs.append(continuous_conf)

                    if continuous_conf >= 0.35:
                        r_cross = 1.75 if continuous_conf >= 0.70 else 3.50
                        dz_cross = (pos_curr[0] - best_snap[0]) * (-best_tang[1]) + (pos_curr[1] - best_snap[1]) * best_tang[0]
                        S_cross = float(engine.eskf.P[0, 0] + r_cross**2)
                        road_cross_nis = (dz_cross**2) / S_cross

                        succ_p, gamma_p = engine.eskf.update_road_position(best_snap, best_tang, r_cross=r_cross, chi2_gate=16.0)
                        if succ_p:
                            cur_outage_corrections += 1
                        else:
                            cur_outage_rejections += 1

                        if d_psi_best < math.radians(25.0):
                            r_hdg = 0.087 if continuous_conf >= 0.70 else 0.174
                            dz_hdg = wrap_pi(engine.eskf.yaw - best_az)
                            S_hdg = float(engine.eskf.P[8, 8] + r_hdg**2)
                            road_hdg_nis = (dz_hdg**2) / S_hdg
                            succ_h, gamma_h = engine.eskf.update_road_heading(best_az, r_road_yaw=r_hdg, chi2_gate=9.0)
                            if succ_h:
                                cur_outage_corrections += 1
                            else:
                                cur_outage_rejections += 1

            rep = integrity_mgr.evaluate_step(
                time_s=t_k,
                in_outage=True,
                gnss_valid=False,
                P_cov=engine.eskf.P,
                map_match_result=None,
                speed_innovation_m_s=speed_innov,
                speed_innov_var=speed_var,
                road_hdg_nis=road_hdg_nis,
                road_cross_nis=road_cross_nis,
                vib_energy=vib_energy,
                turn_rate=omega_z
            )
            if bool(rep.fault_flags):
                cur_outage_faults += 1

        # Outage Final Error Calculation
        gt_p = traj.gt_pos[i1 - 1]
        gt_v = traj.gt_vel[i1 - 1]
        dp = engine.eskf.p - gt_p
        pos_err = float(np.linalg.norm(dp[:2]))
        vert_err = abs(float(dp[2]))
        drift_pct = (pos_err / dist_gt) * 100.0

        gt_spd_end = float(traj.gt_speed[i1 - 1])
        if gt_spd_end > 0.5:
            e_along = np.array([gt_v[0], gt_v[1]]) / gt_spd_end
            e_cross = np.array([-gt_v[1], gt_v[0]]) / gt_spd_end
            along_err = abs(float(np.dot(dp[:2], e_along)))
            cross_err = abs(float(np.dot(dp[:2], e_cross)))
        else:
            along_err = pos_err
            cross_err = 0.0

        yaw_err = math.degrees(abs(float(wrap_pi(engine.eskf.yaw - traj.gt_heading[i1 - 1]))))

        authoritative_results.append({
            "outage_id": seg.outage_id,
            "drive_name": drive_name,
            "duration_s": seg.duration_s,
            "dist_gt_m": dist_gt,
            "drift_pct": drift_pct,
            "pos_err_m": pos_err,
            "cross_err_m": cross_err,
            "along_err_m": along_err,
            "vert_err_m": vert_err,
            "yaw_err_deg": yaw_err,
            "corrections": cur_outage_corrections,
            "rejections": cur_outage_rejections,
            "faults": cur_outage_faults,
            "passed_le_10pct": int(drift_pct <= 10.0)
        })

        # Error Budget statistics
        mean_speed = float(np.mean(traj.gt_speed[i0:i1]))
        speed_rmse = float(np.sqrt(np.mean(np.array(speed_errors)**2)))
        speed_bias = float(np.mean(speed_errors))
        yaw_rate_mean = float(np.mean(yaw_rates))
        lat_accel_mean = float(np.mean(lat_accels))
        vib_mean = float(np.mean(vib_energies))
        stop_sec = stop_ticks * dt
        mean_conf = float(np.mean(map_confs)) if map_confs else 0.0

        error_budget_rows.append({
            "outage_id": seg.outage_id,
            "drive_name": drive_name,
            "duration_s": seg.duration_s,
            "dist_gt_m": dist_gt,
            "pos_err_m": pos_err,
            "along_err_m": along_err,
            "cross_err_m": cross_err,
            "vert_err_m": vert_err,
            "yaw_err_deg": yaw_err,
            "drift_pct": drift_pct,
            "mean_speed_m_s": mean_speed,
            "speed_rmse_m_s": speed_rmse,
            "speed_bias_m_s": speed_bias,
            "yaw_rate_mean_rad_s": yaw_rate_mean,
            "lat_accel_mean_m_s2": lat_accel_mean,
            "vib_energy_mean": vib_mean,
            "stop_duration_s": stop_sec,
            "turn_ticks": turn_ticks,
            "ambiguous_epochs": ambiguity_count,
            "mean_map_confidence": mean_conf
        })

        # Failure Taxonomy classification
        failure_type = "PASS" if drift_pct <= 10.0 else "UNKNOWN"
        primary_cause = "NONE"
        if drift_pct > 10.0:
            if dist_gt <= 50.0 and pos_err <= 35.0:
                failure_type = "MODE_D_STANDSTILL_SINGULARITY"
                primary_cause = "Small distance traveled inflated percentage drift"
            elif yaw_err > 30.0 and cross_err > along_err:
                failure_type = "MODE_A_HEADING_OBSERVABILITY"
                primary_cause = f"Unobserved gyro drift accumulated {yaw_err:.1f} deg yaw error"
            elif along_err > cross_err and abs(speed_bias) > 1.0:
                failure_type = "MODE_C_SPEED_RESIDUAL"
                primary_cause = f"GBDT speed biased by {speed_bias:.2f} m/s causing along-track stretch"
            elif seg.duration_s >= 60.0:
                failure_type = "MODE_B_LONG_OUTAGE_ACCUMULATION"
                primary_cause = f"Open-loop inertial drift accumulation over {seg.duration_s:.0f}s"
            elif turn_ticks > 50:
                failure_type = "MODE_E_TURNING_DYNAMICS"
                primary_cause = "Complex maneuvering / roundabouts without sufficient heading constraint"
            else:
                failure_type = "MODE_MIXED"
                primary_cause = "Combined speed residual and lateral drift"

        failure_mode_rows.append({
            "outage_id": seg.outage_id,
            "drive_name": drive_name,
            "duration_s": seg.duration_s,
            "dist_gt_m": dist_gt,
            "drift_pct": drift_pct,
            "failure_type": failure_type,
            "primary_cause": primary_cause
        })

    # Save CSVs
    pd.DataFrame(authoritative_results).to_csv("reports/part3_authoritative_158_outage_results.csv", index=False)
    pd.DataFrame(error_budget_rows).to_csv("reports/part3_error_budget.csv", index=False)
    pd.DataFrame(failure_mode_rows).to_csv("reports/part3_failure_modes.csv", index=False)

    print("Saved:")
    print("  - reports/part3_authoritative_158_outage_results.csv")
    print("  - reports/part3_error_budget.csv")
    print("  - reports/part3_failure_modes.csv")

    # Print summary of failure taxonomy
    df_fail = pd.DataFrame(failure_mode_rows)
    print("\nFAILURE TAXONOMY BREAKDOWN (158 Outages):")
    print(df_fail["failure_type"].value_counts().to_string())


if __name__ == "__main__":
    main()
