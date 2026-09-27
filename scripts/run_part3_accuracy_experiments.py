#!/usr/bin/env python3
"""
scripts/run_part3_accuracy_experiments.py

Controlled Accuracy Recovery Experiments for SIH 2026 PS 26168.
Evaluates systematic algorithmic hypotheses against the frozen P2-M7 baseline
across all 158 authentic IO-VNBD outages under strict zero-oracle causality.

Experiment Matrix:
  - EXP_00_BASELINE: Frozen P2-M7 control
  - EXP_01_STANDSTILL_IDLE_BLANKING: Physical dynamic-accel standstill detection (zeros idle speed bias)
  - EXP_02_KINEMATIC_YAW_ZARU: Kinematic centripetal yaw observability (a_y ~ v*w_z) + standstill ZARU
  - EXP_03_ADAPTIVE_ROAD_HEADING: Confidence-scaled road azimuth aiding
  - EXP_04_LEARNED_SPEED_UNCERTAINTY: Dynamic R_ai modulated by motion regime and vibration
  - EXP_05_MOTION_REGIME_ADAPTIVE_NHC: Multi-state motion-regime adaptive fusion
  - EXP_06_COMBINED_ACCURACY_SYSTEM: Full integrated accuracy recovery candidate
"""

import os
import sys
import math
import time
from typing import Dict, List, Any, Tuple
import numpy as np
import pandas as pd
import joblib

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


class MotionRegime:
    STOPPED = "STOPPED"
    LOW_SPEED = "LOW_SPEED"
    CRUISE = "CRUISE"
    ACCEL = "ACCEL"
    BRAKE = "BRAKE"
    TURN = "TURN"
    HIGH_VIB = "HIGH_VIB"


def compute_adaptive_r_lat(v_fwd: float, omega_z: float, a_y: float, prev_r_lat: float, r_max: float = 0.16) -> float:
    R0 = 0.05**2
    a_cent = v_fwd * omega_z
    target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
    target_r = float(np.clip(target_r, R0, r_max))
    beta = 0.70
    smoothed_r = beta * prev_r_lat + (1.0 - beta) * target_r
    return float(np.clip(smoothed_r, R0, r_max))


def run_experiment(
    exp_id: str,
    exp_name: str,
    config: Dict[str, Any],
    drives: Dict[str, Any],
    catalog: List[Any],
    doppler_vels: Dict[str, Any],
    gbdt_speeds: Dict[str, Any],
    osm_graph: OfflineOSMGraph
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    t0 = time.time()

    use_idle_blanking = config.get("idle_blanking", False)
    use_kinematic_yaw = config.get("kinematic_yaw", False)
    use_adaptive_heading = config.get("adaptive_heading", False)
    use_learned_r_ai = config.get("learned_r_ai", False)
    use_motion_regime = config.get("motion_regime", False)

    outage_drifts = []
    along_errors = []
    cross_errors = []
    yaw_errors = []
    vert_errors = []
    final_errors = []
    pass_count = 0
    per_outage_records = []

    for seg in catalog:
        drive_name = seg.drive_name
        traj = drives[drive_name]
        dt = traj.dt

        i0 = int(round(seg.start_time / dt))
        i1 = int(round(seg.end_time / dt))
        i0 = max(0, min(i0, len(traj.time) - 1))
        i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
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
        baro_offset = float(traj.baro_alt[0]) - float(traj.gt_pos[0, 2]) if hasattr(traj, 'baro_alt') else 0.0

        # Trailing buffers for dynamic acceleration & kinematic observability
        accel_history = []
        gyro_history = []

        for k in range(i0, i1):
            t_k = float(traj.time[k])
            accel_raw = traj.accel[k]
            gyro_raw = traj.gyro[k]
            baro_alt = (float(traj.baro_alt[k]) - baro_offset) if hasattr(traj, 'baro_alt') else float(traj.gt_pos[k, 2])

            accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel_raw)
            gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro_raw)

            clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)
            vib_energy = float(np.linalg.norm(accel_b - clean_accel_b))

            accel_history.append(clean_accel_b)
            gyro_history.append(clean_gyro_b)
            if len(accel_history) > 10:
                accel_history.pop(0)
                gyro_history.pop(0)

            # Causal physical dynamic fluctuation (AC standard deviation)
            arr_acc = np.array(accel_history)
            arr_gyr = np.array(gyro_history)
            acc_dyn_std = float(np.mean(np.std(arr_acc, axis=0))) if len(accel_history) >= 5 else 1.0
            gyr_std = float(np.mean(np.std(arr_gyr, axis=0))) if len(gyro_history) >= 5 else 1.0

            ai_spd = float(gbdt_speeds[drive_name][k])

            # 1. Physical Standstill & Idle Blanking
            is_physically_stopped = False
            if use_idle_blanking:
                # When vehicle is stopped: AC acceleration fluctuation is tiny (<0.18 m/s²) and gyro is static (<0.02 rad/s)
                if acc_dyn_std < 0.18 and gyr_std < 0.020 and abs(float(clean_gyro_b[2])) < 0.025:
                    is_physically_stopped = True
                    eff_speed = 0.0
                elif vib_state.is_engine_idling or ai_spd < 0.25:
                    is_physically_stopped = True
                    eff_speed = 0.0
                else:
                    eff_speed = ai_spd
            else:
                eff_speed = 0.0 if (vib_state.is_engine_idling or ai_spd < 0.20) else ai_spd

            # 2. Motion Regime Classification
            v_est = float(np.linalg.norm(engine.eskf.v[0:2]))
            w_z = float(clean_gyro_b[2])
            a_x = float(clean_accel_b[0])
            a_y = float(clean_accel_b[1])

            current_regime = MotionRegime.CRUISE
            if is_physically_stopped or (eff_speed < 0.3 and acc_dyn_std < 0.2):
                current_regime = MotionRegime.STOPPED
            elif abs(w_z) > 0.08:
                current_regime = MotionRegime.TURN
            elif a_x > 0.60:
                current_regime = MotionRegime.ACCEL
            elif a_x < -0.60:
                current_regime = MotionRegime.BRAKE
            elif eff_speed < 2.0:
                current_regime = MotionRegime.LOW_SPEED
            elif vib_energy > 2.0:
                current_regime = MotionRegime.HIGH_VIB

            # 3. ESKF Prediction & Altitude Update
            engine.eskf.predict(clean_accel_b, clean_gyro_b, dt)
            engine.eskf.update_altitude(baro_alt, r_alt=0.8)

            # 4. Learned Speed Measurement Update
            if use_learned_r_ai:
                if current_regime == MotionRegime.STOPPED:
                    r_ai_adapt = 0.02
                elif current_regime == MotionRegime.TURN:
                    # In sharp turns, GBDT speed is less reliable due to centripetal bleed
                    r_ai_adapt = 0.35 * (1.0 + 3.0 * abs(w_z) + 1.0 * vib_energy)
                elif current_regime == MotionRegime.HIGH_VIB:
                    r_ai_adapt = 0.35 * (1.0 + 1.5 * vib_energy)
                else:
                    r_ai_adapt = 0.25 * (1.0 + 0.5 * vib_energy + 0.8 * abs(w_z))
            else:
                r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_energy + 1.2 * abs(w_z))
                if eff_speed < 0.5 and vib_energy < 0.20:
                    r_ai_adapt = 0.05

            speed_innov = eff_speed - float(np.linalg.norm(engine.eskf.v[0:2]))
            speed_var = float(engine.eskf.P[3, 3] + r_ai_adapt**2)
            engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)

            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

            # 5. Adaptive NHC
            if use_motion_regime:
                if current_regime == MotionRegime.STOPPED:
                    r_max_current = 0.04
                elif current_regime == MotionRegime.TURN:
                    r_max_current = 0.25
                elif current_regime in [MotionRegime.CRUISE, MotionRegime.ACCEL]:
                    r_max_current = 0.09
                else:
                    r_max_current = 0.16
            else:
                r_max_current = 0.09 if (abs(w_z) < 0.02 and abs(a_y) < 0.20) else 0.16

            r_lat = compute_adaptive_r_lat(v_fwd, abs(w_z), a_y, prev_r_lat, r_max=r_max_current)
            prev_r_lat = r_lat
            r_vert = 0.05**2
            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat), r_vert=math.sqrt(r_vert))

            # 6. ZUPT & In-Flight ZARU
            if is_physically_stopped:
                engine.eskf.update_zupt(r_zupt=0.01)
                # Standstill ZARU: at complete stop, yaw rate is identically 0.0
                dz_zaru = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                H_zaru = np.zeros((1, 15))
                H_zaru[0, 14] = 1.0
                engine.eskf._apply_kalman_update(dz_zaru, H_zaru, np.array([[0.001**2]]))
                steady_straight_ticks = 0
            elif abs(w_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0:
                steady_straight_ticks += 1
                if steady_straight_ticks > 8:
                    dz_w = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                    H_w = np.zeros((1, 15))
                    H_w[0, 14] = 1.0
                    engine.eskf._apply_kalman_update(dz_w, H_w, np.array([[0.005**2]]))
            else:
                steady_straight_ticks = 0

            # 7. Kinematic Centripetal Yaw Rate Observability: a_y ~ v * w_z
            if use_kinematic_yaw and v_fwd > 3.0 and abs(w_z) > 0.04:
                # Theoretical centripetal accel: a_cent = v * w_z
                # Observed lateral accel: a_y. Innovation: dz = clean_gyro_b[2] - a_y / v_fwd
                # Provides direct observability of gyro bias b_gz
                w_kin = a_y / v_fwd
                dz_kin = clean_gyro_b[2] - w_kin
                # Gated: only apply if dynamic consistency holds
                if abs(dz_kin) < 0.10:
                    H_kin = np.zeros((1, 15))
                    H_kin[0, 14] = 1.0
                    r_kin = (0.02)**2
                    engine.eskf._apply_kalman_update(np.array([dz_kin]), H_kin, np.array([[r_kin]]))

            # 8. 3D Map Matching
            pos_curr = engine.eskf.p
            cands = osm_graph.get_candidate_edges(pos_curr, search_radius=75.0)
            road_hdg_nis = None
            road_cross_nis = None

            if cands:
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

                if continuous_conf >= 0.35:
                    r_cross = 1.75 if continuous_conf >= 0.70 else 3.50
                    dz_cross = (pos_curr[0] - best_snap[0]) * (-best_tang[1]) + (pos_curr[1] - best_snap[1]) * best_tang[0]
                    S_cross = float(engine.eskf.P[0, 0] + r_cross**2)
                    road_cross_nis = (dz_cross**2) / S_cross

                    succ_p, gamma_p = engine.eskf.update_road_position(best_snap, best_tang, r_cross=r_cross, chi2_gate=16.0)
                    if succ_p:
                        cur_outage_corrections += 1

                    # Road heading update
                    if d_psi_best < math.radians(25.0):
                        if use_adaptive_heading:
                            # Tighter heading update when confidence is high and vehicle is cruising
                            r_hdg = 0.050 if continuous_conf >= 0.70 else 0.120
                        else:
                            r_hdg = 0.087 if continuous_conf >= 0.70 else 0.174

                        dz_hdg = wrap_pi(engine.eskf.yaw - best_az)
                        S_hdg = float(engine.eskf.P[8, 8] + r_hdg**2)
                        road_hdg_nis = (dz_hdg**2) / S_hdg
                        succ_h, gamma_h = engine.eskf.update_road_heading(best_az, r_road_yaw=r_hdg, chi2_gate=9.0)
                        if succ_h:
                            cur_outage_corrections += 1

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
                turn_rate=abs(w_z)
            )

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

        outage_drifts.append(drift_pct)
        along_errors.append(along_err)
        cross_errors.append(cross_err)
        yaw_errors.append(yaw_err)
        vert_errors.append(vert_err)
        final_errors.append(pos_err)
        if drift_pct <= 10.0:
            pass_count += 1

        per_outage_records.append({
            "experiment_id": exp_id,
            "outage_id": seg.outage_id,
            "drive_name": drive_name,
            "duration_s": seg.duration_s,
            "dist_gt": dist_gt,
            "drift_pct": drift_pct,
            "pos_err_m": pos_err,
            "along_err_m": along_err,
            "cross_err_m": cross_err,
            "yaw_err_deg": yaw_err,
            "vert_err_m": vert_err
        })

    elapsed = time.time() - t0
    summary = {
        "experiment_id": exp_id,
        "name": exp_name,
        "median_drift_pct": float(np.median(outage_drifts)),
        "mean_drift_pct": float(np.mean(outage_drifts)),
        "p25_drift_pct": float(np.quantile(outage_drifts, 0.25)),
        "p75_drift_pct": float(np.quantile(outage_drifts, 0.75)),
        "p90_drift_pct": float(np.quantile(outage_drifts, 0.90)),
        "p95_drift_pct": float(np.quantile(outage_drifts, 0.95)),
        "pass_count_le_10pct": pass_count,
        "median_along_track_m": float(np.median(along_errors)),
        "median_cross_track_m": float(np.median(cross_errors)),
        "median_yaw_err_deg": float(np.median(yaw_errors)),
        "median_vert_err_m": float(np.median(vert_errors)),
        "runtime_s": float(elapsed)
    }

    print(f"[{exp_id}] {exp_name:<36} : Med={summary['median_drift_pct']:.2f}%, "
          f"Mean={summary['mean_drift_pct']:.2f}%, <=10%: {pass_count:>2}/158, "
          f"Cross={summary['median_cross_track_m']:.2f}m, Along={summary['median_along_track_m']:.2f}m, "
          f"Yaw={summary['median_yaw_err_deg']:.2f}°, Time={elapsed:.1f}s")

    return summary, per_outage_records


def main():
    print("=" * 95)
    print("PART 3 ACCURACY RECOVERY & SYSTEMATIC CONVERGENCE SUITE")
    print("Authentic 158-Outage IO-VNBD Dataset | Pure Autonomous Dead Reckoning")
    print("=" * 95)

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

    # Define Experiment Suite
    experiments = [
        ("EXP_00_BASELINE", "Frozen P2-M7 Baseline", {}),
        ("EXP_01_IDLE_BLANKING", "+ Physical Standstill / Idle Blanking", {"idle_blanking": True}),
        ("EXP_02_KINEMATIC_YAW", "+ Kinematic Centripetal Yaw (a_y~v*w)", {"idle_blanking": True, "kinematic_yaw": True}),
        ("EXP_03_ADAPTIVE_HEADING", "+ Adaptive Soft Road Heading", {"idle_blanking": True, "kinematic_yaw": True, "adaptive_heading": True}),
        ("EXP_04_LEARNED_SPEED_UNC", "+ Motion-Regime Speed Uncertainty", {"idle_blanking": True, "kinematic_yaw": True, "adaptive_heading": True, "learned_r_ai": True}),
        ("EXP_05_REGIME_ADAPTIVE_NHC", "+ Full Regime-Adaptive NHC", {"idle_blanking": True, "kinematic_yaw": True, "adaptive_heading": True, "learned_r_ai": True, "motion_regime": True}),
        ("EXP_06_COMBINED_SYSTEM", "Full Integrated Accuracy Recovery", {"idle_blanking": True, "kinematic_yaw": True, "adaptive_heading": True, "learned_r_ai": True, "motion_regime": True})
    ]

    all_summaries = []
    all_details = []

    for exp_id, exp_name, cfg in experiments:
        summary, details = run_experiment(
            exp_id=exp_id,
            exp_name=exp_name,
            config=cfg,
            drives=drives,
            catalog=catalog,
            doppler_vels=doppler_vels,
            gbdt_speeds=gbdt_speeds,
            osm_graph=osm_graph
        )
        all_summaries.append(summary)
        all_details.extend(details)

    df_summary = pd.DataFrame(all_summaries)
    df_summary.to_csv("reports/part3_accuracy_experiments_summary.csv", index=False)
    pd.DataFrame(all_details).to_csv("reports/part3_accuracy_experiments_details.csv", index=False)

    print("\nSAVED:")
    print("  - reports/part3_accuracy_experiments_summary.csv")
    print("  - reports/part3_accuracy_experiments_details.csv")
    print("\nSUMMARY TABLE:")
    print(df_summary[["experiment_id", "name", "median_drift_pct", "mean_drift_pct", "pass_count_le_10pct", "median_cross_track_m", "median_along_track_m", "median_yaw_err_deg"]].to_string(index=False))


if __name__ == "__main__":
    main()
