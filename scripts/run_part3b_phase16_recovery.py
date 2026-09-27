#!/usr/bin/env python3
"""
scripts/run_part3b_phase16_recovery.py

Authoritative Phase 16 Recovery Suite inside the Unified Architecture.
Evaluates:
  1. EXP_00_BASELINE: Current unified autonomous control baseline (P2-M7)
  2. EXP_01_HISTORICAL_P16_REF: Historical Phase 16 reproduction (CAN/GT-speed aided diagnostic)
  3. EXP_02_P16_AUTONOMOUS_SPEED: Exact Phase 16 filter architecture with autonomous GBDT speed
  4. EXP_03_DOPPLER_HANDOVER_ONLY: Component ablation: Pre-outage Doppler handover
  5. EXP_04_P16_R_AI_ONLY: Component ablation: Phase-16 adaptive R_ai
  6. EXP_05_P16_NHC_ONLY: Component ablation: Phase-16 Bounded-Slip NHC
  7. EXP_06_P16_ZARU_ONLY: Component ablation: Phase-16 In-flight ZARU
  8. EXP_07_FULL_P16_RECOVERED: Full recovered Phase-16 architecture (pure autonomous)
  9. Counterfactual Diagnostics (A: Auto Spd + Est Hdg, B: GT Spd + Est Hdg, C: Auto Spd + GT Hdg, D: GT Spd + GT Hdg)

Outputs:
  - reports/phase16_recovery_exp00_baseline.csv
  - reports/phase16_recovery_historical_reference.csv
  - reports/phase16_recovery_exp02_autonomous_speed.csv
  - reports/phase16_recovery_exp07_full.csv
  - reports/phase16_counterfactual_error_budget.csv
  - reports/phase16_recovery_master_comparison.csv
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

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi, euler_to_rotation_matrix
from idr_pipeline.offline_osm_graph import OfflineOSMGraph
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog, OutageSegment
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.map_matcher_3d import Causal3DMapMatcher
from idr_pipeline.integrity_manager import IntegrityManager
from scripts.run_phase16_final_benchmark import (
    extract_gnss_doppler_velocity,
    estimate_preoutage_observability
)


def evaluate_engine_outage(
    traj: TrajectoryData,
    drive_name: str,
    seg: OutageSegment,
    doppler_v: np.ndarray,
    v_ai_all: np.ndarray,
    osm_graph: OfflineOSMGraph,
    mode: str,
    # Architectural flags
    use_doppler_handover: bool = True,
    speed_source: str = "gbdt",  # 'gbdt' or 'gt'
    heading_mode: str = "p16_soft_road",  # 'p16_soft_road', 'unified_p2m7', 'gt_heading'
    nhc_mode: str = "p16_bounded",  # 'p16_bounded', 'n7_loose', 'none'
    r_ai_mode: str = "p16_adapt",  # 'p16_adapt', 'fixed'
    use_zaru: bool = True,
    use_baro_calib: bool = True
) -> Dict[str, Any]:
    dt = traj.dt
    i0 = int(round(seg.start_time / dt))
    i1 = int(round(seg.end_time / dt))
    i0 = max(0, min(i0, len(traj.time) - 1))
    i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
    dist_gt = max(1.0, float(seg.distance_m))

    # Pre-outage Doppler handover (3.0s window before t0)
    if use_doppler_handover:
        ba_init, bg_init, P_init = estimate_preoutage_observability(traj, doppler_v, i0, pre_window_s=3.0)
    else:
        ba_init = np.zeros(3)
        bg_init = np.zeros(3)
        P_init = np.eye(15) * 0.1
        P_init[6:9, 6:9] *= (0.10**2)
        P_init[9:12, 9:12] *= (0.05**2)
        P_init[12:15, 12:15] *= (0.01**2)

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

    prev_r_lat = 0.05**2
    steady_straight_ticks = 0
    cur_outage_corrections = 0
    cur_outage_rejections = 0

    baro_offset = float(traj.baro_alt[0]) - float(traj.gt_pos[0, 2]) if use_baro_calib else 0.0

    map_matcher = Causal3DMapMatcher(graph=osm_graph)
    integrity_mgr = IntegrityManager()

    # Outage propagation loop
    for k in range(i0, i1):
        t_k = float(traj.time[k])
        accel_raw = traj.accel[k].copy()
        gyro_raw = traj.gyro[k].copy()
        baro_val = float(traj.baro_alt[k]) - baro_offset

        accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel_raw)
        gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro_raw)
        clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)

        omega_z = float(clean_gyro_b[2])
        a_y = float(clean_accel_b[1])

        # 1. ESKF Prediction & Altitude Update
        engine.eskf.predict(clean_accel_b, clean_gyro_b, dt)
        engine.eskf.update_altitude(baro_val, r_alt=0.8)

        # 2. Forward Speed Input Selection
        if speed_source == "gt":
            # Diagnostic CAN reference only
            eff_speed = float(traj.gt_speed[k])
        else:
            raw_ai = float(v_ai_all[k])
            eff_speed = 0.0 if vib_state.is_engine_idling else raw_ai

        # Adaptive R_ai
        if r_ai_mode == "p16_adapt":
            vib_diff = float(np.linalg.norm(accel_b - clean_accel_b))
            r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_diff + 1.2 * abs(omega_z))
        else:
            r_ai_adapt = 0.35

        engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)
        v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

        # 3. NHC Lateral Slip Update
        if nhc_mode == "p16_bounded":
            R0 = 0.05**2
            R_max_val = 0.40**2  # 0.16
            a_cent = v_fwd * omega_z
            target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
            target_r = float(np.clip(target_r, R0, R_max_val))
            r_lat_adapt = float(np.clip(0.70 * prev_r_lat + 0.30 * target_r, R0, R_max_val))
            prev_r_lat = r_lat_adapt
            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat_adapt), r_vert=0.05)
        elif nhc_mode == "n7_loose":
            R0 = 0.05**2
            R_max_val = 2.25
            a_cent = v_fwd * omega_z
            target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
            target_r = float(np.clip(target_r, R0, R_max_val))
            r_lat_adapt = float(np.clip(0.70 * prev_r_lat + 0.30 * target_r, R0, R_max_val))
            prev_r_lat = r_lat_adapt
            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat_adapt), r_vert=0.05)

        # ZUPT
        if eff_speed < 0.20 or vib_state.is_engine_idling:
            engine.eskf.update_zupt(r_zupt=0.01)

        # 4. In-Flight ZARU
        if use_zaru:
            is_steady_straight = (abs(omega_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0)
            if is_steady_straight:
                steady_straight_ticks += 1
                if steady_straight_ticks > 8:
                    dz = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                    H = np.zeros((1, 15))
                    H[0, 14] = 1.0
                    engine.eskf._apply_kalman_update(dz, H, np.array([[0.005**2]]))
            else:
                steady_straight_ticks = 0

        # 5. Heading Handling / Map Matching
        if heading_mode == "gt_heading":
            # Diagnostic Oracle heading for counterfactuals
            gt_yaw_k = float(traj.gt_heading[k])
            engine.eskf.yaw = gt_yaw_k
            engine.eskf.R = euler_to_rotation_matrix(engine.eskf.roll, engine.eskf.pitch, gt_yaw_k)
        elif heading_mode == "p16_soft_road":
            # Exact Historical Phase 16 Heading-Only Soft Fusion
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
                    succ, _ = engine.eskf.update_road_heading(fused_az, r_road_yaw=r_yaw, chi2_gate=25.0)
                    if succ:
                        cur_outage_corrections += 1
                    else:
                        cur_outage_rejections += 1
        elif heading_mode == "unified_p2m7":
            # Part 2 P2-M7 Position Snapping + Confidence Gated Heading
            pos_curr = engine.eskf.p
            cands = osm_graph.get_candidate_edges(pos_curr, search_radius=75.0)
            if cands:
                weights, results = [], []
                for e in cands[:3]:
                    d2d, delta_h, snap, tang, az = osm_graph.project_point_to_edge(pos_curr, e)
                    d_psi = abs(wrap_pi(engine.eskf.yaw - az))
                    log_p_dist = -0.5 * (min(d2d, 50.0) / 5.0)**2
                    log_p_psi = -0.5 * (min(d_psi, 1.0) / 0.30)**2
                    log_p_alt = -50.0 - 5.0 * (delta_h - 4.0) if delta_h > 4.0 else -0.5 * (delta_h / 1.8)**2
                    log_p_trans = 0.0
                    if map_matcher.last_matched_edge_id is not None and e.id != map_matcher.last_matched_edge_id:
                        is_conn = (map_matcher.last_matched_edge_id in osm_graph.edges) and (e.id in osm_graph.adjacency.get(osm_graph.edges[map_matcher.last_matched_edge_id].target_id, []))
                        log_p_trans = 0.0 if is_conn else -15.0
                    weights.append(log_p_dist + log_p_psi + log_p_alt + log_p_trans)
                    results.append((e, d2d, delta_h, snap, tang, az))

                max_lp = max(weights)
                exp_w = [math.exp(w - max_lp) for w in weights]
                sum_exp_w = sum(exp_w) + 1e-12
                posteriors = [w / sum_exp_w for w in exp_w]
                best_idx = int(np.argmax(posteriors))
                best_edge, best_d2d, best_dh, best_snap, best_tang, best_az = results[best_idx]
                score_margin = float(weights[best_idx] - (sorted(weights, reverse=True)[1] if len(weights) > 1 else -999.0))
                map_matcher.last_matched_edge_id = best_edge.id

                d_psi_best = abs(wrap_pi(engine.eskf.yaw - best_az))
                yaw_var = float(engine.eskf.P[8, 8])
                c_map = integrity_mgr.compute_continuous_map_confidence(best_d2d, best_dh, d_psi_best, score_margin, yaw_var, map_matcher.consecutive_valid_steps, 4.0)

                if c_map >= 0.35:
                    r_cross = 1.75 if c_map >= 0.70 else 3.50
                    engine.eskf.update_road_position(best_snap, best_tang, r_cross=r_cross, chi2_gate=16.0)
                    if d_psi_best < math.radians(25.0):
                        r_hdg = 0.087 if c_map >= 0.70 else 0.174
                        engine.eskf.update_road_heading(best_az, r_road_yaw=r_hdg, chi2_gate=9.0)

    # Compute outage final metrics
    gt_p = traj.gt_pos[i1 - 1]
    gt_v = traj.gt_vel[i1 - 1]
    dp = engine.eskf.p - gt_p
    pos_err = float(np.linalg.norm(dp[:2]))
    vert_err = abs(float(dp[2]))
    drift_pct = (pos_err / dist_gt) * 100.0

    gt_spd_end = float(traj.gt_speed[i1 - 1])
    if gt_spd_end > 0.5:
        e_along_vec = np.array([gt_v[0], gt_v[1]]) / gt_spd_end
        e_cross_vec = np.array([-gt_v[1], gt_v[0]]) / gt_spd_end
        along_err = abs(float(np.dot(dp[:2], e_along_vec)))
        cross_err = abs(float(np.dot(dp[:2], e_cross_vec)))
    else:
        along_err = pos_err
        cross_err = 0.0

    yaw_err = math.degrees(abs(float(wrap_pi(engine.eskf.yaw - traj.gt_heading[i1 - 1]))))

    return {
        "outage_id": seg.outage_id,
        "drive_name": drive_name,
        "duration_s": seg.duration_s,
        "dist_gt_m": dist_gt,
        "drift_pct": drift_pct,
        "pos_err_m": pos_err,
        "cross_err_m": cross_err,
        "along_err_m": along_err,
        "yaw_err_deg": yaw_err,
        "vert_err_m": vert_err,
        "passed_le_10pct": int(drift_pct <= 10.0)
    }


def run_configuration_suite(
    cfg_id: str,
    cfg_name: str,
    cfg_params: Dict[str, Any],
    drives: Dict[str, Any],
    catalog: List[Any],
    doppler_vels: Dict[str, Any],
    gbdt_speeds: Dict[str, Any],
    osm_graph: OfflineOSMGraph
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    t0 = time.time()
    records = []

    for seg in catalog:
        rec = evaluate_engine_outage(
            traj=drives[seg.drive_name],
            drive_name=seg.drive_name,
            seg=seg,
            doppler_v=doppler_vels[seg.drive_name],
            v_ai_all=gbdt_speeds[seg.drive_name],
            osm_graph=osm_graph,
            mode=cfg_id,
            **cfg_params
        )
        rec["config_id"] = cfg_id
        records.append(rec)

    elapsed = time.time() - t0
    drifts = [r["drift_pct"] for r in records]
    along = [r["along_err_m"] for r in records]
    cross = [r["cross_err_m"] for r in records]
    yaw = [r["yaw_err_deg"] for r in records]
    vert = [r["vert_err_m"] for r in records]
    passes = sum(r["passed_le_10pct"] for r in records)

    summary = {
        "config_id": cfg_id,
        "name": cfg_name,
        "speed_source": cfg_params.get("speed_source", "gbdt"),
        "gt_during_outage": "YES" if (cfg_params.get("speed_source") == "gt" or cfg_params.get("heading_mode") == "gt_heading") else "NO",
        "median_drift_pct": float(np.median(drifts)),
        "mean_drift_pct": float(np.mean(drifts)),
        "p25_drift_pct": float(np.quantile(drifts, 0.25)),
        "p75_drift_pct": float(np.quantile(drifts, 0.75)),
        "p90_drift_pct": float(np.quantile(drifts, 0.90)),
        "p95_drift_pct": float(np.quantile(drifts, 0.95)),
        "pass_count_le_10pct": passes,
        "pass_rate_pct": float(passes / len(catalog) * 100.0),
        "median_along_track_m": float(np.median(along)),
        "median_cross_track_m": float(np.median(cross)),
        "median_yaw_err_deg": float(np.median(yaw)),
        "median_vert_err_m": float(np.median(vert)),
        "runtime_s": float(elapsed)
    }

    print(f"[{cfg_id}] {cfg_name:<40} : Med={summary['median_drift_pct']:.2f}%, "
          f"Mean={summary['mean_drift_pct']:.2f}%, <=10%: {passes:>2}/158, "
          f"Cross={summary['median_cross_track_m']:.2f}m, Along={summary['median_along_track_m']:.2f}m, "
          f"Yaw={summary['median_yaw_err_deg']:.2f}°, Time={elapsed:.1f}s")

    return summary, records


def main():
    print("=" * 100)
    print("PART 3B — PHASE 16 RECOVERY INSIDE THE UNIFIED ARCHITECTURE")
    print("Authentic 158-Outage IO-VNBD Benchmark | Comprehensive Investigation")
    print("=" * 100)

    # 1. Load authentic drives
    data_dir = "real_iovnbd_data"
    drives = RealIOVNBDLoader.load_all_drives(data_dir)
    catalog = build_comprehensive_outage_catalog(drives)
    print(f"Loaded {len(drives)} drives and {len(catalog)} outages.")

    # 2. Extract genuine GNSS Doppler velocity from S-*.csv files
    csv_mapping = {
        "Driver_A_S3b": os.path.join(data_dir, "Driver_A_S3b", "S-S3b.csv"),
        "Vta01b": os.path.join(data_dir, "Vta01b", "S-Vta1b.csv"),
        "Vta04": os.path.join(data_dir, "Vta04", "S-Vta4.csv"),
        "Vta03": os.path.join(data_dir, "Vta03", "S-Vta3.csv"),
        "Vta05": os.path.join(data_dir, "Vta05", "S-Vta5.csv"),
    }
    doppler_vels = {}
    for name, traj in drives.items():
        csv_file = csv_mapping.get(name)
        if csv_file and os.path.exists(csv_file):
            doppler_vels[name] = extract_gnss_doppler_velocity(csv_file, len(traj.time))
        else:
            doppler_vels[name] = traj.gt_vel.copy()

    # 3. Load GBDT forward speed model
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

    # 4. Authentic topological 3D road graph
    osm_graph = OfflineOSMGraph.build_from_drives(drives)

    # 5. Define Experiment Configurations
    configs = [
        # EXP_00: Baseline Control
        ("EXP_00_BASELINE", "Current Unified Baseline (P2-M7)", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "unified_p2m7",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),

        # EXP_01: Historical Phase-16 Reference (Diagnostic CAN-aided)
        ("EXP_01_HISTORICAL_REF", "Historical Phase 16 (CAN/GT-Aided Reference)", {
            "use_doppler_handover": True,
            "speed_source": "gt",
            "heading_mode": "p16_soft_road",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": False  # Historical fed raw baro
        }),

        # EXP_02: Phase-16 Architecture + Autonomous Speed
        ("EXP_02_P16_AUTONOMOUS_SPEED", "Phase 16 Architecture + GBDT Speed", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "p16_soft_road",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": False
        }),

        # EXP_03: Component Ablation: Pre-outage Doppler Handover
        ("EXP_03_DOPPLER_HANDOVER_ONLY", "Unified Baseline - No Doppler Handover", {
            "use_doppler_handover": False,
            "speed_source": "gbdt",
            "heading_mode": "unified_p2m7",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),

        # EXP_04: Component Ablation: Phase-16 Adaptive R_ai
        ("EXP_04_P16_R_AI_ONLY", "Unified Baseline - Fixed R_ai (No Adapt)", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "unified_p2m7",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "fixed",
            "use_zaru": True,
            "use_baro_calib": True
        }),

        # EXP_05: Component Ablation: Phase-16 NHC Configuration
        ("EXP_05_P16_NHC_ONLY", "Unified Baseline - Loose N7 NHC (R_max=2.25)", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "unified_p2m7",
            "nhc_mode": "n7_loose",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),

        # EXP_06: Component Ablation: Phase-16 ZARU
        ("EXP_06_P16_ZARU_ONLY", "Unified Baseline - No In-Flight ZARU", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "unified_p2m7",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": False,
            "use_baro_calib": True
        }),

        # EXP_07: Full Recovered Phase-16 Architecture (Autonomous)
        ("EXP_07_FULL_RECOVERED", "Full Recovered Phase 16 (Heading-Only Map)", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "p16_soft_road",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),

        # Counterfactual Diagnostics
        ("DIAG_A_AUTO_SPD_EST_HDG", "Diag A: Auto Speed + Est Heading", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "p16_soft_road",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),
        ("DIAG_B_GT_SPD_EST_HDG", "Diag B: GT Speed + Est Heading (CAN Oracle)", {
            "use_doppler_handover": True,
            "speed_source": "gt",
            "heading_mode": "p16_soft_road",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),
        ("DIAG_C_AUTO_SPD_GT_HDG", "Diag C: Auto Speed + GT Heading (Gyro Oracle)", {
            "use_doppler_handover": True,
            "speed_source": "gbdt",
            "heading_mode": "gt_heading",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        }),
        ("DIAG_D_GT_SPD_GT_HDG", "Diag D: GT Speed + GT Heading (Dual Oracle)", {
            "use_doppler_handover": True,
            "speed_source": "gt",
            "heading_mode": "gt_heading",
            "nhc_mode": "p16_bounded",
            "r_ai_mode": "p16_adapt",
            "use_zaru": True,
            "use_baro_calib": True
        })
    ]

    all_summaries = []
    exp_records_map = {}

    for cfg_id, cfg_name, cfg_params in configs:
        summary, recs = run_configuration_suite(
            cfg_id=cfg_id,
            cfg_name=cfg_name,
            cfg_params=cfg_params,
            drives=drives,
            catalog=catalog,
            doppler_vels=doppler_vels,
            gbdt_speeds=gbdt_speeds,
            osm_graph=osm_graph
        )
        all_summaries.append(summary)
        exp_records_map[cfg_id] = recs

    os.makedirs("reports", exist_ok=True)

    # Save designated experiment files
    pd.DataFrame(exp_records_map["EXP_00_BASELINE"]).to_csv("reports/phase16_recovery_exp00_baseline.csv", index=False)
    pd.DataFrame(exp_records_map["EXP_01_HISTORICAL_REF"]).to_csv("reports/phase16_recovery_historical_reference.csv", index=False)
    pd.DataFrame(exp_records_map["EXP_02_P16_AUTONOMOUS_SPEED"]).to_csv("reports/phase16_recovery_exp02_autonomous_speed.csv", index=False)
    pd.DataFrame(exp_records_map["EXP_07_FULL_RECOVERED"]).to_csv("reports/phase16_recovery_exp07_full.csv", index=False)

    # Counterfactual Diagnostics Table
    diag_rows = [s for s in all_summaries if s["config_id"].startswith("DIAG_")]
    pd.DataFrame(diag_rows).to_csv("reports/phase16_counterfactual_error_budget.csv", index=False)

    # Master comparison table
    df_all = pd.DataFrame(all_summaries)
    df_all.to_csv("reports/phase16_recovery_master_comparison.csv", index=False)

    print("\nSAVED ALL REPORTS:")
    print("  - reports/phase16_recovery_exp00_baseline.csv")
    print("  - reports/phase16_recovery_historical_reference.csv")
    print("  - reports/phase16_recovery_exp02_autonomous_speed.csv")
    print("  - reports/phase16_recovery_exp07_full.csv")
    print("  - reports/phase16_counterfactual_error_budget.csv")
    print("  - reports/phase16_recovery_master_comparison.csv")

    print("\nMASTER COMPARISON TABLE:")
    disp_cols = ["config_id", "name", "speed_source", "gt_during_outage", "median_drift_pct", "mean_drift_pct", "pass_count_le_10pct", "median_cross_track_m", "median_along_track_m", "median_yaw_err_deg"]
    print(df_all[disp_cols].to_string(index=False))


if __name__ == "__main__":
    main()
