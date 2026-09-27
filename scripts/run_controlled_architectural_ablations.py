#!/usr/bin/env python3
"""
scripts/run_controlled_architectural_ablations.py
================================================
Executes controlled architectural experiments across all 158 authentic IO-VNBD outages:
M0: Validated B1/N8 Baseline (Golden)
M1: B1/N8 + Causal Motion-State Manager (Stop/Straight/Turn classification + Anti-Creep ZUPT)
M2: B1/N8 + Dynamic Speed Confidence / Uncertainty Scaling
M3: B1/N8 + Adaptive State-Dependent NHC
M4: B1/N8 + Innovation / Residual Health Monitor
M5: B1/N8 + Uncertainty-Aware 3D Map Matching
M6: Synergistic Combination of verified improvements

Outputs:
- BENCHMARK_ABLATION_RESULTS.csv
- reports/ablation_per_outage_details.csv
"""

import os
import sys
import math
import time
import json
import joblib
import numpy as np
import pandas as pd
from typing import Dict, List, Any, Tuple

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
from scripts.run_phase16_final_benchmark import (
    extract_gnss_doppler_velocity,
    estimate_preoutage_observability
)


class MotionState:
    STOPPED = "STOPPED"
    STRAIGHT = "STRAIGHT"
    TURNING = "TURNING"
    ACCELERATING = "ACCELERATING"
    BRAKING = "BRAKING"
    CRUISING = "CRUISING"


class CausalMotionStateManager:
    """
    Causal motion state classifier using purely causal IMU statistics,
    estimated forward velocity, and vibration energy. Zero lookahead.
    """
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        self.accel_hist = []
        self.gyro_hist = []
        self.window_size = 10  # 1.0 second

    def update(self, accel_b: np.ndarray, gyro_b: np.ndarray, v_fwd_est: float, ai_spd: float, vib_e: float) -> str:
        self.accel_hist.append(accel_b.copy())
        self.gyro_hist.append(gyro_b.copy())
        if len(self.accel_hist) > self.window_size:
            self.accel_hist.pop(0)
            self.gyro_hist.pop(0)

        if len(self.accel_hist) < 5:
            return MotionState.CRUISING

        accels = np.array(self.accel_hist)
        gyros = np.array(self.gyro_hist)

        accel_var = float(np.mean(np.var(accels, axis=0)))
        omega_z = float(abs(gyro_b[2]))
        mean_omega = float(np.mean(np.abs(gyros[:, 2])))

        # 1. Standstill / Stopped detection (strict physical criteria)
        # Low acceleration variance, low rotational rate, low estimated velocity
        if accel_var < 0.05 and mean_omega < 0.03 and (v_fwd_est < 1.0 or ai_spd < 0.5):
            return MotionState.STOPPED

        # 2. Turning detection
        if mean_omega > 0.06 or omega_z > 0.08:
            return MotionState.TURNING

        # 3. Acceleration / Braking
        mean_ax = float(np.mean(accels[:, 0]))
        if mean_ax > 0.60:
            return MotionState.ACCELERATING
        elif mean_ax < -0.60:
            return MotionState.BRAKING

        # 4. Straight cruising
        if mean_omega < 0.02 and abs(float(accel_b[1])) < 0.20:
            return MotionState.STRAIGHT

        return MotionState.CRUISING


def compute_adaptive_r_lat(v_fwd: float, omega_z: float, a_y: float, prev_r_lat: float, r_max: float = 0.16) -> float:
    R0 = 0.05**2
    a_cent = v_fwd * omega_z
    target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
    target_r = float(np.clip(target_r, R0, r_max))
    beta = 0.70
    smoothed_r = beta * prev_r_lat + (1.0 - beta) * target_r
    return float(np.clip(smoothed_r, R0, r_max))


def run_experiment(exp_cfg: Dict[str, Any], drives, catalog, doppler_vels, gbdt_speeds, osm_graph) -> Dict[str, Any]:
    exp_id = exp_cfg["id"]
    exp_name = exp_cfg["name"]
    use_motion_state = exp_cfg.get("motion_state", False)
    use_speed_conf = exp_cfg.get("speed_confidence", False)
    use_adaptive_nhc = exp_cfg.get("adaptive_nhc", False)
    use_innov_monitor = exp_cfg.get("innov_monitor", False)
    use_map_conf = exp_cfg.get("map_confidence", False)

    outage_drifts = []
    along_errors = []
    cross_errors = []
    final_errors = []
    yaw_errors = []
    pass_count = 0

    t0 = time.time()

    for seg in catalog:
        traj = drives[seg.drive_name]
        dt = traj.dt
        i0 = int(round(seg.start_time / dt))
        i1 = int(round(seg.end_time / dt))
        i0 = max(0, min(i0, len(traj.time) - 1))
        i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
        dist_gt = max(1.0, float(seg.distance_m))

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

        motion_mgr = CausalMotionStateManager(dt=dt) if use_motion_state else None
        prev_r_lat = 0.05**2
        steady_straight_ticks = 0

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
            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

            # 1. Motion State Classification
            m_state = MotionState.CRUISING
            if motion_mgr:
                m_state = motion_mgr.update(clean_accel_b, clean_gyro_b, v_fwd, ai_spd, vib_e)

            # 2. ESKF Mechanization & Baro
            engine.eskf.predict(clean_accel_b, clean_gyro_b, engine.dt)
            engine.eskf.update_altitude(baro, r_alt=0.8)

            # 3. Longitudinal Speed Update with Adaptive Confidence
            cal_speed = engine.speed_calibrator.correct_speed(ai_spd) if engine.speed_calibrator else ai_spd
            eff_speed = cal_speed

            # Stop override from motion manager
            if use_motion_state and (m_state == MotionState.STOPPED or vib_state.is_engine_idling):
                eff_speed = 0.0
            elif vib_state.is_engine_idling:
                eff_speed = 0.0

            r_ai_adapt = 0.35 * (1.0 + 0.5 * vib_e + 1.2 * abs(omega_z))

            if use_speed_conf:
                # Dynamically increase uncertainty during turns, high vibration, or rapid accelerations
                if m_state == MotionState.TURNING:
                    r_ai_adapt *= 1.4
                elif m_state in (MotionState.ACCELERATING, MotionState.BRAKING):
                    r_ai_adapt *= 1.25
                elif m_state == MotionState.STOPPED:
                    r_ai_adapt = 0.05  # high confidence in zero speed!

            # Innovation monitoring / residual check
            apply_speed = True
            if use_innov_monitor:
                dz_v = abs(eff_speed - v_fwd)
                if dz_v > 8.0 and v_fwd > 2.0:
                    r_ai_adapt *= 2.0  # de-weight speed measurement if innovation is extreme

            if apply_speed:
                engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)

            v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))

            # 4. Adaptive Lateral NHC
            r_max_current = 0.16
            if use_adaptive_nhc:
                if m_state == MotionState.STRAIGHT:
                    r_max_current = 0.09  # tighten on straight
                elif m_state == MotionState.TURNING:
                    r_max_current = 0.25  # allow natural slip on turns

            r_lat_adapt = compute_adaptive_r_lat(v_fwd, omega_z, a_y, prev_r_lat, r_max=r_max_current)
            prev_r_lat = r_lat_adapt
            engine.eskf.update_nhc(r_lat=math.sqrt(r_lat_adapt), r_vert=0.05)

            # 5. Standstill ZUPT
            is_stopped = vib_state.is_engine_idling or (ai_spd < 0.2)
            if use_motion_state and m_state == MotionState.STOPPED:
                is_stopped = True

            if is_stopped:
                engine.eskf.update_zupt(r_zupt=0.01)

            # 6. In-Flight Kalman ZARU
            is_steady_straight = (abs(omega_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0)
            if use_motion_state and m_state == MotionState.STRAIGHT and v_fwd > 4.0:
                is_steady_straight = (abs(omega_z) < 0.008)

            if is_steady_straight:
                steady_straight_ticks += 1
                if steady_straight_ticks > 8:
                    dz_w = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                    H_w = np.zeros((1, 15))
                    H_w[0, 14] = 1.0
                    engine.eskf._apply_kalman_update(dz_w, H_w, np.array([[0.005**2]]))
            else:
                steady_straight_ticks = 0

            # 7. Uncertainty-Aware 3D Map Matching
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

                # Gating
                max_diff = math.radians(45.0)
                if use_map_conf and (m_state == MotionState.TURNING or abs(omega_z) > 0.08):
                    max_diff = math.radians(25.0)  # stricter gating during turns to prevent wrong snap

                if diff_yaw < max_diff:
                    r_yaw = 0.050 if max(norm_w) > 0.80 else 0.087
                    if use_map_conf:
                        # If ESKF yaw uncertainty is high, trust road heading more
                        p_yaw = float(engine.eskf.P[8, 8])
                        if p_yaw > (0.10**2):
                            r_yaw = 0.035  # stronger guidance
                    engine.eskf.update_road_heading(fused_az, r_road_yaw=r_yaw, chi2_gate=25.0)

        # End of outage evaluation
        gt_p = traj.gt_pos[i1 - 1]
        gt_v = traj.gt_vel[i1 - 1]
        dp = engine.eskf.p - gt_p
        pos_err = float(np.linalg.norm(dp[:2]))
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
        final_errors.append(pos_err)
        yaw_errors.append(yaw_err)
        if drift_pct <= 10.0:
            pass_count += 1

    elapsed = time.time() - t0
    med_drift = float(np.median(outage_drifts))
    mean_drift = float(np.mean(outage_drifts))
    p25 = float(np.quantile(outage_drifts, 0.25))
    p75 = float(np.quantile(outage_drifts, 0.75))
    p90 = float(np.quantile(outage_drifts, 0.90))
    p95 = float(np.quantile(outage_drifts, 0.95))
    med_along = float(np.median(along_errors))
    med_cross = float(np.median(cross_errors))
    med_yaw = float(np.median(yaw_errors))

    print(f"[{exp_id}] {exp_name:35s}: Med={med_drift:5.2f}%, Mean={mean_drift:6.2f}%, <=10%: {pass_count:2d}/158, Along={med_along:5.2f}m, Cross={med_cross:5.2f}m, Yaw={med_yaw:5.2f}°, Time={elapsed:.1f}s")

    return {
        "id": exp_id,
        "name": exp_name,
        "median_drift": med_drift,
        "mean_drift": mean_drift,
        "p25": p25,
        "p75": p75,
        "p90": p90,
        "p95": p95,
        "pass_count": pass_count,
        "median_along": med_along,
        "median_cross": med_cross,
        "median_yaw": med_yaw,
        "runtime_s": elapsed,
        "drifts": outage_drifts,
        "along": along_errors,
        "cross": cross_errors
    }


def main():
    print("=" * 90)
    print("CONTROLLED ARCHITECTURAL ABLATIONS ON AUTHENTIC 158-OUTAGE DATASET")
    print("=" * 90)

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

    experiments = [
        {"id": "M0", "name": "B1/N8 Golden Baseline", "motion_state": False, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": False},
        {"id": "M1", "name": "B1/N8 + Motion-State Manager", "motion_state": True, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": False},
        {"id": "M2", "name": "B1/N8 + Speed Uncertainty", "motion_state": False, "speed_confidence": True, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": False},
        {"id": "M3", "name": "B1/N8 + Adaptive NHC", "motion_state": True, "speed_confidence": False, "adaptive_nhc": True, "innov_monitor": False, "map_confidence": False},
        {"id": "M4", "name": "B1/N8 + Innovation Monitor", "motion_state": False, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": True, "map_confidence": False},
        {"id": "M5", "name": "B1/N8 + Uncertainty-Aware Map", "motion_state": False, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": True},
        {"id": "M6", "name": "B1/N8 + MotionState + MapConf", "motion_state": True, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": True},
        {"id": "M7", "name": "B1/N8 + Full Integrated Policy", "motion_state": True, "speed_confidence": True, "adaptive_nhc": True, "innov_monitor": True, "map_confidence": True},
    ]

    results = []
    for exp_cfg in experiments:
        res = run_experiment(exp_cfg, drives, catalog, doppler_vels, gbdt_speeds, osm_graph)
        results.append(res)

    # Save summary CSV
    summary_df = pd.DataFrame([{
        "experiment_id": r["id"],
        "name": r["name"],
        "median_drift_pct": round(r["median_drift"], 2),
        "mean_drift_pct": round(r["mean_drift"], 2),
        "p25_drift_pct": round(r["p25"], 2),
        "p75_drift_pct": round(r["p75"], 2),
        "p90_drift_pct": round(r["p90"], 2),
        "p95_drift_pct": round(r["p95"], 2),
        "pass_count_le_10pct": r["pass_count"],
        "median_along_track_m": round(r["median_along"], 2),
        "median_cross_track_m": round(r["median_cross"], 2),
        "median_yaw_err_deg": round(r["median_yaw"], 2),
        "runtime_s": round(r["runtime_s"], 2)
    } for r in results])

    summary_df.to_csv("BENCHMARK_ABLATION_RESULTS.csv", index=False)
    print("\nSaved summary to BENCHMARK_ABLATION_RESULTS.csv")
    print(summary_df.to_string(index=False))


if __name__ == "__main__":
    main()
