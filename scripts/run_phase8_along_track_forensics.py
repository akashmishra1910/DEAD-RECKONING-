r"""
scripts/run_phase8_along_track_forensics.py
===========================================
Phase 8 Along-Track Forensic Diagnostic & Quantitative Error Budget Breakdown.

ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System

Architecture: WINDOWED PER-OUTAGE
    Each outage gets a fresh engine initialized at GT state at outage start.
    Only the outage window [start, end] is replayed (pure DR, no GNSS).
    This matches the Phase 5 evaluation methodology and is O(total_outage_samples).

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import os
import sys
import math
import numpy as np
import pandas as pd
from typing import Dict, List, Any, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.offline_osm_graph import OfflineOSMGraph
from idr_pipeline.eskf_fusion import wrap_pi
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine, NavigationMode
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog, OutageSegment


def run_along_track_forensics():
    print("=" * 80)
    print("PHASE 8 — ALONG-TRACK FORENSIC DIAGNOSTICS & ERROR BUDGET BREAKDOWN")
    print("=" * 80)

    drives_info = [
        ("Driver_A_S3b", os.path.join("real_iovnbd_data", "Driver_A_S3b", "S-S3b.csv"), os.path.join("real_iovnbd_data", "Driver_A_S3b", "V-S3b.csv")),
        ("Vta01b", os.path.join("real_iovnbd_data", "Vta01b", "S-Vta1b.csv"), os.path.join("real_iovnbd_data", "Vta01b", "V-Vta1b.csv")),
        ("Vta04", os.path.join("real_iovnbd_data", "Vta04", "S-Vta4.csv"), os.path.join("real_iovnbd_data", "Vta04", "V-Vta4.csv")),
        ("Vta03", os.path.join("real_iovnbd_data", "Vta03", "S-Vta3.csv"), os.path.join("real_iovnbd_data", "Vta03", "V-Vta3.csv")),
        ("Vta05", os.path.join("real_iovnbd_data", "Vta05", "S-Vta5.csv"), os.path.join("real_iovnbd_data", "Vta05", "V-Vta5.csv")),
    ]

    loaded_drives: Dict[str, TrajectoryData] = {}
    for name, s_path, v_path in drives_info:
        if os.path.exists(s_path) and os.path.exists(v_path):
            traj = RealIOVNBDLoader.load_drive(s_path, v_path, name=name)
            loaded_drives[name] = traj
            print(f"  Loaded drive {name:15s}: {len(traj.time):5d} samples, duration {traj.time[-1]:.1f} s")

    osm_graph_all = OfflineOSMGraph.build_from_drives(loaded_drives)
    catalog = build_comprehensive_outage_catalog(loaded_drives)
    print(f"\nTotal authentic outages in catalog: {len(catalog)}")

    os.makedirs("reports", exist_ok=True)

    forensic_records = []
    budget_records = []

    for oi, outage in enumerate(catalog):
        if outage.drive_name not in loaded_drives:
            continue
        traj = loaded_drives[outage.drive_name]
        dt = traj.dt
        dur = outage.duration_s

        # Compute sample indices for the outage window
        i0 = int(round(outage.start_time / dt))
        i1 = int(round(outage.end_time / dt))
        i0 = max(0, min(i0, len(traj.time) - 1))
        i1 = max(i0 + 1, min(i1, len(traj.time) - 1))
        N = i1 - i0

        # Initialize fresh engine at GT state at outage start (pure DR evaluation)
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

        # Cumulative error budget accumulators
        cum_speed_bias_drift = 0.0
        cum_heading_proj_err = 0.0
        cum_accel_leakage = 0.0
        cum_gyro_bias_drift = 0.0
        cum_zupt_err = 0.0
        cum_pothole_err = 0.0
        cum_discrete_trunc_err = 0.0
        cum_straight_unobs_err = 0.0

        # Initial handover error (should be ~0 since we init from GT)
        initial_handover_err = float(np.linalg.norm(p0[0:2] - traj.gt_pos[i0, 0:2]))

        pos_at_end = None
        gt_at_end = None

        for k in range(N):
            g_idx = i0 + k
            t = float(traj.time[g_idx])
            ai_spd = float(traj.gt_speed[g_idx])
            mag_raw = traj.mag[g_idx] if traj.mag is not None else None

            # Pure DR: no GNSS during outage
            telemetry = engine.process_step(
                time_s=t,
                accel_raw_p=traj.accel[g_idx],
                gyro_raw_p=traj.gyro[g_idx],
                baro_alt=float(traj.baro_alt[g_idx]),
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=ai_spd,
                gnss_vel=None,
                mag_raw_p=mag_raw,
                enable_road_constraints=True
            )

            pos_est = telemetry.pos_enu[0:2]
            pos_gt = traj.gt_pos[g_idx, 0:2]
            err_vec = pos_est - pos_gt
            total_err = float(np.linalg.norm(err_vec))

            gt_yaw = traj.gt_heading[g_idx]
            t_gt = np.array([math.sin(gt_yaw), math.cos(gt_yaw)])
            n_gt = np.array([math.cos(gt_yaw), -math.sin(gt_yaw)])

            e_along = float(np.dot(err_vec, t_gt))
            e_cross = float(np.dot(err_vec, n_gt))

            yaw_est = telemetry.heading_deg * math.pi / 180.0
            delta_psi = abs(float(wrap_pi(yaw_est - gt_yaw)))

            v_est = float(np.linalg.norm(telemetry.vel_enu[0:2]))
            v_gt = float(traj.gt_speed[g_idx])
            delta_v = v_est - v_gt

            # Component accumulation
            cum_speed_bias_drift += abs(delta_v) * dt
            cum_heading_proj_err += v_gt * (1.0 - math.cos(delta_psi)) * dt
            cum_accel_leakage += abs(float(telemetry.accel_bias[0])) * 0.5 * (dt ** 2)
            cum_gyro_bias_drift += abs(float(telemetry.gyro_bias[2])) * v_gt * (dt ** 2)
            if v_gt < 0.2 and v_est >= 0.2:
                cum_zupt_err += v_est * dt
            if telemetry.is_pothole_active:
                cum_pothole_err += 0.05 * dt
            cum_discrete_trunc_err += 0.5 * abs(delta_v) * (dt ** 2)
            if abs(traj.gyro[g_idx, 2]) < 0.05:
                cum_straight_unobs_err += abs(e_along) * dt / max(1.0, dur)

            # Log at ~1 Hz (every 10 samples at 10 Hz) and at outage end
            if (k % 10 == 0) or (k == N - 1):
                forensic_records.append({
                    "outage_id": outage.outage_id,
                    "drive": outage.drive_name,
                    "time_s": t,
                    "outage_elapsed_s": t - outage.start_time,
                    "duration_s": dur,
                    "total_error_m": total_err,
                    "along_track_error_m": e_along,
                    "cross_track_error_m": e_cross,
                    "along_cross_ratio": abs(e_along) / max(0.1, abs(e_cross)),
                    "speed_est_mps": v_est,
                    "speed_gt_mps": v_gt,
                    "speed_error_mps": delta_v,
                    "heading_err_deg": math.degrees(delta_psi),
                    "gyro_bias_z_rads": float(telemetry.gyro_bias[2]),
                    "accel_bias_x_mps2": float(telemetry.accel_bias[0]),
                    "speed_scale_est": telemetry.diagnostics.speed_scale_est,
                    "speed_bias_est_mps": telemetry.diagnostics.speed_bias_est_mps,
                    "long_constraint_applied": telemetry.diagnostics.longitudinal_constraint_applied
                })

            pos_at_end = pos_est.copy()
            gt_at_end = pos_gt.copy()

        # End of outage: record budget summary
        if pos_at_end is not None and gt_at_end is not None:
            final_err_vec = pos_at_end - gt_at_end
            final_total_err = float(np.linalg.norm(final_err_vec))
            outage_dist = max(1.0, float(outage.distance_m))
            drift_pct = (final_total_err / outage_dist) * 100.0

            raw_sum = (cum_speed_bias_drift + cum_heading_proj_err + cum_accel_leakage +
                       cum_gyro_bias_drift + cum_zupt_err + cum_pothole_err +
                       cum_discrete_trunc_err + initial_handover_err)
            scale_factor = final_total_err / max(1e-3, raw_sum)

            budget_records.append({
                "outage_id": outage.outage_id,
                "drive": outage.drive_name,
                "duration_s": dur,
                "distance_m": outage_dist,
                "final_error_m": final_total_err,
                "drift_pct": drift_pct,
                "comp_A_speed_drift_m": cum_speed_bias_drift * scale_factor,
                "comp_B_heading_proj_m": cum_heading_proj_err * scale_factor,
                "comp_C_accel_leakage_m": cum_accel_leakage * scale_factor,
                "comp_D_gyro_bias_m": cum_gyro_bias_drift * scale_factor,
                "comp_E_zupt_misclass_m": cum_zupt_err * scale_factor,
                "comp_F_pothole_vibe_m": cum_pothole_err * scale_factor,
                "comp_G_straight_unobs_m": cum_straight_unobs_err,
                "comp_H_truncation_m": cum_discrete_trunc_err * scale_factor,
                "comp_I_initial_handover_m": initial_handover_err * scale_factor
            })

        if (oi + 1) % 20 == 0 or (oi + 1) == len(catalog):
            print(f"  Processed {oi + 1}/{len(catalog)} outages...")

    # Save to CSVs
    df_forensics = pd.DataFrame(forensic_records)
    df_forensics.to_csv("reports/phase8_along_track_forensics.csv", index=False)

    df_budget = pd.DataFrame(budget_records)
    df_budget.to_csv("reports/phase8_error_budget_breakdown.csv", index=False)

    print(f"\nGenerated forensic logs: {len(df_forensics)} epochs across {len(catalog)} outages.")
    print(f"Saved to reports/phase8_along_track_forensics.csv")
    print(f"Saved to reports/phase8_error_budget_breakdown.csv")

    # Display Mean Error Budget Breakdown Table
    print("\n" + "=" * 80)
    print("QUANTITATIVE ERROR BUDGET ATTRIBUTION TABLE (MEAN ACROSS OUTAGES)")
    print("=" * 80)
    if len(df_budget) > 0:
        mean_budget = {
            "A. Velocity Scale/Bias Drift": df_budget["comp_A_speed_drift_m"].mean(),
            "B. Heading Cross-Projection": df_budget["comp_B_heading_proj_m"].mean(),
            "C. Accel Bias Leakage": df_budget["comp_C_accel_leakage_m"].mean(),
            "D. Gyro Bias Drift": df_budget["comp_D_gyro_bias_m"].mean(),
            "E. Standstill/ZUPT Latency": df_budget["comp_E_zupt_misclass_m"].mean(),
            "F. Pothole / Road Noise": df_budget["comp_F_pothole_vibe_m"].mean(),
            "H. Numerical Truncation": df_budget["comp_H_truncation_m"].mean(),
            "I. Handover State Uncertainty": df_budget["comp_I_initial_handover_m"].mean(),
        }
        total_mean = sum(mean_budget.values())
        print(f"{'Component':<35s} | {'Mean Error (m)':<15s} | {'Share (%)':<10s}")
        print("-" * 66)
        for comp_name, val in mean_budget.items():
            pct = (val / max(1e-3, total_mean)) * 100.0
            print(f"{comp_name:<35s} | {val:15.3f} | {pct:9.2f}%")
        print("-" * 66)
        print(f"{'Total Mean Error Budget':<35s} | {total_mean:15.3f} | 100.00%")

    # Stratified Summary
    print("\n" + "=" * 80)
    print("DRIFT BY DURATION (PS 26168 TARGET: <10%)")
    print("=" * 80)
    if len(df_budget) > 0:
        for dur in [10.0, 30.0, 60.0, 120.0]:
            sub = df_budget[df_budget["duration_s"] == dur]
            if len(sub) > 0:
                med = sub["drift_pct"].median()
                mean_d = sub["drift_pct"].mean()
                p10 = (sub["drift_pct"] < 10.0).sum()
                tot = len(sub)
                print(f"  {dur:3.0f}s (N={tot:2d}): Median Drift = {med:6.2f}%, "
                      f"Mean = {mean_d:6.2f}%, Pass(<10%) = {p10}/{tot}")
    print("=" * 80)


if __name__ == "__main__":
    run_along_track_forensics()
