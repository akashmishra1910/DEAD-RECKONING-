"""
idr_pipeline/i2nav_external_replay.py
=====================================
Dedicated replay adapter and validation harness for the real-world i2Nav GINS dataset
(Wuhan University, Tang et al., 2022).

Handles:
1. Ingestion of 4 real external MEMS/industrial IMUs:
   - ICM20602 (consumer-grade chip, ~200 Hz)
   - ADIS16460 (industrial-grade module, ~205 Hz)
   - ADIS16465 (industrial-grade module, ~200 Hz)
   - HGuide-i300 (Honeywell industrial-grade module, ~200 Hz)
2. Proper interpretation of incremental measurements:
   - gyro_rate = delta_angle / delta_t
   - accel_rate = delta_velocity / delta_t
   using per-sample delta_t.
3. Coordinate frame transformation:
   - Converts i2Nav FRD (Forward-Right-Down) to project FLU (Forward-Left-Up)
     via verified rotation matrix R_frd2flu = diag([1.0, -1.0, -1.0]).
4. Strictly causal GNSS RTK positioning ingestion:
   - Reads GNSS_RTK.pos at actual 1 Hz timestamps.
   - Derives causal velocity from consecutive position fixes.
   - Zero future lookahead; zero interpolation of future measurements.
5. Strict isolation of ground-truth (truth.nav):
   - truth.nav is used EXCLUSIVELY for post-hoc validation and error metric computation.
   - Never used to initialize, drive, or correct the estimator.
6. Deterministic causal synthetic GNSS-denied evaluation:
   - Imposes controlled outage windows (10s, 30s, 60s, 120s) on identical real driving segments.
7. Metric computation:
   - Position error (horizontal, 3D, along-track, cross-track)
   - Velocity error
   - Positional drift percentage versus distance traveled.

Author: Lead Autonomous Navigation Systems Engineer
"""

import os
import math
import json
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

from .dual_profile import (
    EdgeNavigationEngine,
    NavigationProfileConfig,
    EdgeCalibrationConfig,
    GnssFixType,
    ProfileType,
)
from .real_iovnbd_loader import geodetic_to_enu


# Rotation matrix from i2Nav FRD (Forward-Right-Down) to Project FLU (Forward-Left-Up)
# X_flu = X_frd (Forward)
# Y_flu = -Y_frd (Left = -Right)
# Z_flu = -Z_frd (Up = -Down)
R_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


@dataclass
class I2NavOutageResult:
    sensor_name: str
    outage_id: str
    duration_s: float
    start_time_s: float
    end_time_s: float
    distance_traveled_m: float
    final_pos_error_m: float
    max_pos_error_m: float
    mean_pos_error_m: float
    drift_pct: float
    along_track_err_m: float
    cross_track_err_m: float
    max_vel_error_mps: float
    mean_vel_error_mps: float


@dataclass
class I2NavRunSummary:
    sensor_name: str
    sample_count: int
    duration_s: float
    mean_rate_hz: float
    total_distance_m: float
    gnss_samples_used: int
    outages: List[I2NavOutageResult] = field(default_factory=list)
    overall_mean_pos_err_m: float = 0.0
    overall_max_pos_err_m: float = 0.0


class I2NavReplayAdapter:
    """
    Adapter and evaluation pipeline for the real i2Nav multi-sensor dataset.
    """

    SENSOR_PATHS = {
        "ICM20602": {
            "imu": "awesome-gins-datasets/ICM20602/data/ICM20602/ICM20602.txt",
            "truth": "awesome-gins-datasets/ICM20602/data/ICM20602/truth.nav",
            "desc": "InvenSense Consumer-Grade Chip",
            "grade": "Consumer MEMS",
        },
        "ADIS16460": {
            "imu": "awesome-gins-datasets/ADIS16460/data/ADIS16460/ADIS16460.txt",
            "truth": "awesome-gins-datasets/ADIS16460/data/ADIS16460/truth.nav",
            "desc": "Analog Devices Industrial Module",
            "grade": "Industrial MEMS",
        },
        "ADIS16465": {
            "imu": "awesome-gins-datasets/ADIS16465/data/ADIS16465/ADIS16465.txt",
            "truth": "awesome-gins-datasets/ADIS16465/data/ADIS16465/truth.nav",
            "desc": "Analog Devices Tactical Industrial Module",
            "grade": "Industrial/Tactical MEMS",
        },
        "HGuide-i300": {
            "imu": "awesome-gins-datasets/i300/data/i300/i300.txt",
            "truth": "awesome-gins-datasets/i300/data/i300/truth.nav",
            "desc": "Honeywell Industrial Grade Module",
            "grade": "Industrial MEMS",
        },
    }

    GNSS_PATH = "awesome-gins-datasets/GNSS RTK/data/GNSS_RTK.pos"

    def __init__(self, base_dir: str = "."):
        self.base_dir = base_dir

    def resolve_path(self, rel_path: str) -> str:
        return os.path.join(self.base_dir, rel_path)

    def inspect_dataset(self) -> Dict[str, Any]:
        """
        Programmatically inspects all 4 IMU files, GNSS, and truth files.
        Returns a comprehensive dataset inventory dictionary.
        """
        inventory: Dict[str, Any] = {"sensors": {}, "gnss": {}}

        # Inspect GNSS
        gnss_abs = self.resolve_path(self.GNSS_PATH)
        if os.path.exists(gnss_abs):
            gnss_data = np.loadtxt(gnss_abs)
            t_gnss = gnss_data[:, 0]
            dt_gnss = np.diff(t_gnss)
            inventory["gnss"] = {
                "file": self.GNSS_PATH,
                "samples": len(t_gnss),
                "duration_s": float(t_gnss[-1] - t_gnss[0]),
                "t_start": float(t_gnss[0]),
                "t_end": float(t_gnss[-1]),
                "dt_mean": float(np.mean(dt_gnss)),
                "dt_min": float(np.min(dt_gnss)),
                "dt_max": float(np.max(dt_gnss)),
                "cadence_hz": float(1.0 / np.mean(dt_gnss)),
                "monotonic": bool((dt_gnss > 0).all()),
                "missing_values": int(np.isnan(gnss_data).sum() + np.isinf(gnss_data).sum()),
            }

        # Inspect each IMU
        for name, meta in self.SENSOR_PATHS.items():
            imu_abs = self.resolve_path(meta["imu"])
            truth_abs = self.resolve_path(meta["truth"])

            if not os.path.exists(imu_abs):
                continue

            imu_data = np.loadtxt(imu_abs)
            t_imu = imu_data[:, 0]
            dt_imu = np.diff(t_imu)
            d_angle = imu_data[:, 1:4]
            d_vel = imu_data[:, 4:7]

            truth_info = {}
            if os.path.exists(truth_abs):
                truth_data = np.loadtxt(truth_abs)
                t_truth = truth_data[:, 1]
                dt_truth = np.diff(t_truth)
                truth_info = {
                    "samples": len(t_truth),
                    "t_start": float(t_truth[0]),
                    "t_end": float(t_truth[-1]),
                    "cadence_hz": float(1.0 / np.mean(dt_truth)),
                }

            inventory["sensors"][name] = {
                "description": meta["desc"],
                "grade": meta["grade"],
                "file": meta["imu"],
                "samples": len(t_imu),
                "columns": int(imu_data.shape[1]),
                "duration_s": float(t_imu[-1] - t_imu[0]),
                "t_start": float(t_imu[0]),
                "t_end": float(t_imu[-1]),
                "dt_mean": float(np.mean(dt_imu)),
                "dt_min": float(np.min(dt_imu)),
                "dt_max": float(np.max(dt_imu)),
                "effective_rate_hz": float(1.0 / np.mean(dt_imu)),
                "monotonic": bool((dt_imu > 0).all()),
                "d_angle_range": [float(d_angle.min()), float(d_angle.max())],
                "d_vel_range": [float(d_vel.min()), float(d_vel.max())],
                "missing_values": int(np.isnan(imu_data).sum() + np.isinf(imu_data).sum()),
                "truth": truth_info,
            }

        return inventory

    def load_gnss(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float, float]:
        """
        Loads GNSS RTK positioning fixes and returns:
        times (N,), enu_positions (N, 3), accuracies (N,), lat0, lon0, h0
        """
        gnss_abs = self.resolve_path(self.GNSS_PATH)
        data = np.loadtxt(gnss_abs)
        times = data[:, 0]
        lat = data[:, 1]
        lon = data[:, 2]
        h = data[:, 3]
        std_lat = data[:, 4]
        std_lon = data[:, 5]

        lat0 = float(lat[0])
        lon0 = float(lon[0])
        h0 = float(h[0])

        enu_pos = geodetic_to_enu(lat, lon, h, lat0, lon0, h0)
        horiz_acc = np.sqrt(std_lat**2 + std_lon**2)

        return times, enu_pos, horiz_acc, lat0, lon0, h0

    def load_truth(self, imu_name: str, lat0: float, lon0: float, h0: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Loads ground truth from truth.nav and converts coordinates relative to (lat0, lon0, h0).
        Returns:
        times (M,), enu_positions (M, 3), enu_velocities (M, 3), euler_attitudes_deg (M, 3)
        """
        meta = self.SENSOR_PATHS[imu_name]
        truth_abs = self.resolve_path(meta["truth"])
        data = np.loadtxt(truth_abs)

        times = data[:, 1]
        lat = data[:, 2]
        lon = data[:, 3]
        h = data[:, 4]
        v_n = data[:, 5]
        v_e = data[:, 6]
        v_d = data[:, 7]
        roll = data[:, 8]
        pitch = data[:, 9]
        yaw = data[:, 10]

        enu_pos = geodetic_to_enu(lat, lon, h, lat0, lon0, h0)
        # ENU velocities: v_E, v_N, v_U (-v_D)
        enu_vel = np.column_stack([v_e, v_n, -v_d])
        att_deg = np.column_stack([roll, pitch, yaw])

        return times, enu_pos, enu_vel, att_deg

    def run_replay(
        self,
        imu_name: str,
        start_offset_s: float = 0.0,
        duration_s: float = 300.0,
        outage_windows: Optional[List[Tuple[float, float]]] = None,
    ) -> Tuple[I2NavRunSummary, Dict[str, np.ndarray]]:
        """
        Executes causal replay of the real i2Nav sensor stream into the production EdgeNavigationEngine.

        Parameters:
        - imu_name: One of 'ICM20602', 'ADIS16460', 'ADIS16465', 'HGuide-i300'
        - start_offset_s: Relative start time in seconds (e.g. 0.0)
        - duration_s: Duration of continuous trajectory to evaluate (e.g. 300.0s)
        - outage_windows: List of (t_start_rel, t_end_rel) defining causal GNSS denial intervals.
        """
        meta = self.SENSOR_PATHS[imu_name]
        imu_abs = self.resolve_path(meta["imu"])

        # 1. Load GNSS and establish reference geodetic datum
        t_gnss_all, enu_gnss_all, acc_gnss_all, lat0, lon0, h0 = self.load_gnss()

        # 2. Load ground truth for post-hoc evaluation
        t_truth_all, enu_truth_all, vel_truth_all, att_truth_all = self.load_truth(imu_name, lat0, lon0, h0)

        # 3. Load IMU data
        imu_raw = np.loadtxt(imu_abs)
        t_imu_all = imu_raw[:, 0]

        t0_base = t_imu_all[0] + start_offset_s
        t_end_target = t0_base + duration_s

        # Slice IMU to requested evaluation window
        mask_imu = (t_imu_all >= t0_base) & (t_imu_all <= t_end_target)
        imu_sub = imu_raw[mask_imu]

        t_imu = imu_sub[:, 0]
        d_angles = imu_sub[:, 1:4]
        d_vels = imu_sub[:, 4:7]
        N_imu = len(t_imu)

        # Slice GNSS to window
        mask_gnss = (t_gnss_all >= t0_base - 1.0) & (t_gnss_all <= t_end_target + 1.0)
        t_gnss = t_gnss_all[mask_gnss]
        enu_gnss = enu_gnss_all[mask_gnss]
        acc_gnss = acc_gnss_all[mask_gnss]

        # 4. Initialize production EdgeNavigationEngine with EDGE_FOG profile
        engine = EdgeNavigationEngine()
        cfg = NavigationProfileConfig.edge_fog_defaults()
        # Compute exact empirical dt from dataset
        dt_empirical = float(np.mean(np.diff(t_imu)))
        effective_hz = 1.0 / dt_empirical
        cfg.imu_sample_rate_hz = effective_hz
        cfg.target_update_rate_hz = effective_hz

        # Apply exact FRD -> FLU coordinate rotation matrix
        cfg.calibration = EdgeCalibrationConfig(
            R_sensor_to_body=R_FRD_TO_FLU,
            accel_scale_factor=np.ones(3),
            gyro_scale_factor=np.ones(3),
            accel_bias_b=np.zeros(3),
            gyro_bias_b=np.zeros(3)
        )
        engine.initialize(cfg)

        # Convert outage windows relative to t0_base
        active_outages = []
        if outage_windows:
            for w_start_rel, w_end_rel in outage_windows:
                active_outages.append((t0_base + w_start_rel, t0_base + w_end_rel, w_end_rel - w_start_rel))

        # Replay telemetry recording buffers
        rec_time: List[float] = []
        rec_pos: List[np.ndarray] = []
        rec_vel: List[np.ndarray] = []
        rec_att: List[np.ndarray] = []
        rec_in_outage: List[bool] = []

        gnss_ptr = 0
        last_gnss_t = -1.0
        last_gnss_pos = None
        gnss_pushed_count = 0

        # Initial GNSS lock if available
        while gnss_ptr < len(t_gnss) and t_gnss[gnss_ptr] <= t_imu[0]:
            last_gnss_t = t_gnss[gnss_ptr]
            last_gnss_pos = enu_gnss[gnss_ptr]
            gnss_ptr += 1

        if last_gnss_pos is not None:
            engine.pushGnss(
                timestamp_s=t_imu[0],
                pos_enu=last_gnss_pos,
                vel_enu=None,
                valid=True,
                accuracy_m=float(acc_gnss[max(0, gnss_ptr - 1)]),
                fix_type=GnssFixType.RTK_FIXED
            )

        # 5. Causal Streaming Loop
        for k in range(1, N_imu):
            t_curr = float(t_imu[k])
            t_prev = float(t_imu[k - 1])
            dt = max(1e-5, t_curr - t_prev)

            # Causal check for GNSS outages
            in_denial = False
            for w_start, w_end, _ in active_outages:
                if w_start <= t_curr < w_end:
                    in_denial = True
                    break

            # Push causal GNSS fixes that arrived since previous IMU sample
            while gnss_ptr < len(t_gnss) and t_gnss[gnss_ptr] <= t_curr:
                gnss_t = float(t_gnss[gnss_ptr])
                gnss_pos = enu_gnss[gnss_ptr]
                gnss_acc = float(acc_gnss[gnss_ptr])

                # Derive causal GNSS velocity if previous fix available
                vel_derived = None
                if last_gnss_pos is not None and last_gnss_t > 0:
                    dt_g = gnss_t - last_gnss_t
                    if 0.5 <= dt_g <= 2.5:
                        vel_derived = (gnss_pos - last_gnss_pos) / dt_g

                last_gnss_t = gnss_t
                last_gnss_pos = gnss_pos

                if not in_denial:
                    engine.pushGnss(
                        timestamp_s=gnss_t,
                        pos_enu=gnss_pos,
                        vel_enu=vel_derived,
                        valid=True,
                        accuracy_m=gnss_acc,
                        fix_type=GnssFixType.RTK_FIXED
                    )
                    gnss_pushed_count += 1
                gnss_ptr += 1

            if in_denial:
                # Force denial in engine
                engine.pushGnss(t_curr, np.zeros(3), np.zeros(3), valid=False)

            # Convert increments to physical rates (per-sample dt)
            omega_meas = d_angles[k] / dt  # rad/s in sensor FRD
            accel_meas = d_vels[k] / dt    # m/s^2 in sensor FRD

            # Push IMU sample
            engine.pushImu(t_curr, accel_meas, omega_meas)

            # Filter update
            ok = engine.update(timestamp_s=t_curr, ai_speed=None)
            if ok:
                st = engine.getNavigationState()
                if st is not None:
                    rec_time.append(t_curr)
                    rec_pos.append(st.pos_enu.copy())
                    rec_vel.append(st.vel_enu.copy())
                    rec_att.append(np.array([st.roll_deg, st.pitch_deg, st.heading_deg]))
                    rec_in_outage.append(st.in_outage)

        rec_t_arr = np.array(rec_time)
        rec_pos_arr = np.array(rec_pos)
        rec_vel_arr = np.array(rec_vel)

        # 6. Post-hoc Truth Alignment & Metric Computation
        # Interpolate ground truth to exact estimator timestamps
        mask_tr = (t_truth_all >= rec_t_arr[0] - 0.1) & (t_truth_all <= rec_t_arr[-1] + 0.1)
        tr_t = t_truth_all[mask_tr]
        tr_pos = enu_truth_all[mask_tr]
        tr_vel = vel_truth_all[mask_tr]

        truth_pos_interp = np.zeros_like(rec_pos_arr)
        truth_vel_interp = np.zeros_like(rec_vel_arr)
        for col in range(3):
            truth_pos_interp[:, col] = np.interp(rec_t_arr, tr_t, tr_pos[:, col])
            truth_vel_interp[:, col] = np.interp(rec_t_arr, tr_t, tr_vel[:, col])

        # Overall errors
        pos_err_vec = rec_pos_arr - truth_pos_interp
        horiz_err = np.linalg.norm(pos_err_vec[:, :2], axis=1)
        mean_err = float(np.mean(horiz_err))
        max_err = float(np.max(horiz_err))

        # Compute Outage Metrics
        outage_results: List[I2NavOutageResult] = []
        for i, (w_start, w_end, dur) in enumerate(active_outages):
            outage_mask = (rec_t_arr >= w_start) & (rec_t_arr <= w_end)
            if not np.any(outage_mask):
                continue

            out_pos_err = horiz_err[outage_mask]
            out_tr_pos = truth_pos_interp[outage_mask]
            out_tr_vel = truth_vel_interp[outage_mask]
            out_rec_pos = rec_pos_arr[outage_mask]
            out_rec_vel = rec_vel_arr[outage_mask]

            # Distance traveled during outage
            dist_traveled = float(np.sum(np.linalg.norm(np.diff(out_tr_pos, axis=0), axis=1)))
            final_err = float(out_pos_err[-1])
            max_out_err = float(np.max(out_pos_err))
            mean_out_err = float(np.mean(out_pos_err))
            drift_pct = (final_err / max(1.0, dist_traveled)) * 100.0

            # Along-track and cross-track decomposition
            v_dir = out_tr_vel[-1, :2]
            v_norm = np.linalg.norm(v_dir)
            if v_norm > 0.1:
                u_along = v_dir / v_norm
                u_cross = np.array([-u_along[1], u_along[0]])
                final_err_vec = pos_err_vec[outage_mask][-1, :2]
                along_err = float(abs(np.dot(final_err_vec, u_along)))
                cross_err = float(abs(np.dot(final_err_vec, u_cross)))
            else:
                along_err = final_err
                cross_err = 0.0

            vel_err_mag = np.linalg.norm(out_rec_vel - out_tr_vel, axis=1)

            outage_results.append(
                I2NavOutageResult(
                    sensor_name=imu_name,
                    outage_id=f"{imu_name}_{int(dur)}s",
                    duration_s=float(dur),
                    start_time_s=float(w_start - t0_base),
                    end_time_s=float(w_end - t0_base),
                    distance_traveled_m=dist_traveled,
                    final_pos_error_m=final_err,
                    max_pos_error_m=max_out_err,
                    mean_pos_error_m=mean_out_err,
                    drift_pct=drift_pct,
                    along_track_err_m=along_err,
                    cross_track_err_m=cross_err,
                    max_vel_error_mps=float(np.max(vel_err_mag)),
                    mean_vel_error_mps=float(np.mean(vel_err_mag)),
                )
            )

        summary = I2NavRunSummary(
            sensor_name=imu_name,
            sample_count=N_imu,
            duration_s=float(t_imu[-1] - t_imu[0]),
            mean_rate_hz=effective_hz,
            total_distance_m=float(np.sum(np.linalg.norm(np.diff(truth_pos_interp, axis=0), axis=1))),
            gnss_samples_used=gnss_pushed_count,
            outages=outage_results,
            overall_mean_pos_err_m=mean_err,
            overall_max_pos_err_m=max_err,
        )

        trajectories = {
            "time": rec_t_arr,
            "est_pos": rec_pos_arr,
            "truth_pos": truth_pos_interp,
            "horiz_error": horiz_err,
            "in_outage": np.array(rec_in_outage, dtype=bool),
        }

        return summary, trajectories


def run_four_sensor_benchmark(
    duration_s: float = 240.0,
    outage_durations: Tuple[int, ...] = (10, 30, 60, 120),
) -> Tuple[List[I2NavRunSummary], List[I2NavOutageResult]]:
    """
    Executes an identical, non-tuned benchmark evaluation across all 4 i2Nav IMUs.
    Applies identical synthetic outage windows to every sensor:
    - 10s outage
    - 30s outage
    - 60s outage
    - 120s outage
    """
    adapter = I2NavReplayAdapter()

    # Define standardized non-overlapping outage windows during steady vehicle travel:
    # 0s to 50s: GNSS-aided convergence & initial lock
    # 50s to 60s: 10s outage
    # 70s to 100s: 30s outage
    # 110s to 170s: 60s outage
    # 180s to 300s (or test separate 120s window)
    outages = [
        (50.0, 60.0),    # 10s outage
        (70.0, 100.0),   # 30s outage
        (110.0, 170.0),  # 60s outage
    ]
    if duration_s >= 310.0:
        outages.append((180.0, 300.0))  # 120s outage

    summaries = []
    all_outages = []

    for sensor in ["ICM20602", "ADIS16460", "ADIS16465", "HGuide-i300"]:
        summary, _ = adapter.run_replay(
            imu_name=sensor,
            start_offset_s=0.0,
            duration_s=duration_s,
            outage_windows=outages,
        )
        summaries.append(summary)
        all_outages.extend(summary.outages)

    return summaries, all_outages
