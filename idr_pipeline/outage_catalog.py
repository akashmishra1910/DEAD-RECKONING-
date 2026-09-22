r"""
idr_pipeline/outage_catalog.py
==============================
Systematic Authentic Outage Catalog Generator for IO-VNBD Naturalistic Driving.

Provides stratified cataloging across:
- 10s, 30s, 60s, 120s duration buckets
- Multi-driver, multi-corridor coverage (5 authentic drives)
- Rich dynamics (standstill, stop-and-go, cruising, acceleration, braking, highway curves)

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Any, Optional
from .real_iovnbd_loader import TrajectoryData


@dataclass
class OutageSegment:
    outage_id: str
    drive_name: str
    start_time: float
    end_time: float
    duration_s: float
    distance_m: float
    mean_speed_mps: float
    max_speed_mps: float
    motion_regime: str
    delta_heading_deg: float
    max_yaw_rate_rad_s: float


def build_comprehensive_outage_catalog(
    drives: Dict[str, TrajectoryData],
    pre_outage_init_sec: float = 15.0
) -> List[OutageSegment]:
    """
    Builds the 158-outage comprehensive authentic catalog.
    Stratified as:
    - 10s: ~81 outages
    - 30s: ~39 outages
    - 60s: ~25 outages
    - 120s: ~13 outages
    Total = 158 outages across all 5 drives.
    """
    catalog: List[OutageSegment] = []
    outage_idx = 0

    duration_configs = [
        (10.0, [
            ("Driver_A_S3b", 12.0),
            ("Vta01b", 8.0),
            ("Vta04", 10.0),
            ("Vta03", 8.0),
            ("Vta05", 5.0)
        ]),
        (30.0, [
            ("Driver_A_S3b", 25.0),
            ("Vta01b", 15.0),
            ("Vta04", 15.0),
            ("Vta03", 8.0)
        ]),
        (60.0, [
            ("Driver_A_S3b", 35.0),
            ("Vta01b", 15.0),
            ("Vta04", 20.0)
        ]),
        (120.0, [
            ("Driver_A_S3b", 50.0),
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
                i0 = int(round(t_start / dt))
                i1 = int(round(t_end / dt))

                gt_spd = traj.gt_speed[i0:i1]
                gt_hdg = traj.gt_heading[i0:i1]
                gyro_z = traj.gyro[i0:i1, 2]

                dist = float(np.sum(gt_spd * dt))
                mean_spd = float(np.mean(gt_spd))
                max_spd = float(np.max(gt_spd))
                n_stops = int(np.sum(gt_spd < 0.5))

                hdg_unwrapped = np.unwrap(gt_hdg)
                delta_hdg_deg = float(np.degrees(np.abs(hdg_unwrapped[-1] - hdg_unwrapped[0]))) if len(hdg_unwrapped) > 1 else 0.0
                max_yaw_rate = float(np.max(np.abs(gyro_z))) if len(gyro_z) > 0 else 0.0

                if max_spd < 0.5:
                    regime = "standstill"
                elif n_stops > 10:
                    regime = "stop_and_go"
                elif delta_hdg_deg > 25.0 or max_yaw_rate > 0.15:
                    regime = "turning"
                elif max_spd > 15.0:
                    regime = "high_speed"
                else:
                    regime = "cruising"

                outage_idx += 1
                outage_id_str = f"OUT-{int(duration):03d}s-{outage_idx:03d}_{drive_name}"

                catalog.append(OutageSegment(
                    outage_id=outage_id_str,
                    drive_name=drive_name,
                    start_time=t_start,
                    end_time=t_end,
                    duration_s=duration,
                    distance_m=dist,
                    mean_speed_mps=mean_spd,
                    max_speed_mps=max_spd,
                    motion_regime=regime,
                    delta_heading_deg=delta_hdg_deg,
                    max_yaw_rate_rad_s=max_yaw_rate
                ))

                t_start += step_sec

    return catalog
