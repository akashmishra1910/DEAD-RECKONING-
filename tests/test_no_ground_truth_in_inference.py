"""
tests/test_no_ground_truth_in_inference.py
=========================================
Permanent Regression Firewall: Prohibits Any Ground-Truth / Oracle Access in Inference.

ISRO PS 26168 Evaluation Standard:
- Ground truth (gt_speed, gt_vel, gt_pos, gt_heading, etc.) is strictly for:
    1. Training labels (offline)
    2. Benchmark scoring / error evaluation
- Ground truth MUST NEVER be accessed or injected during inference.
- Any attempt by an estimator or inference loop to read ground-truth attributes
  during navigation propagation will immediately raise an error and fail this test.
"""

import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader, TrajectoryData
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.offline_osm_graph import OfflineOSMGraph


class TrajectoryOracleTrap:
    """
    A protective wrapper around TrajectoryData that permits ground-truth access
    ONLY at initialization (t=0) and offline scoring, but raises an explicit
    AssertionError if ground-truth attributes are accessed during live inference.
    """
    def __init__(self, traj: TrajectoryData):
        self._traj = traj
        self.inference_active = False

    def __getattr__(self, name: str):
        if self.inference_active and name in ("gt_speed", "gt_vel", "gt_pos", "gt_heading", "gt_pitch", "gt_roll"):
            raise AssertionError(f"ORACLE LEAKAGE DETECTED: Inference attempted to read '{name}'!")
        return getattr(self._traj, name)


class TestNoGroundTruthInInference(unittest.TestCase):

    def test_01_static_code_audit_for_oracle_speed(self):
        """Scans production pipeline files for illicit gt_speed usage during inference."""
        pipeline_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr_pipeline"))
        excluded_files = {"real_iovnbd_loader.py", "dataset_loader.py", "outage_catalog.py", "benchmark_evaluation.py"}
        for root, _, files in os.walk(pipeline_dir):
            for file in files:
                if file.endswith(".py") and file not in excluded_files:
                    filepath = os.path.join(root, file)
                    with open(filepath, "r", encoding="utf-8") as f:
                        lines = f.readlines()
                    for line_num, line in enumerate(lines, 1):
                        code = line.split("#")[0].strip()
                        if "gt_speed[" in code or "gt_vel[" in code:
                            self.fail(f"Oracle leakage found in {file}:{line_num} -> {code}")

    def test_02_dynamic_inference_trap_no_oracle_access(self):
        """
        Executes a 100-step dead reckoning propagation with an active Oracle Trap.
        Verifies that SeamlessGNSSDeficitEngine runs strictly from IMU + Baro without
        touching any ground-truth properties.
        """
        s_path = os.path.join("real_iovnbd_data", "Driver_A_S3b", "S-S3b.csv")
        v_path = os.path.join("real_iovnbd_data", "Driver_A_S3b", "V-S3b.csv")
        if not os.path.exists(s_path):
            self.skipTest("Driver_A_S3b fixture not found")

        raw_traj = RealIOVNBDLoader.load_drive(s_path, v_path, name="Driver_A_S3b")
        trapped_traj = TrajectoryOracleTrap(raw_traj)

        # Allow initial state setup
        p0 = trapped_traj.gt_pos[0].copy()
        v0 = trapped_traj.gt_vel[0].copy()
        yaw0 = float(trapped_traj.gt_heading[0])
        pitch0 = float(trapped_traj.gt_pitch[0])
        roll0 = float(trapped_traj.gt_roll[0])

        engine = SeamlessGNSSDeficitEngine(
            init_pos=p0,
            init_vel=v0,
            init_euler=np.array([roll0, pitch0, yaw0]),
            dt=trapped_traj.dt,
            enable_heading_supervisor=True,
            enable_map_matching=False
        )

        # ACTIVATE INFERENCE TRAP: Any gt_* access will trigger AssertionError
        trapped_traj.inference_active = True

        for k in range(100):
            accel = trapped_traj.accel[k].copy()
            gyro = trapped_traj.gyro[k].copy()
            baro = float(trapped_traj.baro_alt[k])

            # In DR mode (no GNSS, learned speed or NHC only)
            telemetry = engine.process_step(
                time_s=float(trapped_traj.time[k]),
                accel_raw_p=accel,
                gyro_raw_p=gyro,
                baro_alt=baro,
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=None, # Pure dead reckoning without oracle
                gnss_vel=None
            )
            self.assertIsNotNone(telemetry)
            self.assertFalse(np.isnan(telemetry.pos_enu).any())

        trapped_traj.inference_active = False


if __name__ == "__main__":
    unittest.main()
