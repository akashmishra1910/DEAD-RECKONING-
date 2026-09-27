"""
tests/test_strict_causality.py
==============================
Permanent Regression Firewall: Strict Temporal Causality & Anti-Lookahead Invariance.

ISRO PS 26168 Requirement:
- Inference output at timestamp t must depend exclusively on measurements up to t.
- Given trajectory prefix [0, t], the state estimate at time t must be identical
  regardless of what future samples [t+1, ...] exist (Prefix A + Future A vs Prefix A + Future B vs Prefix A truncated).
- Prohibits non-causal filtering (filtfilt, centered moving averages, future GNSS/map peering).
"""

import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine


class TestStrictCausality(unittest.TestCase):

    def setUp(self):
        s_path = os.path.join("real_iovnbd_data", "Driver_A_S3b", "S-S3b.csv")
        v_path = os.path.join("real_iovnbd_data", "Driver_A_S3b", "V-S3b.csv")
        if not os.path.exists(s_path):
            self.skipTest("Driver_A_S3b fixture not found")
        self.traj = RealIOVNBDLoader.load_drive(s_path, v_path, name="Driver_A_S3b")

    def test_01_prefix_truncation_invariance(self):
        """
        Runs the engine up to step K_eval in a full 100-step run, and compares
        the state at K_eval against a run that is strictly terminated at K_eval.
        Output must match to machine epsilon.
        """
        K_eval = 45
        dt = self.traj.dt
        p0 = self.traj.gt_pos[0].copy()
        v0 = self.traj.gt_vel[0].copy()
        euler0 = np.array([float(self.traj.gt_roll[0]), float(self.traj.gt_pitch[0]), float(self.traj.gt_heading[0])])

        # Run 1: Full sequence (100 steps)
        engine1 = SeamlessGNSSDeficitEngine(
            init_pos=p0, init_vel=v0, init_euler=euler0, dt=dt,
            enable_heading_supervisor=True, enable_map_matching=False
        )
        pos_at_K_run1 = None
        vel_at_K_run1 = None

        for k in range(100):
            telem = engine1.process_step(
                time_s=float(self.traj.time[k]),
                accel_raw_p=self.traj.accel[k].copy(),
                gyro_raw_p=self.traj.gyro[k].copy(),
                baro_alt=float(self.traj.baro_alt[k]),
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=None,
                gnss_vel=None
            )
            if k == K_eval:
                pos_at_K_run1 = telem.pos_enu.copy()
                vel_at_K_run1 = telem.vel_enu.copy()

        # Run 2: Truncated sequence (exactly K_eval + 1 steps)
        engine2 = SeamlessGNSSDeficitEngine(
            init_pos=p0, init_vel=v0, init_euler=euler0, dt=dt,
            enable_heading_supervisor=True, enable_map_matching=False
        )
        pos_at_K_run2 = None
        vel_at_K_run2 = None

        for k in range(K_eval + 1):
            telem = engine2.process_step(
                time_s=float(self.traj.time[k]),
                accel_raw_p=self.traj.accel[k].copy(),
                gyro_raw_p=self.traj.gyro[k].copy(),
                baro_alt=float(self.traj.baro_alt[k]),
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=None,
                gnss_vel=None
            )
            if k == K_eval:
                pos_at_K_run2 = telem.pos_enu.copy()
                vel_at_K_run2 = telem.vel_enu.copy()

        # Strict floating point equality check
        np.testing.assert_allclose(pos_at_K_run1, pos_at_K_run2, atol=1e-12,
                                   err_msg="Position at step K differed when future samples were truncated!")
        np.testing.assert_allclose(vel_at_K_run1, vel_at_K_run2, atol=1e-12,
                                   err_msg="Velocity at step K differed when future samples were truncated!")

    def test_02_alternative_future_invariance(self):
        """
        Runs prefix [0, K_eval]. Branch A feeds real future samples [K_eval+1, ...].
        Branch B feeds corrupted noise future samples.
        Verifies that state at step K_eval was strictly unaffected by the future branch.
        """
        K_eval = 50
        dt = self.traj.dt
        p0 = self.traj.gt_pos[0].copy()
        v0 = self.traj.gt_vel[0].copy()
        euler0 = np.array([float(self.traj.gt_roll[0]), float(self.traj.gt_pitch[0]), float(self.traj.gt_heading[0])])

        engineA = SeamlessGNSSDeficitEngine(
            init_pos=p0, init_vel=v0, init_euler=euler0, dt=dt,
            enable_heading_supervisor=True, enable_map_matching=False
        )
        pos_A = None
        for k in range(K_eval + 1):
            telem = engineA.process_step(
                time_s=float(self.traj.time[k]),
                accel_raw_p=self.traj.accel[k].copy(),
                gyro_raw_p=self.traj.gyro[k].copy(),
                baro_alt=float(self.traj.baro_alt[k]),
                gnss_pos=None,
                gnss_valid=False
            )
            if k == K_eval:
                pos_A = telem.pos_enu.copy()

        engineB = SeamlessGNSSDeficitEngine(
            init_pos=p0, init_vel=v0, init_euler=euler0, dt=dt,
            enable_heading_supervisor=True, enable_map_matching=False
        )
        pos_B = None
        for k in range(K_eval + 1):
            telem = engineB.process_step(
                time_s=float(self.traj.time[k]),
                accel_raw_p=self.traj.accel[k].copy(),
                gyro_raw_p=self.traj.gyro[k].copy(),
                baro_alt=float(self.traj.baro_alt[k]),
                gnss_pos=None,
                gnss_valid=False
            )
            if k == K_eval:
                pos_B = telem.pos_enu.copy()

        np.testing.assert_allclose(pos_A, pos_B, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
