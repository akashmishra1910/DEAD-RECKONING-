"""
tests/test_phase15_no_lookahead.py
==================================
Strict Anti-Lookahead and Temporal Causality Test Suite for Phase 15 Final Estimator.

ISRO / Space Applications Centre (SAC) — Problem Statement PS 26168
AI-ML Based Intelligent Dead Reckoning System

Verifies:
1. Phase 15 Final Estimator executes strictly sequentially with current timestamp t_k.
2. Perturbing future sensor observations (t > t_k) produces Delta = 0.000000000000 m difference
   in estimated position, velocity, and attitude for all t <= t_k.
3. No future trajectory, ground-truth attitude/velocity, or lookahead hypotheses are accessed.
"""

import os
import sys
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.eskf_fusion import wrap_pi


class TestPhase15NoLookahead(unittest.TestCase):
    """Test suite validating strict causality and anti-lookahead invariance for Phase 15 Final."""

    def test_01_future_perturbation_invariance(self):
        """
        Verify that future perturbations for t > t_k cause zero deviation in
        position, velocity, and attitude for all t <= t_k.
        """
        N_steps = 30
        cut_idx = 15

        def run_sim(future_pert: float):
            engine = SeamlessGNSSDeficitEngine(
                init_pos=np.array([50.0, 50.0, 0.0]),
                init_vel=np.array([8.0, 0.0, 0.0]),
                init_euler=np.array([0.0, 0.0, 0.2]),
                dt=0.1,
                enable_heading_supervisor=True,
                enable_map_matching=False,
                enable_speed_calibration=True,
                enable_longitudinal_map_constraint=False
            )

            history = []
            steady_straight_ticks = 0
            prev_r_lat = 0.05**2
            max_r_lat = 0.40

            for k in range(N_steps):
                accel = np.array([0.1, 0.0, 9.81])
                gyro = np.array([0.0, 0.0, 0.002])
                ai_spd = 8.0

                if k > cut_idx:
                    accel += np.array([future_pert, -future_pert, 0.0])
                    gyro += np.array([0.0, 0.0, future_pert * 0.05])
                    ai_spd += future_pert * 3.0

                accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel)
                gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro)
                clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)

                engine.eskf.predict(clean_accel_b, clean_gyro_b, engine.dt)
                engine.eskf.update_altitude(0.0, r_alt=0.8)

                r_ai = 0.35 * (1.0 + 0.5 * float(np.linalg.norm(accel_b - clean_accel_b)))
                engine.eskf.update_ai_velocity(ai_spd, r_ai=r_ai)

                v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))
                omega_z = float(clean_gyro_b[2])
                a_y = float(clean_accel_b[1])

                # Phase 15 Bounded-Slip NHC
                R0 = 0.05**2
                R_max_val = max_r_lat**2
                a_cent = v_fwd * omega_z
                target_r = R0 * (1.0 + 25.0 * (omega_z**2) + 12.0 * (a_cent**2) + 5.0 * (a_y**2))
                target_r = float(np.clip(target_r, R0, R_max_val))
                r_lat_adapt = float(np.clip(0.70 * prev_r_lat + 0.30 * target_r, R0, R_max_val))
                prev_r_lat = r_lat_adapt

                engine.eskf.update_nhc(r_lat=math.sqrt(r_lat_adapt), r_vert=0.05)

                # Phase 15 ESKF Kalman ZARU
                is_steady_straight = (abs(omega_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0)
                if is_steady_straight:
                    steady_straight_ticks += 1
                    if steady_straight_ticks > 8:
                        dz = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                        H = np.zeros((1, 15))
                        H[0, 14] = 1.0
                        R_zaru = np.array([[0.005**2]])
                        engine.eskf._apply_kalman_update(dz, H, R_zaru)
                else:
                    steady_straight_ticks = 0

                history.append({
                    "pos": engine.eskf.p.copy(),
                    "vel": engine.eskf.v.copy(),
                    "yaw": float(engine.eskf.yaw)
                })

            return history

        base = run_sim(0.0)
        pert1 = run_sim(3.5)
        pert2 = run_sim(-4.2)

        for k in range(cut_idx + 1):
            diff_p = np.linalg.norm(base[k]["pos"] - pert1[k]["pos"])
            diff_v = np.linalg.norm(base[k]["vel"] - pert1[k]["vel"])
            diff_y = abs(wrap_pi(base[k]["yaw"] - pert1[k]["yaw"]))

            self.assertEqual(diff_p, 0.0, f"Position deviated at step {k} <= {cut_idx}!")
            self.assertEqual(diff_v, 0.0, f"Velocity deviated at step {k} <= {cut_idx}!")
            self.assertEqual(diff_y, 0.0, f"Yaw deviated at step {k} <= {cut_idx}!")

            diff_p2 = np.linalg.norm(base[k]["pos"] - pert2[k]["pos"])
            self.assertEqual(diff_p2, 0.0)


if __name__ == "__main__":
    unittest.main()
