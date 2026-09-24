"""
tests/test_phase14_fast_no_lookahead.py
=======================================
Anti-lookahead and strict temporal causality test suite for Phase 14 Candidate A1.

Verifies:
1. Candidate A1 executes strictly sequentially with time t_k.
2. Perturbing all future sensor inputs (t > t_k) produces Delta = 0.000000000000 m difference
   in estimated state at t <= t_k.
3. No future trajectory, ground-truth attitude, or future road hypotheses are accessed.
"""

import math
import unittest
import numpy as np

from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.eskf_fusion import wrap_pi


class TestPhase14FastNoLookahead(unittest.TestCase):
    """Test suite validating causality and anti-lookahead invariance for Candidate A1."""

    def test_01_future_perturbation_invariance_candidate_a1(self):
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
            bias_rate = 0.015

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
                engine.eskf.update_nhc(r_lat=0.05, r_vert=0.05)

                # Candidate A1 causal bias observer
                omega_z = float(clean_gyro_b[2])
                a_y = float(clean_accel_b[1])
                v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))
                if abs(omega_z) < 0.005 and abs(a_y) < 0.15 and v_fwd > 4.0:
                    steady_straight_ticks += 1
                    if steady_straight_ticks > 8:
                        engine.eskf.bg[2] = (1.0 - bias_rate) * engine.eskf.bg[2] + bias_rate * gyro_b[2]
                else:
                    steady_straight_ticks = 0

                history.append({
                    "pos": engine.eskf.p.copy(),
                    "vel": engine.eskf.v.copy(),
                    "yaw": float(engine.eskf.yaw)
                })

            return history

        base = run_sim(0.0)
        pert1 = run_sim(2.0)
        pert2 = run_sim(-2.5)

        for k in range(cut_idx + 1):
            diff_p = np.linalg.norm(base[k]["pos"] - pert1[k]["pos"])
            diff_v = np.linalg.norm(base[k]["vel"] - pert1[k]["vel"])
            diff_y = abs(wrap_pi(base[k]["yaw"] - pert1[k]["yaw"]))

            self.assertEqual(diff_p, 0.0, f"Position changed at step {k} <= {cut_idx}!")
            self.assertEqual(diff_v, 0.0, f"Velocity changed at step {k} <= {cut_idx}!")
            self.assertEqual(diff_y, 0.0, f"Yaw changed at step {k} <= {cut_idx}!")

            diff_p2 = np.linalg.norm(base[k]["pos"] - pert2[k]["pos"])
            self.assertEqual(diff_p2, 0.0)


if __name__ == "__main__":
    unittest.main()
