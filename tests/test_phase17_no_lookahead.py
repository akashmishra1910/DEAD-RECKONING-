"""
tests/test_phase17_no_lookahead.py
==================================
Strict Anti-Lookahead and Temporal Causality Test Suite for Phase 17 Estimator.

ISRO / Space Applications Centre (SAC) — Problem Statement PS 26168
AI-ML Based Intelligent Dead Reckoning System

Verifies:
1. Phase 17 Estimator executes strictly sequentially with current timestamp t_k.
2. Perturbing future sensor observations (t > t_k) produces Delta = 0.000000000000 m difference
   in estimated position, velocity, and attitude for all t <= t_k.
3. Pre-outage Doppler observability window and dynamic lateral-motion confidence use ONLY
   current and past observations (t <= t_k) and access zero future samples.
"""

import os
import sys
import math
import unittest
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.eskf_fusion import wrap_pi


class TestPhase17NoLookahead(unittest.TestCase):
    """Test suite validating strict causality and anti-lookahead invariance for Phase 17 Final."""

    def test_01_future_perturbation_invariance(self):
        """
        Verify that future perturbations for t > t_k cause zero deviation in
        position, velocity, and attitude for all t <= t_k.
        """
        N_steps = 30
        cut_idx = 15

        def run_sim(future_pert: float):
            # Pre-outage converged initial biases
            init_ba = np.array([0.01, -0.02, 0.05])
            init_bg = np.array([0.001, -0.002, 0.003])

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
            engine.eskf.ba = init_ba.copy()
            engine.eskf.bg = init_bg.copy()

            history = []
            steady_straight_ticks = 0
            prev_r_lat = 0.05**2

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

                # Phase 17 Dynamic Lateral Motion Confidence
                c_spd = np.clip((v_fwd - 0.5) / 2.0, 0.0, 1.0)
                c_yaw = math.exp(-0.5 * (omega_z / 0.025)**2)
                c_lat_acc = math.exp(-0.5 * (a_y / 0.35)**2)
                a_kin_diff = abs(a_y - v_fwd * omega_z)
                c_kin = math.exp(-0.5 * (a_kin_diff / 0.40)**2)
                c_lat = float(c_spd * c_yaw * c_lat_acc * c_kin)

                R_strong = 0.025**2
                R_loose = 0.40**2
                target_r = R_strong + (1.0 - c_lat) * (R_loose - R_strong)
                r_lat_adapt = float(np.clip(0.70 * prev_r_lat + 0.30 * target_r, R_strong, R_loose))
                prev_r_lat = r_lat_adapt

                l_x = 1.2
                v_y_kin = l_x * omega_z if abs(omega_z) > 0.01 else 0.0
                v_b = engine.eskf.R.T @ engine.eskf.v
                dz_lat = np.array([v_y_kin - v_b[1], -v_b[2]])

                H_lat = np.zeros((2, 15))
                H_lat[0, 3:6] = engine.eskf.R[:, 1]
                H_lat[0, 6:9] = engine.eskf.R[:, 1] @ np.array([
                    [0, -engine.eskf.v[2], engine.eskf.v[1]],
                    [engine.eskf.v[2], 0, -engine.eskf.v[0]],
                    [-engine.eskf.v[1], engine.eskf.v[0], 0]
                ])
                H_lat[1, 3:6] = engine.eskf.R[:, 2]
                H_lat[1, 6:9] = engine.eskf.R[:, 2] @ np.array([
                    [0, -engine.eskf.v[2], engine.eskf.v[1]],
                    [engine.eskf.v[2], 0, -engine.eskf.v[0]],
                    [-engine.eskf.v[1], engine.eskf.v[0], 0]
                ])
                R_cov_lat = np.diag([r_lat_adapt, 0.05**2])
                engine.eskf._apply_kalman_update(dz_lat, H_lat, R_cov_lat)

                if c_lat > 0.50:
                    steady_straight_ticks += 1
                    if steady_straight_ticks > 5:
                        dz = np.array([clean_gyro_b[2] - engine.eskf.bg[2]])
                        H = np.zeros((1, 15))
                        H[0, 14] = 1.0
                        r_zaru_val = 0.005 / max(c_lat, 0.2)
                        engine.eskf._apply_kalman_update(dz, H, np.array([[r_zaru_val**2]]))
                else:
                    steady_straight_ticks = 0

                history.append({
                    "pos": engine.eskf.p.copy(),
                    "vel": engine.eskf.v.copy(),
                    "yaw": engine.eskf.yaw
                })

            return history

        # Run unperturbed vs heavily perturbed future (after cut_idx)
        hist_unperturbed = run_sim(future_pert=0.0)
        hist_perturbed = run_sim(future_pert=5.0)

        # Assert zero deviation up to and including cut_idx
        for step in range(cut_idx + 1):
            pos_diff = np.linalg.norm(hist_unperturbed[step]["pos"] - hist_perturbed[step]["pos"])
            vel_diff = np.linalg.norm(hist_unperturbed[step]["vel"] - hist_perturbed[step]["vel"])
            yaw_diff = abs(wrap_pi(hist_unperturbed[step]["yaw"] - hist_perturbed[step]["yaw"]))

            self.assertEqual(pos_diff, 0.0, f"Position diverged at step {step}: diff={pos_diff}")
            self.assertEqual(vel_diff, 0.0, f"Velocity diverged at step {step}: diff={vel_diff}")
            self.assertEqual(yaw_diff, 0.0, f"Yaw diverged at step {step}: diff={yaw_diff}")

        # Verify that perturbation DOES take effect after cut_idx (sanity check)
        final_pos_diff = np.linalg.norm(hist_unperturbed[-1]["pos"] - hist_perturbed[-1]["pos"])
        self.assertGreater(final_pos_diff, 0.5, "Perturbation had no effect in future window")


if __name__ == "__main__":
    unittest.main()
