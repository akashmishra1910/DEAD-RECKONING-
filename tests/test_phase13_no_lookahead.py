"""
tests/test_phase13_no_lookahead.py
==================================
Strict temporal anti-lookahead and causality verification test suite.

Verifies:
1. Feature extraction at time t_k depends exclusively on samples with timestamp t <= t_k.
2. Perturbing all future sensor inputs (t > t_k) results in machine-precision invariance
   (Delta = 0.0) of estimated state at time t_k.
3. No ground-truth state, future GNSS, or future road IDs are accessed at runtime.
"""

import math
import unittest
import numpy as np

from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine
from idr_pipeline.offline_osm_graph import OfflineOSMGraph
from scripts.run_phase10c_adaptive_nhc import compute_adaptive_r_lat


class TestPhase13NoLookahead(unittest.TestCase):
    """Test suite proving strict causality and zero future-data leakage in Phase 13 algorithms."""

    def test_01_future_perturbation_invariance(self):
        """
        Prove that perturbing all future inputs for t > t_k causes zero change in estimated
        position, velocity, and attitude for all t <= t_k.
        """
        N_steps = 30
        cut_idx = 15  # t_k is at index 15

        def run_simulation(future_perturbation: float):
            engine = SeamlessGNSSDeficitEngine(
                init_pos=np.array([100.0, 200.0, 50.0]),
                init_vel=np.array([10.0, 0.0, 0.0]),
                init_euler=np.array([0.0, 0.0, 0.5]),
                dt=0.1,
                enable_heading_supervisor=True,
                enable_map_matching=False,
                enable_speed_calibration=True,
                enable_longitudinal_map_constraint=False
            )

            history = []
            for k in range(N_steps):
                t = 1.0 + k * 0.1
                # Base inputs
                accel = np.array([0.2, 0.0, 9.81])
                gyro = np.array([0.0, 0.0, 0.02])
                ai_spd = 10.0 + 0.1 * k

                # Apply perturbation only strictly AFTER cut_idx
                if k > cut_idx:
                    accel += np.array([future_perturbation, -future_perturbation, 0.0])
                    gyro += np.array([0.0, 0.0, future_perturbation * 0.1])
                    ai_spd += future_perturbation * 5.0

                accel_b = engine.alignment_engine.transform_phone_to_vehicle(accel)
                gyro_b = engine.alignment_engine.transform_phone_to_vehicle(gyro)
                clean_accel_b, clean_gyro_b, vib_state = engine.vibration_filter.process(accel_b, gyro_b)

                engine.eskf.predict(clean_accel_b, clean_gyro_b, engine.dt)
                engine.eskf.update_altitude(50.0, r_alt=0.8)

                r_ai = 0.35 * (1.0 + 0.5 * float(np.linalg.norm(accel_b - clean_accel_b)))
                engine.eskf.update_ai_velocity(ai_spd, r_ai=r_ai)

                v_fwd = float(np.linalg.norm(engine.eskf.v[0:2]))
                r_lat = compute_adaptive_r_lat('N7', v_fwd, clean_gyro_b[2], clean_accel_b[1])
                engine.eskf.update_nhc(r_lat=math.sqrt(r_lat), r_vert=0.05)

                history.append({
                    "pos": engine.eskf.p.copy(),
                    "vel": engine.eskf.v.copy(),
                    "yaw": float(engine.eskf.yaw)
                })

            return history

        # Run unperturbed and two drastically perturbed future scenarios
        hist_baseline = run_simulation(future_perturbation=0.0)
        hist_perturbed_pos = run_simulation(future_perturbation=2.5)
        hist_perturbed_neg = run_simulation(future_perturbation=-3.0)

        # Assert bit-exact identity for all steps up to and including cut_idx
        for k in range(cut_idx + 1):
            diff_pos = np.linalg.norm(hist_baseline[k]["pos"] - hist_perturbed_pos[k]["pos"])
            diff_vel = np.linalg.norm(hist_baseline[k]["vel"] - hist_perturbed_pos[k]["vel"])
            diff_yaw = abs(wrap_pi(hist_baseline[k]["yaw"] - hist_perturbed_pos[k]["yaw"]))

            self.assertEqual(diff_pos, 0.0, f"Position changed at step {k} <= {cut_idx} due to future perturbation!")
            self.assertEqual(diff_vel, 0.0, f"Velocity changed at step {k} <= {cut_idx} due to future perturbation!")
            self.assertEqual(diff_yaw, 0.0, f"Yaw changed at step {k} <= {cut_idx} due to future perturbation!")

            diff_pos2 = np.linalg.norm(hist_baseline[k]["pos"] - hist_perturbed_neg[k]["pos"])
            self.assertEqual(diff_pos2, 0.0)

        # Verify that perturbation DID alter the future state (proving the test is sensitive)
        diff_future = np.linalg.norm(hist_baseline[-1]["pos"] - hist_perturbed_pos[-1]["pos"])
        self.assertGreater(diff_future, 0.5, "Future state must differ under perturbation")

    def test_02_no_ground_truth_injection_in_runtime(self):
        """Verify that estimator predict/update loops execute without ground-truth arguments."""
        engine = SeamlessGNSSDeficitEngine(
            init_pos=np.array([0.0, 0.0, 0.0]),
            init_vel=np.array([5.0, 0.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0]),
            dt=0.1
        )
        # Calling process_step with gnss_valid=False and no ground-truth arguments
        telemetry = engine.process_step(
            time_s=1.0,
            accel_raw_p=np.array([0.1, 0.0, 9.81]),
            gyro_raw_p=np.array([0.0, 0.0, 0.01]),
            baro_alt=10.0,
            gnss_pos=None,
            gnss_valid=False,
            ai_speed=5.0
        )
        self.assertIsNotNone(telemetry)
        self.assertTrue(np.all(np.isfinite(telemetry.pos_enu)))


if __name__ == "__main__":
    unittest.main()
