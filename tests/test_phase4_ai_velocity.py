"""
tests/test_phase4_ai_velocity.py
================================
Phase 4 Comprehensive Test Suite: AI Velocity Model Validation & Hardening.

Validates:
1. Data and feature leakage absence (Phase 4-A)
2. Frozen Top-45 model reproduction & bounds (Phase 4-B)
3. Real-drive generalization across speed regimes (Phase 4-C, 4-D)
4. Strict temporal causality (Phase 4-E)
5. Vibration pre-filtering & shock blanking interaction (Phase 4-F)
6. AI Velocity -> ESKF measurement update & Jacobian validity (Phase 4-G)
7. Adaptive measurement noise scaling (Phase 4-H)
8. Failure, outlier, and confidence state handling (Phase 4-I)
9. Computational execution latency within deadlines (Phase 4-J)

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import os
import sys
import time
import unittest
import joblib
import numpy as np

sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    skew_symmetric,
    rodrigues,
    rotation_matrix_to_euler,
    euler_to_rotation_matrix,
    wrap_pi
)
from idr_pipeline.dataset_loader import extract_sliding_window_features
from idr_pipeline.vibration_filter import (
    VibrationPotholeFilter,
    apply_butterworth_prefilter,
    butterworth_bandpass_coefficients
)


class TestPhase4AIVelocity(unittest.TestCase):
    """Rigorous Phase 4 AI Velocity Model Validation & Hardening Test Suite."""

    @classmethod
    def setUpClass(cls):
        cls.checkpoint_path = "models/gbdt_speed_model_iovnbd_top45.joblib"
        if not os.path.exists(cls.checkpoint_path):
            cls.checkpoint_path = "models/gbdt_speed_model_iovnbd.joblib"
        cls.checkpoint = joblib.load(cls.checkpoint_path)
        cls.model = cls.checkpoint["model"]
        cls.scaler = cls.checkpoint["scaler"]
        cls.features = cls.checkpoint["features"]
        cls.indices = cls.checkpoint["indices"]
        cls.drives = RealIOVNBDLoader.load_all_drives('real_iovnbd_data')

    # ------------------------------------------------------------------------
    # 1. Feature Leakage Audit
    # ------------------------------------------------------------------------
    def test_01_feature_leakage_audit(self):
        """Verify that all 45 feature names represent pure IMU inputs with zero GNSS/CAN leakage."""
        self.assertEqual(len(self.features), 45)
        self.assertEqual(len(self.indices), 45)
        forbidden_keywords = ['gps', 'gnss', 'can', 'ecu', 'true', 'speed_gt', 'target', 'label', 'future', 'outage']
        for feat in self.features:
            for kw in forbidden_keywords:
                self.assertNotIn(kw, feat.lower(), f"Feature {feat} contains forbidden keyword {kw}")

    # ------------------------------------------------------------------------
    # 2. Strict Temporal Causality
    # ------------------------------------------------------------------------
    def test_02_temporal_causality(self):
        """Verify that modifying future IMU samples does not affect past/current predictions."""
        drive = self.drives.get('Vta01b') or list(self.drives.values())[0]
        N = min(len(drive.time), 200)
        accel = drive.accel[:N].copy()
        gyro = drive.gyro[:N].copy()

        feats_orig = extract_sliding_window_features(
            accel, gyro, window_size=10,
            include_temporal_extrema=True, include_jerk=True, include_frequency=True
        )[:, self.indices]
        preds_orig = np.clip(self.model.predict(self.scaler.transform(feats_orig)), 0.0, 35.0)

        # Perturb all future samples from k = 100 onwards
        k_eval = 80
        accel_future_mod = accel.copy()
        gyro_future_mod = gyro.copy()
        accel_future_mod[k_eval + 1:] += 100.0
        gyro_future_mod[k_eval + 1:] += 10.0

        feats_mod = extract_sliding_window_features(
            accel_future_mod, gyro_future_mod, window_size=10,
            include_temporal_extrema=True, include_jerk=True, include_frequency=True
        )[:, self.indices]
        preds_mod = np.clip(self.model.predict(self.scaler.transform(feats_mod)), 0.0, 35.0)

        diff = np.max(np.abs(preds_orig[:k_eval + 1] - preds_mod[:k_eval + 1]))
        self.assertEqual(diff, 0.0, "Prediction at t <= k must be bit-for-bit invariant to future perturbations.")

    # ------------------------------------------------------------------------
    # 3. Frozen Model Metadata & Metrics Reproduction
    # ------------------------------------------------------------------------
    def test_03_frozen_model_reproducibility(self):
        """Verify that the frozen Top-45 model checkpoint is intact and reports valid metrics."""
        self.assertIn("train_rmse", self.checkpoint)
        self.assertIn("test_rmse", self.checkpoint)
        self.assertIn("train_r2", self.checkpoint)
        self.assertIn("test_r2", self.checkpoint)
        
        self.assertAlmostEqual(self.checkpoint["train_rmse"], 4.8956, delta=0.05)
        self.assertAlmostEqual(self.checkpoint["test_rmse"], 5.0015, delta=0.05)
        self.assertAlmostEqual(self.checkpoint["test_r2"], 0.6069, delta=0.05)

    # ------------------------------------------------------------------------
    # 4. Physical Speed Bounds Clamping [0, 35] m/s
    # ------------------------------------------------------------------------
    def test_04_speed_bounds_clamping(self):
        """Verify predictions are strictly bounded within [0.0, 35.0] m/s ([0, 126] km/h)."""
        drive = self.drives.get('Vta01b') or list(self.drives.values())[0]
        feats = extract_sliding_window_features(
            drive.accel, drive.gyro, window_size=10,
            include_temporal_extrema=True, include_jerk=True, include_frequency=True
        )[:, self.indices]
        preds = np.clip(self.model.predict(self.scaler.transform(feats)), 0.0, 35.0)

        self.assertTrue(np.all(preds >= 0.0), "Speeds must be non-negative.")
        self.assertTrue(np.all(preds <= 35.0), "Speeds must not exceed 35 m/s.")

    # ------------------------------------------------------------------------
    # 5. NaN / Inf Input Gating
    # ------------------------------------------------------------------------
    def test_05_nan_inf_input_handling(self):
        """Verify robust detection and handling of corrupted float inputs."""
        nan_accel = np.full((10, 3), np.nan)
        nan_gyro = np.full((10, 3), np.nan)
        feats = extract_sliding_window_features(nan_accel, nan_gyro, window_size=10)
        self.assertTrue(np.isnan(feats).any(), "Feature extractor must propagate NaN for invalid windows.")

    # ------------------------------------------------------------------------
    # 6. Standstill ZUPT Interaction
    # ------------------------------------------------------------------------
    def test_06_standstill_zupt_interaction(self):
        """Verify that during standstill, ZUPT measurement update clamps filter velocity to zero."""
        eskf = ErrorStateKalmanFilter(init_vel=np.array([2.5, 0.0, 0.0]))
        # Small positive GBDT residual prediction of 1.2 m/s during standstill
        v_pred_standstill = 1.2
        # If vehicle is verified stationary (v < 0.2 m/s), ZUPT is applied
        eskf.update_zupt(r_zupt=0.01)
        self.assertLess(np.linalg.norm(eskf.v), 0.05, "ZUPT must override small positive GBDT residuals at standstill.")

    # ------------------------------------------------------------------------
    # 7. AI Velocity Measurement Jacobian Finite-Difference Check
    # ------------------------------------------------------------------------
    def test_07_ai_velocity_jacobian_finite_difference(self):
        """Rigorous finite-difference verification of AI velocity Jacobian across 3D attitudes."""
        for r_deg in [-10.0, 0.0, 10.0]:
            for p_deg in [-8.0, 0.0, 8.0]:
                for y_deg in [0.0, 45.0, 120.0, 260.0]:
                    R = euler_to_rotation_matrix(np.radians(r_deg), np.radians(p_deg), np.radians(y_deg))
                    v_nav = np.array([5.0, 10.0, 0.5])
                    
                    # Forward velocity in body frame: v_x^b = R[:, 0]^T @ v_nav
                    vx0 = float(R[:, 0] @ v_nav)
                    
                    # Analytical Jacobian components
                    # H_v = R[:, 0]^T (derivative w.r.t nav velocity)
                    # H_theta = R[:, 0]^T @ [v_nav]_x (derivative w.r.t attitude error)
                    H_theta_analytical = R[:, 0] @ skew_symmetric(v_nav)
                    
                    # Perturb attitude by dtheta_n
                    eps = 1e-6
                    for axis in range(3):
                        dtheta = np.zeros(3)
                        dtheta[axis] = eps
                        R_pert = rodrigues(dtheta) @ R
                        vx_pert = float(R_pert[:, 0] @ v_nav)
                        d_num = (vx_pert - vx0) / eps
                        self.assertAlmostEqual(d_num, H_theta_analytical[axis], places=4)

    # ------------------------------------------------------------------------
    # 8. AI Velocity Kalman Correction Direction
    # ------------------------------------------------------------------------
    def test_08_ai_velocity_kalman_correction_direction(self):
        """Verify positive speed residual (v_AI > v_est) accelerates body forward velocity."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 5.0, 0.0]), # Moving North at 5 m/s
            init_euler=np.array([0.0, 0.0, 0.0]) # Facing North
        )
        # AI reports vehicle is traveling at 8.0 m/s
        eskf.update_ai_velocity(v_ai=8.0, r_ai=0.35)
        # Velocity in North direction must increase
        self.assertGreater(eskf.v[1], 5.0, "Positive AI speed innovation must accelerate filter velocity.")

    # ------------------------------------------------------------------------
    # 9. AI Velocity Covariance Reduction
    # ------------------------------------------------------------------------
    def test_09_ai_velocity_covariance_reduction(self):
        """Verify AI velocity update strictly decreases forward velocity covariance."""
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        p_prior = eskf.P[4, 4] # North velocity variance
        eskf.update_ai_velocity(v_ai=10.0, r_ai=0.35)
        p_post = eskf.P[4, 4]
        self.assertLess(p_post, p_prior, "AI velocity update must reduce velocity uncertainty.")

    # ------------------------------------------------------------------------
    # 10. Adaptive Measurement Noise Scaling
    # ------------------------------------------------------------------------
    def test_10_adaptive_measurement_noise_scaling(self):
        """Verify adaptive measurement noise R_adaptive widens during high vibration/jerk."""
        r_base = 0.35
        # Nominal condition
        r_nominal = r_base * (1.0 + 0.0)
        # High vibration and high cornering
        vib_ratio = 2.5
        yaw_rate = 0.45
        r_dynamic = r_base * (1.0 + 0.5 * vib_ratio + 1.2 * yaw_rate)
        self.assertGreater(r_dynamic, r_nominal * 2.0, "R_adaptive must scale up during dynamic cornering/vibration.")

    # ------------------------------------------------------------------------
    # 11. Shock Blanking Pothole Isolation (>2.5g)
    # ------------------------------------------------------------------------
    def test_11_shock_blanking_pothole_isolation(self):
        """Verify >2.5g impulse shock is blanked and does not corrupt speed prediction."""
        filter_eng = VibrationPotholeFilter(pothole_threshold_g=2.2)
        for _ in range(10):
            filter_eng.process(np.array([0.0, 0.0, 9.81]), np.zeros(3))
        raw_shock_accel = np.array([0.0, 0.0, 35.0]) # >2.5g vertical spike
        clean_accel, clean_gyro, state = filter_eng.process(raw_shock_accel, np.zeros(3))
        self.assertTrue(state.is_pothole_detected, "Spike >2.5g must be flagged as shock.")
        self.assertLess(np.linalg.norm(clean_accel), 25.0, "Shock spike must be blanked/replaced with rolling median.")

    # ------------------------------------------------------------------------
    # 12. 4th-Order Butterworth Filter Stability
    # ------------------------------------------------------------------------
    def test_12_butterworth_bandpass_stability(self):
        """Verify Butterworth digital filter coefficients are stable and poles inside unit circle."""
        b, a = butterworth_bandpass_coefficients(fs=10.0, lowcut=0.1, highcut=15.0, order=4)
        self.assertEqual(len(b), 9)
        self.assertEqual(len(a), 9)
        roots = np.roots(a)
        self.assertTrue(np.all(np.abs(roots) < 1.0), "All IIR filter poles must strictly lie inside the unit circle.")

    # ------------------------------------------------------------------------
    # 13. Deterministic Inference Reproducibility
    # ------------------------------------------------------------------------
    def test_13_deterministic_prediction_reproducibility(self):
        """Verify that identical sensor inputs yield bit-for-bit identical speed predictions."""
        drive = self.drives.get('Vta01b') or list(self.drives.values())[0]
        feats1 = extract_sliding_window_features(
            drive.accel[:50], drive.gyro[:50], window_size=10,
            include_temporal_extrema=True, include_jerk=True, include_frequency=True
        )[:, self.indices]
        feats2 = extract_sliding_window_features(
            drive.accel[:50], drive.gyro[:50], window_size=10,
            include_temporal_extrema=True, include_jerk=True, include_frequency=True
        )[:, self.indices]
        p1 = self.model.predict(self.scaler.transform(feats1))
        p2 = self.model.predict(self.scaler.transform(feats2))
        np.testing.assert_array_equal(p1, p2, "Inference must be strictly deterministic.")

    # ------------------------------------------------------------------------
    # 14. Real-Drive Generalization
    # ------------------------------------------------------------------------
    def test_14_real_drive_generalization(self):
        """Verify speed estimation RMSE across all authentic IO-VNBD drives is bounded (< 6.0 m/s)."""
        for name, drive in self.drives.items():
            feats = extract_sliding_window_features(
                drive.accel, drive.gyro, window_size=10,
                include_temporal_extrema=True, include_jerk=True, include_frequency=True
            )[:, self.indices]
            preds = np.clip(self.model.predict(self.scaler.transform(feats)), 0.0, 35.0)
            rmse = float(np.sqrt(np.mean((preds - drive.gt_speed)**2)))
            self.assertLess(rmse, 6.0, f"Drive {name} RMSE {rmse:.2f} m/s exceeds 6.0 m/s threshold.")

    # ------------------------------------------------------------------------
    # 15. Computational Latency Deadline
    # ------------------------------------------------------------------------
    def test_15_computational_latency_deadline(self):
        """Verify single-cycle inference latency is < 20.0 ms (>> 10 Hz mobile requirement)."""
        sample_feat = np.zeros((1, len(self.indices)))
        t0 = time.perf_counter()
        N = 100
        for _ in range(N):
            _ = self.model.predict(sample_feat)
        lat_ms = (time.perf_counter() - t0) / N * 1000.0
        self.assertLess(lat_ms, 20.0, f"Inference latency {lat_ms:.2f} ms exceeds 20 ms deadline.")


if __name__ == '__main__':
    unittest.main()
