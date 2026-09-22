"""
tests/test_phase2_alignment_initialization.py
=============================================
Phase 2 Comprehensive Test Suite: In-Vehicle Alignment & Initialization Hardening.

Validates:
1. Clean Stationary Initialization & Leveling
2. Noisy Stationary Initialization
3. Stationary Gyroscope Bias Estimation & Variance
4. Multi-Orientation Gravity Leveling (100 Random Phone Orientations)
5. Clean Forward Surge Dynamic Alignment (PCA Eigenvector)
6. Stationary Start Protection (Insufficient Surge Rejection — KF-02 Fix)
7. Heading Degeneracy & Isotropic Noise Detection
8. Braking Transient Alignment & Forward Sign Resolution
9. Turning Start Dynamics Resolution
10. Engine Harmonic Vibration Immunity during Stillness
11. Cradle Mount Slip & Bump Watchdog Triggering (<4s Recalibration)
12. Phone Tap / Touch Impulse Rejection during Stillness
13. Corrupted NaN / Inf Sensor Sample Sanitization
14. Short & Empty Sample Window Handling
15. Mounting Matrix SO(3) Invariants (det = +1.0, R @ R.T = I across 200 mountings)
16. Machine-Readable AlignmentQuality Diagnostics Completeness
17. Full Multi-Mode Timeline (Stationary -> Surge -> Cruise)
18. Seamless Alignment -> ESKF Handoff (Body Frame Rotation & Bias Injection)
19. Deterministic Initialization Reproducibility
20. Real IO-VNBD Naturalistic Drive Startup Validation

Author: Lead Navigation Systems Engineer
"""

import unittest
import numpy as np
from typing import Tuple

from idr_pipeline.alignment_engine import (
    InVehicleAlignmentEngine,
    AlignmentMode,
    AlignmentQuality,
    AlignmentState,
    renormalize_so3,
    sanitize_imu_sample,
)
from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
)
from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader


class TestPhase2AlignmentInitialization(unittest.TestCase):
    """Phase 2 Alignment & Initialization Hardening Test Suite."""

    def test_01_stationary_initialization_clean(self):
        """Verify clean stationary initialization correctly determines gravity axis and roll/pitch."""
        engine = InVehicleAlignmentEngine(leveling_samples=25, dt=0.1)
        
        # Phone tilted 30 deg pitch up, 10 deg roll right
        R_true_p2b = euler_to_rotation_matrix(np.radians(10.0), np.radians(30.0), 0.0)
        # Gravity in phone frame is R_true_p2b.T @ [0, 0, 9.80665]
        g_phone = R_true_p2b.T @ np.array([0.0, 0.0, 9.80665])

        for _ in range(30):
            state = engine.process_sample(
                accel_p=g_phone,
                gyro_p=np.zeros(3),
                is_stationary=True
            )

        self.assertTrue(state.is_leveled)
        self.assertFalse(state.is_aligned) # Not aligned in yaw yet
        self.assertEqual(state.mode, AlignmentMode.STATIONARY_LEVELED)
        self.assertGreater(state.quality.gravity_confidence, 0.9)
        self.assertEqual(state.quality.forward_axis_confidence, 0.0)

        # Verify estimated z_b in phone frame matches true gravity direction
        z_b_est = engine.R_p2b[2, :] # Third row of R_p2b is z_b in phone coordinates
        cos_sim = np.dot(z_b_est, g_phone / np.linalg.norm(g_phone))
        self.assertAlmostEqual(cos_sim, 1.0, places=5)

    def test_02_noisy_stationary_initialization(self):
        """Verify gravity estimation with realistic Gaussian accelerometer noise (sigma = 0.1 m/s^2)."""
        engine = InVehicleAlignmentEngine(leveling_samples=50, dt=0.1)
        np.random.seed(42)

        g_true = np.array([1.5, -2.0, 9.4])
        g_true = g_true / np.linalg.norm(g_true) * 9.80665

        for _ in range(60):
            noise = np.random.randn(3) * 0.1
            engine.process_sample(
                accel_p=g_true + noise,
                gyro_p=np.random.randn(3) * 0.005,
                is_stationary=True
            )

        self.assertTrue(engine.state.is_leveled)
        z_est = engine.R_p2b[2, :]
        z_true = g_true / np.linalg.norm(g_true)
        angle_err_deg = float(np.degrees(np.arccos(np.clip(np.dot(z_est, z_true), -1.0, 1.0))))
        self.assertLess(angle_err_deg, 0.5, f"Gravity leveling error too high: {angle_err_deg:.3f} deg")

    def test_03_gyro_bias_estimation(self):
        """Verify stationary gyroscope bias estimation and variance computation."""
        engine = InVehicleAlignmentEngine(leveling_samples=40, dt=0.1)
        np.random.seed(123)

        true_bg_phone = np.array([0.015, -0.025, 0.008])
        for _ in range(50):
            noise = np.random.randn(3) * 0.001
            engine.process_sample(
                accel_p=np.array([0.0, 0.0, 9.80665]),
                gyro_p=true_bg_phone + noise,
                is_stationary=True
            )

        self.assertTrue(engine.state.is_leveled)
        est_bg_body = engine.state.quality.initial_gyro_bias
        # Since R_p2b is horizontal leveled, transform true_bg to body and compare
        expected_bg_body = engine.state.R_p2b @ true_bg_phone
        np.testing.assert_allclose(est_bg_body, expected_bg_body, atol=1e-3)
        self.assertGreater(engine.state.quality.gyro_bias_variance, 0.0)

    def test_04_gravity_leveling_various_phone_orientations(self):
        """Verify gravity leveling across 100 arbitrary 3D phone mounting orientations."""
        np.random.seed(999)
        for _ in range(100):
            # Random Euler angles
            r = np.random.uniform(-np.pi, np.pi)
            p = np.random.uniform(-np.pi * 0.4, np.pi * 0.4)
            y = np.random.uniform(-np.pi, np.pi)
            R_true = euler_to_rotation_matrix(r, p, y)

            g_phone = R_true.T @ np.array([0.0, 0.0, 9.80665])
            accel_batch = np.tile(g_phone, (30, 1))

            R_est, quality = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
                accel_phone=accel_batch,
                return_quality=True
            )

            # Check SO(3) invariants
            self.assertAlmostEqual(np.linalg.det(R_est), 1.0, places=10)
            ortho_err = np.linalg.norm(R_est @ R_est.T - np.eye(3))
            self.assertLess(ortho_err, 1e-12)

            # Check z_b matches true gravity
            z_est = R_est[2, :]
            z_true = g_phone / np.linalg.norm(g_phone)
            self.assertAlmostEqual(float(np.dot(z_est, z_true)), 1.0, places=5)

    def test_05_clean_surge_excitation_forward_axis(self):
        """Verify dynamic forward surge alignment resolves forward body axis x_b cleanly."""
        engine = InVehicleAlignmentEngine(leveling_samples=20, min_accel_transient=0.4, dt=0.1)

        # Phone mounted facing Forward with 20 deg pitch down
        R_mount = euler_to_rotation_matrix(0.0, np.radians(-20.0), 0.0)
        g_p = R_mount.T @ np.array([0.0, 0.0, 9.80665])

        # 1. Leveling phase (25 samples)
        for _ in range(25):
            engine.process_sample(accel_p=g_p, gyro_p=np.zeros(3), is_stationary=True)

        self.assertTrue(engine.state.is_leveled)
        self.assertFalse(engine.state.is_aligned)

        # 2. Dynamic forward surge: 1.5 m/s^2 forward in vehicle body frame
        a_surge_body = np.array([1.5, 0.0, 9.80665])
        a_surge_phone = R_mount.T @ a_surge_body

        for _ in range(20):
            engine.process_sample(accel_p=a_surge_phone, gyro_p=np.zeros(3), is_stationary=False)

        self.assertTrue(engine.state.is_aligned)
        self.assertEqual(engine.state.mode, AlignmentMode.DYNAMIC_ALIGNED)
        self.assertTrue(engine.state.quality.alignment_valid)
        self.assertGreater(engine.state.quality.forward_axis_confidence, 0.5)

        # Verify forward vector in body frame: R_p2b @ (R_mount.T @ [1, 0, 0]) ≈ [1, 0, 0]
        x_body_recovered = engine.transform_phone_to_vehicle(R_mount.T @ np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(x_body_recovered, [1.0, 0.0, 0.0], atol=1e-3)

    def test_06_stationary_start_protection_insufficient_surge(self):
        """
        Verify Stationary Start Protection (Fix for Known Failure KF-02):
        When vehicle remains stationary, engine does NOT invent an arbitrary forward axis.
        """
        engine = InVehicleAlignmentEngine(leveling_samples=25, dt=0.1)
        np.random.seed(55)

        # 10 seconds of stationary stillness with minor sensor noise
        for _ in range(100):
            noise = np.random.randn(3) * 0.02
            state = engine.process_sample(
                accel_p=np.array([0.0, 0.0, 9.80665]) + noise,
                gyro_p=np.random.randn(3) * 0.001,
                is_stationary=True
            )

        self.assertTrue(state.is_leveled)
        self.assertFalse(state.is_aligned, "Engine incorrectly declared aligned during stillness!")
        self.assertFalse(state.quality.alignment_valid)
        self.assertEqual(state.quality.forward_axis_confidence, 0.0)
        self.assertEqual(state.quality.heading_confidence, 0.0)
        self.assertEqual(state.mode, AlignmentMode.STATIONARY_LEVELED)

    def test_07_heading_degeneracy_and_isotropic_noise_rejection(self):
        """Verify isotropic horizontal vibration is rejected without declaring alignment valid."""
        np.random.seed(77)
        # Isotropic horizontal noise
        accel_noise = np.random.randn(100, 3) * 0.1
        accel_noise[:, 2] += 9.80665

        R_est, quality = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            accel_phone=accel_noise,
            return_quality=True
        )

        self.assertFalse(quality.alignment_valid, "Isotropic noise should not produce valid forward axis")
        self.assertEqual(quality.forward_axis_confidence, 0.0)
        self.assertAlmostEqual(np.linalg.det(R_est), 1.0, places=10)

    def test_08_braking_transient_alignment(self):
        """Verify forward axis sign resolution during a braking transient."""
        np.random.seed(88)
        N = 100
        # Simulated drive: deceleration phase is sharper than acceleration
        accel = np.zeros((N, 3))
        accel[:, 2] = 9.80665
        # Forward acceleration: +0.5 m/s^2 for 30 steps
        accel[10:40, 0] = 0.5
        # Sharp braking deceleration: -2.0 m/s^2 for 15 steps
        accel[50:65, 0] = -2.0

        R_p2b, quality = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            accel_phone=accel,
            return_quality=True
        )

        self.assertTrue(quality.alignment_valid)
        # Check forward axis points in direction of forward surge (+X)
        self.assertGreater(R_p2b[0, 0], 0.8, "Forward sign was inverted during braking analysis")

    def test_09_turning_start_dynamics(self):
        """Verify alignment handling when vehicle turns shortly after moving."""
        engine = InVehicleAlignmentEngine(leveling_samples=20, min_accel_transient=0.4, dt=0.1)

        # 1. Leveling
        for _ in range(25):
            engine.process_sample(
                accel_p=np.array([0.0, 0.0, 9.80665]),
                gyro_p=np.zeros(3),
                is_stationary=True
            )

        # 2. Forward surge + centripetal left turn
        for _ in range(20):
            engine.process_sample(
                accel_p=np.array([1.2, 0.6, 9.80665]), # Forward 1.2, Left 0.6 m/s^2
                gyro_p=np.array([0.0, 0.0, 0.1]),
                is_stationary=False
            )

        self.assertTrue(engine.state.is_aligned)
        self.assertTrue(engine.state.quality.alignment_valid)

    def test_10_engine_harmonic_vibration_immunity(self):
        """Verify gravity leveling accuracy under 25 Hz engine idling vibration (0.8 m/s^2 RMS)."""
        engine = InVehicleAlignmentEngine(leveling_samples=50, dt=0.01) # 100 Hz sampling
        t = np.linspace(0, 1.0, 100)

        for i in range(100):
            vib_25hz = 0.8 * np.sin(2 * np.pi * 25.0 * t[i])
            acc = np.array([vib_25hz * 0.3, vib_25hz * 0.3, 9.80665 + vib_25hz])
            engine.process_sample(accel_p=acc, gyro_p=np.zeros(3), is_stationary=True)

        self.assertTrue(engine.state.is_leveled)
        z_est = engine.R_p2b[2, :]
        np.testing.assert_allclose(z_est, [0.0, 0.0, 1.0], atol=0.05)

    def test_11_cradle_slip_watchdog_trigger(self):
        """Verify watchdog detects cradle slip (>5 deg tilt) and triggers fast recalibration."""
        engine = InVehicleAlignmentEngine(leveling_samples=20, slip_threshold_deg=5.0, dt=0.1)

        # 1. Complete leveling and dynamic alignment
        for _ in range(25):
            engine.process_sample(accel_p=np.array([0.0, 0.0, 9.80665]), gyro_p=np.zeros(3), is_stationary=True)
        for _ in range(20):
            engine.process_sample(accel_p=np.array([1.0, 0.0, 9.80665]), gyro_p=np.zeros(3), is_stationary=False)

        self.assertTrue(engine.state.is_aligned)

        # 2. Phone bumps in cradle: tilt changes by 15 degrees
        R_bump = euler_to_rotation_matrix(np.radians(15.0), 0.0, 0.0)
        g_bumped = R_bump.T @ np.array([0.0, 0.0, 9.80665])

        state_after_bump = None
        for _ in range(10):
            state_after_bump = engine.process_sample(accel_p=g_bumped, gyro_p=np.zeros(3), is_stationary=True)

        self.assertEqual(state_after_bump.mode, AlignmentMode.RECALIBRATING)
        self.assertFalse(state_after_bump.is_aligned)

    def test_12_phone_tap_impulse_rejection_during_leveling(self):
        """Verify phone taps during stillness are filtered from stationary buffers."""
        engine = InVehicleAlignmentEngine(leveling_samples=25, dt=0.1)

        # 20 normal stationary samples
        for _ in range(20):
            engine.process_sample(accel_p=np.array([0.0, 0.0, 9.80665]), gyro_p=np.zeros(3), is_stationary=True)

        # User taps phone (sharp angular jerk w = 1.5 rad/s)
        engine.process_sample(
            accel_p=np.array([2.0, 1.0, 15.0]),
            gyro_p=np.array([1.5, -1.2, 0.8]),
            is_stationary=True
        )

        # 10 more stationary samples
        for _ in range(10):
            engine.process_sample(accel_p=np.array([0.0, 0.0, 9.80665]), gyro_p=np.zeros(3), is_stationary=True)

        self.assertTrue(engine.state.is_leveled)
        # Verify initial gyro bias did not absorb the 1.5 rad/s tap
        self.assertLess(np.linalg.norm(engine.state.quality.initial_gyro_bias), 0.05)

    def test_13_corrupted_nan_inf_sensor_samples(self):
        """Verify engine sanitizes NaNs and Infs in sensor streams without crashing."""
        engine = InVehicleAlignmentEngine(leveling_samples=20, dt=0.1)

        # Feed contaminated vectors
        for i in range(30):
            if i == 10:
                acc = np.array([np.nan, 0.0, 9.80665])
                gyr = np.array([0.0, np.inf, 0.0])
            elif i == 15:
                acc = np.array([0.0, -np.inf, np.nan])
                gyr = np.array([np.nan, np.nan, np.nan])
            else:
                acc = np.array([0.0, 0.0, 9.80665])
                gyr = np.zeros(3)

            state = engine.process_sample(accel_p=acc, gyro_p=gyr, is_stationary=True)

        self.assertFalse(np.isnan(state.R_p2b).any())
        self.assertFalse(np.isinf(state.R_p2b).any())
        self.assertAlmostEqual(np.linalg.det(state.R_p2b), 1.0, places=10)

    def test_14_short_and_empty_sample_windows(self):
        """Verify short or empty sample windows fall back safely to identity matrix."""
        R_empty = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel_phone=np.zeros((0, 3)))
        np.testing.assert_allclose(R_empty, np.eye(3))

        R_short, q_short = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            accel_phone=np.array([[0.0, 0.0, 9.8]]),
            return_quality=True
        )
        np.testing.assert_allclose(R_short, np.eye(3))
        self.assertFalse(q_short.alignment_valid)

    def test_15_mounting_matrix_so3_invariants(self):
        """Verify det(R_p2b) = +1.0 and R @ R.T = I across 200 arbitrary batch estimates."""
        np.random.seed(333)
        for _ in range(200):
            r = np.random.uniform(-np.pi, np.pi)
            p = np.random.uniform(-np.pi * 0.4, np.pi * 0.4)
            R_true = euler_to_rotation_matrix(r, p, 0.0)

            accel = np.random.randn(80, 3) * 0.2
            # Add forward surge
            accel[20:50, 0] += 1.5
            accel[:, 2] += 9.80665
            accel_p = accel @ R_true

            R_est = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel_phone=accel_p)
            det_val = np.linalg.det(R_est)
            ortho_err = np.linalg.norm(R_est @ R_est.T - np.eye(3))

            self.assertAlmostEqual(det_val, 1.0, places=10)
            self.assertLess(ortho_err, 1e-12)

    def test_16_alignment_quality_diagnostics_completeness(self):
        """Verify all fields in AlignmentQuality are correctly populated and bounded."""
        accel = np.zeros((100, 3))
        accel[:, 2] = 9.80665
        accel[30:60, 0] = 1.5
        gyr = np.zeros((100, 3)) + 0.005

        R_p2b, quality = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
            accel_phone=accel,
            gyro_phone=gyr,
            return_quality=True
        )

        self.assertIsInstance(quality, AlignmentQuality)
        self.assertGreaterEqual(quality.gravity_confidence, 0.0)
        self.assertLessEqual(quality.gravity_confidence, 1.0)
        self.assertGreaterEqual(quality.forward_axis_confidence, 0.0)
        self.assertLessEqual(quality.forward_axis_confidence, 1.0)
        self.assertTrue(quality.alignment_valid)
        self.assertLess(quality.det_residual, 1e-10)
        self.assertLess(quality.orthogonality_residual, 1e-12)
        self.assertGreater(quality.eigenvalue_ratio, 2.0)
        self.assertGreater(quality.peak_surge_accel, 1.0)

    def test_17_full_multimode_timeline(self):
        """Verify complete progression: INITIALIZING -> STATIONARY_LEVELED -> DYNAMIC_ALIGNED."""
        engine = InVehicleAlignmentEngine(leveling_samples=20, dt=0.1)

        # 1. Step 0: INITIALIZING
        self.assertEqual(engine.state.mode, AlignmentMode.INITIALIZING)

        # 2. Stationary 25 steps: STATIONARY_LEVELED
        for _ in range(25):
            engine.process_sample(accel_p=np.array([0.0, 0.0, 9.80665]), gyro_p=np.zeros(3), is_stationary=True)
        self.assertEqual(engine.state.mode, AlignmentMode.STATIONARY_LEVELED)

        # 3. Surge acceleration 20 steps: DYNAMIC_ALIGNED
        for _ in range(20):
            engine.process_sample(accel_p=np.array([1.5, 0.0, 9.80665]), gyro_p=np.zeros(3), is_stationary=False)
        self.assertEqual(engine.state.mode, AlignmentMode.DYNAMIC_ALIGNED)
        self.assertTrue(engine.state.is_aligned)

    def test_18_alignment_to_eskf_handoff(self):
        """Verify end-to-end handoff from InVehicleAlignmentEngine into ErrorStateKalmanFilter."""
        engine = InVehicleAlignmentEngine(leveling_samples=20, dt=0.1)

        # Leveling + Gyro bias
        true_bg = np.array([0.01, -0.01, 0.02])
        for _ in range(25):
            engine.process_sample(
                accel_p=np.array([0.0, 0.0, 9.80665]),
                gyro_p=true_bg,
                is_stationary=True
            )
        # Surge
        for _ in range(20):
            engine.process_sample(
                accel_p=np.array([1.5, 0.0, 9.80665]),
                gyro_p=true_bg,
                is_stationary=False
            )

        self.assertTrue(engine.state.quality.alignment_valid)
        R_p2b = engine.state.R_p2b
        init_bg_body = engine.state.quality.initial_gyro_bias

        # Initialize ESKF with initial gyro bias from alignment
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=np.zeros(3),
            init_bg=init_bg_body
        )

        # Predict step using body frame transformed IMU
        raw_phone_acc = np.array([1.0, 0.0, 9.80665])
        raw_phone_gyr = true_bg.copy()

        acc_body = R_p2b @ raw_phone_acc
        gyr_body = R_p2b @ raw_phone_gyr

        kf.predict(accel_meas=acc_body, gyro_meas=gyr_body, dt=0.1)

        # Because init_bg matched gyr_body, angular rate unbiases to zero and roll/pitch/yaw remain zero
        self.assertLess(np.linalg.norm([kf.roll, kf.pitch, kf.yaw]), 1e-5)
        # Forward acceleration unbiases cleanly
        self.assertAlmostEqual(kf.v[1], 0.1, places=3) # North velocity = 1.0 m/s^2 * 0.1s

    def test_19_deterministic_initialization_reproducibility(self):
        """Verify that identical sensor inputs yield bit-for-bit identical alignment outputs."""
        np.random.seed(42)
        accel = np.random.randn(50, 3) + np.array([0.5, 0.0, 9.8])

        R1, q1 = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, return_quality=True)
        R2, q2 = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(accel, return_quality=True)

        np.testing.assert_array_equal(R1, R2)
        self.assertEqual(q1.gravity_confidence, q2.gravity_confidence)
        self.assertEqual(q1.forward_axis_confidence, q2.forward_axis_confidence)

    def test_20_real_iovnbd_startup_validation(self):
        """Verify alignment engine executes cleanly on all authentic IO-VNBD drives in real_iovnbd_data."""
        drives = RealIOVNBDLoader.load_all_drives("real_iovnbd_data")
        self.assertGreater(len(drives), 0)

        for name, drive in drives.items():
            N = min(200, len(drive.accel))
            R_p2b, quality = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
                accel_phone=drive.accel[:N],
                gyro_phone=drive.gyro[:N],
                speed=drive.gt_speed[:N],
                return_quality=True
            )

            # Assertions on real driving data
            self.assertFalse(np.isnan(R_p2b).any(), f"NaN in R_p2b for {name}")
            self.assertAlmostEqual(np.linalg.det(R_p2b), 1.0, places=6, msg=f"Invalid det for {name}")
            ortho_err = np.linalg.norm(R_p2b @ R_p2b.T - np.eye(3))
            self.assertLess(ortho_err, 1e-6, f"Ortho error for {name}")
            self.assertGreater(quality.gravity_confidence, 0.5, f"Low gravity confidence for {name}")


if __name__ == '__main__':
    unittest.main()
