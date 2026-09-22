"""
tests/test_eskf_fusion.py
=========================
Dedicated unit and isolated validation suite for the upgraded 15-state
3D Attitude Error-State Kalman Filter (ESKF) for ISRO PS 26168.
"""

import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath('.'))

from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    rodrigues,
    skew_symmetric,
    renormalize_rotation_matrix
)
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine


class TestESKFFusion3D(unittest.TestCase):

    def setUp(self):
        np.random.seed(42)

    def test_01_quaternion_dcm_orthonormality(self):
        """TEST 1: Propagate through 500 arbitrary gyro steps; verify R remains strictly orthonormal."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([10.0, 0.0, 0.0]),
            init_euler=np.array([0.1, -0.05, 1.2])
        )
        dt = 0.05
        for _ in range(500):
            # Arbitrary 3-axis continuous angular rate
            gyro = np.array([
                0.05 * np.sin(0.1 * _),
                0.03 * np.cos(0.08 * _),
                0.12 * np.sin(0.05 * _)
            ])
            accel = np.array([0.2, 0.0, 9.81])
            eskf.predict(accel, gyro, dt)

        ortho_err = float(np.linalg.norm(eskf.R @ eskf.R.T - np.eye(3)))
        det_err = float(np.abs(np.linalg.det(eskf.R) - 1.0))  # Right-handed SO(3) convention det = +1.0
        self.assertLess(ortho_err, 1e-10, f"Orthonormality error {ortho_err:.2e} must be < 1e-10.")
        self.assertLess(det_err, 1e-10, f"Determinant magnitude error {det_err:.2e} must be < 1e-10.")
        print(f"  [PASS] Test 1: Orthonormality preserved across 500 steps (ortho_err={ortho_err:.2e}, det={np.linalg.det(eskf.R):.4f})")

    def test_02_zero_angular_velocity(self):
        """TEST 2: Initialize arbitrary attitude; propagate with zero gyro; verify attitude remains constant."""
        init_euler = np.array([0.25, -0.15, 0.85])
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([5.0, 0.0, 0.0]),
            init_euler=init_euler
        )
        R_orig = eskf.R.copy()
        dt = 0.1
        for _ in range(50):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.zeros(3), dt)

        diff_R = float(np.linalg.norm(eskf.R - R_orig))
        diff_euler = float(np.max(np.abs(np.array([eskf.roll, eskf.pitch, eskf.yaw]) - init_euler)))
        self.assertLess(diff_R, 1e-12, f"R drifted under zero gyro: {diff_R:.2e}")
        self.assertLess(diff_euler, 1e-12, f"Euler drifted under zero gyro: {diff_euler:.2e}")
        print(f"  [PASS] Test 2: Zero angular velocity constancy verified (max_diff={diff_R:.2e})")

    def test_03_pure_yaw_rotation(self):
        """TEST 3: Apply only z-axis angular rate; verify yaw changes while roll/pitch remain constant."""
        init_euler = np.array([0.0, 0.0, 0.5])
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=init_euler
        )
        w_z = 0.2  # 0.2 rad/s (positive = counter-clockwise around Up = left turn)
        dt = 0.1
        steps = 20  # Total angle = 0.4 rad
        for _ in range(steps):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.array([0.0, 0.0, w_z]), dt)

        # In right-handed FLU (+z Up), CCW rotation decreases clockwise azimuth yaw
        expected_yaw = 0.5 - w_z * dt * steps
        self.assertAlmostEqual(eskf.yaw, expected_yaw, places=4)
        self.assertAlmostEqual(eskf.roll, 0.0, places=5)
        self.assertAlmostEqual(eskf.pitch, 0.0, places=5)
        print(f"  [PASS] Test 3: Pure yaw rotation verified (yaw={eskf.yaw:.4f}, roll={eskf.roll:.4f}, pitch={eskf.pitch:.4f})")

    def test_04_pure_roll_rotation(self):
        """TEST 4: Apply only x-axis angular rate; verify roll changes (confirms 3D propagation vs yaw-only)."""
        init_euler = np.array([0.0, 0.0, 0.0])
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=init_euler
        )
        w_x = 0.15  # 0.15 rad/s
        dt = 0.1
        steps = 15  # Total angle = 0.225 rad
        for _ in range(steps):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.array([w_x, 0.0, 0.0]), dt)

        expected_roll = w_x * dt * steps
        self.assertAlmostEqual(eskf.roll, expected_roll, places=4)
        self.assertAlmostEqual(eskf.yaw, 0.0, places=4)
        self.assertAlmostEqual(eskf.pitch, 0.0, places=4)
        print(f"  [PASS] Test 4: Pure roll rotation verified (roll={eskf.roll:.4f} != 0.0, yaw={eskf.yaw:.4f})")

    def test_05_pure_pitch_rotation(self):
        """TEST 5: Apply only y-axis angular rate; verify pitch evolves according to kinematics."""
        init_euler = np.array([0.0, 0.0, 0.0])
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=init_euler
        )
        # In vehicle body coordinates, w_y < 0 pitches nose up (elevation > 0)
        w_y = -0.10
        dt = 0.1
        steps = 25  # Total angle = 0.25 rad
        for _ in range(steps):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.array([0.0, w_y, 0.0]), dt)

        expected_pitch = 0.25
        self.assertAlmostEqual(eskf.pitch, expected_pitch, places=3)
        self.assertAlmostEqual(eskf.roll, 0.0, places=4)
        self.assertAlmostEqual(eskf.yaw, 0.0, places=4)
        print(f"  [PASS] Test 5: Pure pitch rotation verified (pitch={eskf.pitch:.4f}, roll={eskf.roll:.4f})")

    def test_06_combined_3axis_rotation(self):
        """TEST 6: Apply simultaneous 3-axis gyro rates; verify R remains finite, orthogonal, and normalized."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=np.array([0.1, -0.2, 0.5])
        )
        dt = 0.02
        for _ in range(100):
            gyro = np.array([0.3, -0.2, 0.4])
            eskf.predict(np.array([0.0, 0.0, 9.80665]), gyro, dt)

        ortho_err = float(np.linalg.norm(eskf.R @ eskf.R.T - np.eye(3)))
        self.assertFalse(np.isnan(eskf.R).any())
        self.assertFalse(np.isinf(eskf.R).any())
        self.assertLess(ortho_err, 1e-10)
        print(f"  [PASS] Test 6: Combined 3-axis rotation (ortho_err={ortho_err:.2e})")

    def test_07_attitude_error_injection(self):
        """TEST 7: Verify all three attitude components dx[6:9] inject properly (no discarded roll/pitch)."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=np.array([0.0, 0.0, 0.0])
        )
        R_before = eskf.R.copy()
        # Synthetic residual with H observing attitude directly
        H = np.zeros((3, 15))
        H[0:3, 6:9] = np.eye(3)
        dz = np.array([0.05, -0.04, 0.08])  # Direct error in roll, pitch, yaw
        R_cov = np.eye(3) * 0.001

        eskf._apply_kalman_update(dz, H, R_cov)

        R_after = eskf.R.copy()
        diff_R = float(np.linalg.norm(R_after - R_before))
        self.assertGreater(diff_R, 0.05, "Attitude matrix must change significantly upon 3D correction.")

        # Check that roll and pitch both updated (not just yaw)
        self.assertNotAlmostEqual(eskf.roll, 0.0, places=2, msg="Roll must be updated by dx[6].")
        self.assertNotAlmostEqual(eskf.pitch, 0.0, places=2, msg="Pitch must be updated by dx[7].")
        self.assertNotAlmostEqual(eskf.yaw, 0.0, places=2, msg="Yaw must be updated by dx[8].")
        print(f"  [PASS] Test 7: 3D attitude error injection (roll={eskf.roll:.4f}, pitch={eskf.pitch:.4f}, yaw={eskf.yaw:.4f})")

    def test_08_ai_velocity_update(self):
        """TEST 8: Verify AI forward velocity update dimensions (1x15) and execution without NaN/Inf."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([12.0, 0.0, 0.0]),
            init_euler=np.array([0.05, -0.02, 1.57])
        )
        # Apply AI velocity observation (v_ai = 12.5 m/s)
        eskf.update_ai_velocity(12.5, r_ai=0.35)
        self.assertFalse(np.isnan(eskf.v).any())
        self.assertFalse(np.isnan(eskf.P).any())
        self.assertFalse(np.isinf(eskf.P).any())
        print(f"  [PASS] Test 8: AI velocity update executed successfully (v_updated={eskf.v})")

    def test_09_nhc_measurement_update(self):
        """TEST 9: Verify Non-Holonomic Constraints dimensions (2x15) and zero lateral/vertical velocity."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([10.0, 1.5, -0.8]),  # Artificial sideways skid and bounce
            init_euler=np.array([0.0, 0.0, 1.57079])  # Facing East
        )
        # Update NHC multiple times to damp non-forward velocity
        for _ in range(5):
            eskf.update_nhc(r_lat=0.01, r_vert=0.01)

        v_b = eskf.R.T @ eskf.v
        self.assertLess(abs(v_b[1]), 0.5, "Lateral velocity must be strongly attenuated by NHC.")
        self.assertLess(abs(v_b[2]), 0.5, "Vertical velocity must be strongly attenuated by NHC.")
        self.assertFalse(np.isnan(eskf.P).any())
        print(f"  [PASS] Test 9: NHC measurement update verified (v_lateral_body={v_b[1]:.4f}, v_vert_body={v_b[2]:.4f})")

    def test_10_zupt_measurement_update(self):
        """TEST 10: Verify Standstill Zero Velocity Update (3x15) collapses velocity and variance."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([2.5, -1.0, 0.4]),
            init_euler=np.zeros(3)
        )
        v_norm_prior = float(np.linalg.norm(eskf.v))
        eskf.update_zupt(r_zupt=0.005)
        v_norm_post = float(np.linalg.norm(eskf.v))

        self.assertLess(v_norm_post, 0.1 * v_norm_prior, "ZUPT must drastically reduce velocity magnitude.")
        self.assertLess(eskf.P[3, 3], 0.01, "Velocity variance must collapse upon ZUPT.")
        print(f"  [PASS] Test 10: ZUPT verified (velocity {v_norm_prior:.3f} m/s -> {v_norm_post:.4f} m/s)")

    def test_11_covariance_stability_and_symmetry(self):
        """TEST 11: Run repeated predict/update cycles; verify P remains finite, symmetric, and positive-definite."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([10.0, 0.0, 0.0]),
            init_euler=np.array([0.05, 0.02, 1.2])
        )
        dt = 0.1
        for step in range(100):
            accel = np.array([0.1 * np.sin(0.1*step), 0.05 * np.cos(0.1*step), 9.80665])
            gyro = np.array([0.02 * np.cos(0.05*step), 0.01 * np.sin(0.05*step), 0.05])
            eskf.predict(accel, gyro, dt)
            eskf.update_ai_velocity(10.0, r_ai=0.35)
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)
            if step % 10 == 0:
                eskf.update_altitude(0.0, r_alt=0.8)

        # 1. Symmetry check: ||P - P.T||
        symm_err = float(np.max(np.abs(eskf.P - eskf.P.T)))
        self.assertLess(symm_err, 1e-12, f"Covariance asymmetry {symm_err:.2e} must be < 1e-12.")

        # 2. Positive definiteness: min eigenvalue > 0 (or within numerical zero tolerance)
        eigvals = np.linalg.eigvalsh(eskf.P)
        min_eig = float(np.min(eigvals))
        self.assertGreater(min_eig, -1e-10, f"Covariance has severe negative eigenvalue: {min_eig:.2e}")
        self.assertFalse(np.isnan(eskf.P).any())
        self.assertFalse(np.isinf(eskf.P).any())
        print(f"  [PASS] Test 11: Covariance stability (symm_err={symm_err:.2e}, min_eig={min_eig:.2e})")

    def test_12_synthetic_maneuver_trajectory(self):
        """TEST 12: Deterministic synthetic trajectory (straight -> accelerate -> brake -> turn -> pitch -> halt)."""
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([10.0, 0.0, 0.0]),
            init_euler=np.array([0.0, 0.0, np.pi/2]), # East
            acc_noise=0.10,
            gyro_noise=0.005
        )
        dt = 0.1

        # Phase 1: Straight cruise (20 steps)
        for _ in range(20):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.zeros(3), dt)
            eskf.update_ai_velocity(10.0, r_ai=0.35)
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)

        # Phase 2: Left turn (20 steps @ 0.1 rad/s)
        for _ in range(20):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.array([0.0, 0.0, 0.1]), dt)
            eskf.update_ai_velocity(10.0, r_ai=0.35)
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)

        # Phase 3: Pitch incline (10 steps @ -0.05 rad/s)
        for _ in range(10):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.array([0.0, -0.05, 0.0]), dt)
            eskf.update_ai_velocity(10.0, r_ai=0.35)
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)

        # Phase 4: Standstill (10 steps)
        for _ in range(10):
            eskf.predict(np.array([0.0, 0.0, 9.80665]), np.zeros(3), dt)
            eskf.update_zupt(r_zupt=0.005)

        self.assertFalse(np.isnan(eskf.p).any())
        self.assertFalse(np.isnan(eskf.v).any())
        self.assertLess(np.linalg.norm(eskf.v), 0.1, "Velocity after ZUPT must be halted.")
        print(f"  [PASS] Test 12: Synthetic multi-maneuver trajectory completed (final_p={eskf.p}, v_norm={np.linalg.norm(eskf.v):.4f})")

    def test_13_alignment_to_eskf_turning_integration(self):
        """TEST 13: End-to-end integration: In-Vehicle Alignment Engine (FLU) -> ESKF (ENU).
        Verifies:
        1. Alignment engine produces right-handed R_p2b with det(R_p2b) = +1.0.
        2. ESKF initial R_b2n has det(R_b2n) = +1.0.
        3. A left turn from East (yaw = pi/2) correctly decreases heading azimuth towards North (yaw = 0).
        4. Transformed velocities and positions evolve in the physically correct direction without left/right inversion.
        """
        from scipy.spatial.transform import Rotation

        # 1. Synthetic phone mounted in cradle with realistic roll, pitch, yaw relative to vehicle FLU
        # Vehicle body: Forward (+x), Left (+y), Up (+z)
        R_b2p_true = Rotation.from_euler('xyz', [12.0, -18.0, 35.0], degrees=True).as_matrix()
        R_p2b_true = R_b2p_true.T

        # Generate synthetic calibration data for batch alignment
        N_cal = 400
        a_veh = np.zeros((N_cal, 3))
        a_veh[:, 2] = 9.80665  # Gravity reaction (+Up)
        a_veh[50:150, 0] = 1.2  # Forward acceleration surge
        a_veh[200:300, 0] = -1.5 # Braking deceleration

        # Sensor readings in phone frame
        a_phone = (R_b2p_true @ a_veh.T).T + np.random.normal(0, 0.01, (N_cal, 3))

        # Estimate mounting matrix R_p2b via batch alignment
        R_p2b_est = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(a_phone)

        # Assert alignment matrix properties
        det_p2b = float(np.linalg.det(R_p2b_est))
        ortho_p2b = float(np.linalg.norm(R_p2b_est @ R_p2b_est.T - np.eye(3)))
        self.assertAlmostEqual(det_p2b, 1.0, places=6, msg="Alignment engine R_p2b must have det = +1.0.")
        self.assertLess(ortho_p2b, 1e-10, msg="Alignment engine R_p2b must be strictly orthogonal.")

        # 2. Initialize ESKF facing East (yaw = pi/2 = 90 deg)
        init_pos = np.array([0.0, 0.0, 0.0])
        init_vel = np.array([10.0, 0.0, 0.0])  # Driving East at 10 m/s
        init_euler = np.array([0.0, 0.0, np.pi / 2])
        eskf = ErrorStateKalmanFilter(init_pos=init_pos, init_vel=init_vel, init_euler=init_euler)

        det_b2n_init = float(np.linalg.det(eskf.R))
        self.assertAlmostEqual(det_b2n_init, 1.0, places=6, msg="ESKF R_b2n must have det = +1.0.")

        # 3. Simulate vehicle executing a left turn towards North
        # In FLU (+z Up), a left turn corresponds to counter-clockwise angular rate: omega_z > 0
        w_veh_turn = np.array([0.0, 0.0, 0.15])  # 0.15 rad/s CCW
        # Corresponding angular rate in phone frame: vec_phone = R_b2p @ vec_body
        w_phone_turn = R_b2p_true @ w_veh_turn

        dt = 0.1
        N_turn_steps = 25  # Total turn = 0.375 rad (~21.5 deg)
        for _ in range(N_turn_steps):
            # Transform phone gyro to vehicle body frame using calibrated R_p2b_est
            w_b_meas = R_p2b_est @ w_phone_turn
            a_b_meas = np.array([0.0, 0.0, 9.80665])  # Gravity reaction

            eskf.predict(a_b_meas, w_b_meas, dt)
            eskf.update_ai_velocity(10.0, r_ai=0.35)
            eskf.update_nhc(r_lat=0.05, r_vert=0.05)

        # 4. Verification
        det_b2n_final = float(np.linalg.det(eskf.R))
        ortho_b2n_final = float(np.linalg.norm(eskf.R @ eskf.R.T - np.eye(3)))
        self.assertAlmostEqual(det_b2n_final, 1.0, places=6, msg="ESKF R_b2n must preserve det = +1.0 during turn.")
        self.assertLess(ortho_b2n_final, 1e-10)

        # In ENU with clockwise azimuth, turning left from East (90 deg) turns toward North (0 deg)
        # Therefore yaw MUST decrease
        self.assertLess(eskf.yaw, np.pi / 2, "Turning left from East must decrease yaw azimuth towards North.")
        # Vehicle velocity in navigation frame must have positive North component (v_y > 0)
        self.assertGreater(eskf.v[1], 1.0, "Velocity after left turn from East must have positive North component.")
        # Vehicle position must have positive North displacement
        self.assertGreater(eskf.p[1], 1.0, "Position after left turn must displace positively in North direction.")

        print(f"  [PASS] Test 13: Alignment->ESKF integration (det_p2b={det_p2b:.4f}, det_b2n={det_b2n_final:.4f}, final_yaw={np.degrees(eskf.yaw):.1f} deg, v_North={eskf.v[1]:.2f} m/s)")

    def test_14_heading_measurement_convergence(self):
        """TEST 14: Verify magnetometer / compass heading measurement update in ENU/FLU.
        Ensures posterior yaw converges monotonically towards the observed target heading.
        """
        init_yaw = 0.50  # rad
        eskf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([5.0, 0.0, 0.0]),
            init_euler=np.array([0.0, 0.0, init_yaw])
        )

        target_heading = 0.65  # rad (+0.15 rad offset)
        eskf.update_heading(target_heading, r_yaw=0.03)

        self.assertGreater(eskf.yaw, init_yaw, "Posterior yaw must move towards target heading.")
        self.assertLessEqual(eskf.yaw, target_heading, "Posterior yaw must not overshoot unreasonably.")
        self.assertAlmostEqual(np.linalg.det(eskf.R), 1.0, places=6, msg="Attitude must remain in SO(3).")
        print(f"  [PASS] Test 14: Heading update convergence (prior={init_yaw:.3f} -> post={eskf.yaw:.3f}, target={target_heading:.3f})")


if __name__ == "__main__":
    print("=" * 80)
    print("RUNNING ISOLATED 3D ATTITUDE ESKF TEST SUITE")
    print("=" * 80)
    unittest.main()
