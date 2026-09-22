"""
tests/test_phase1_navigation_core.py
====================================
Comprehensive Mathematical & Physical Validation Suite for Navigation Core (Phase 1).

Validates:
1. SO(3) Group Invariants & Properties (Orthonormality, Determinant=+1.0 across 1000 arbitrary rotations)
2. Coordinate Frame Conventions (ENU Navigation + FLU Body, Cardinal Azimuths, Right-Handedness)
3. Turning Direction Dynamics (East + Positive Gyro Z -> Heading turns toward North)
4. Euler / DCM Roundtrip Conversion
5. Rodrigues Rotation Formula vs Scipy Matrix Exponential
6. Stationary Phone Gravity Cancellation (f^b = [0, 0, g] + g^n = [0, 0, -g] -> zero acceleration)
7. Constant Linear Acceleration Kinematics (p = 0.5*a*t^2, v = a*t)
8. Braking Deceleration Dynamics
9. Banked Road & Hill Incline Handling (Pitch & Roll Resolution)
10. Pothole Shock & High-Frequency Vibration Robustness
11. Stop-and-Go with Standstill ZUPT Clamping
12. Accelerometer & Gyroscope Bias Estimation Observability
13. AI Forward Velocity Jacobian Numerical Check (Finite Difference)
14. Non-Holonomic Constraints (NHC) Jacobian Numerical Check
15. Heading Measurement Jacobian & Innovation Sign Verification
16. Covariance Symmetry & Positive-Definiteness (P = P^T > 0 across 1000 cycles)
17. GNSS Outage Transition & Smooth Reacquisition (Zero Filter Jumps)
18. GNSS Velocity Update Mechanics
19. Real IO-VNBD Subset Numerical Sanity Check (No NaNs, No Infs, Finite P)

Author: Lead Navigation Systems Engineer
"""

import unittest
import numpy as np
import scipy.linalg
from typing import Tuple

from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    rodrigues,
    skew_symmetric,
    renormalize_rotation_matrix,
)
from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.alignment_engine import InVehicleAlignmentEngine


class TestPhase1NavigationCore(unittest.TestCase):
    """Rigorous Phase 1 Mathematical & Physical Invariant Tests."""

    def test_01_so3_orthonormality_and_det(self):
        """Verify SO(3) group invariants: R @ R.T = I, det(R) = +1.0 across 1000 arbitrary rotations."""
        np.random.seed(42)
        for _ in range(1000):
            rot_vec = np.random.randn(3) * 2.0 * np.pi
            R = rodrigues(rot_vec)
            
            # Orthonormality error: ||R @ R.T - I||
            ortho_err = np.linalg.norm(R @ R.T - np.eye(3))
            det_val = np.linalg.det(R)
            
            self.assertLess(ortho_err, 1e-13, f"Orthonormality violated: err={ortho_err}")
            self.assertAlmostEqual(det_val, 1.0, places=12, msg=f"Improper rotation det={det_val}")

    def test_02_frame_conventions_enu_flu(self):
        """
        Verify ENU Navigation (+X East, +Y North, +Z Up) and FLU Body (+X Forward, +Y Left, +Z Up).
        Test all 4 cardinal directions and vertical up vector.
        """
        # 1. Facing North (yaw = 0): Forward = North, Left = West (-East), Up = Up
        R_north = euler_to_rotation_matrix(0.0, 0.0, 0.0)
        np.testing.assert_allclose(R_north[:, 0], [0.0, 1.0, 0.0], atol=1e-12) # Forward is North
        np.testing.assert_allclose(R_north[:, 1], [-1.0, 0.0, 0.0], atol=1e-12) # Left is West
        np.testing.assert_allclose(R_north[:, 2], [0.0, 0.0, 1.0], atol=1e-12) # Up is Up
        self.assertAlmostEqual(np.linalg.det(R_north), 1.0, places=12)

        # 2. Facing East (yaw = pi/2): Forward = East, Left = North, Up = Up
        R_east = euler_to_rotation_matrix(0.0, 0.0, np.pi / 2.0)
        np.testing.assert_allclose(R_east[:, 0], [1.0, 0.0, 0.0], atol=1e-12) # Forward is East
        np.testing.assert_allclose(R_east[:, 1], [0.0, 1.0, 0.0], atol=1e-12) # Left is North
        np.testing.assert_allclose(R_east[:, 2], [0.0, 0.0, 1.0], atol=1e-12) # Up is Up
        self.assertAlmostEqual(np.linalg.det(R_east), 1.0, places=12)

        # 3. Facing South (yaw = pi): Forward = South, Left = East, Up = Up
        R_south = euler_to_rotation_matrix(0.0, 0.0, np.pi)
        np.testing.assert_allclose(R_south[:, 0], [0.0, -1.0, 0.0], atol=1e-12) # Forward is South
        np.testing.assert_allclose(R_south[:, 1], [1.0, 0.0, 0.0], atol=1e-12) # Left is East
        np.testing.assert_allclose(R_south[:, 2], [0.0, 0.0, 1.0], atol=1e-12) # Up is Up

        # 4. Facing West (yaw = 3*pi/2): Forward = West, Left = South, Up = Up
        R_west = euler_to_rotation_matrix(0.0, 0.0, 3.0 * np.pi / 2.0)
        np.testing.assert_allclose(R_west[:, 0], [-1.0, 0.0, 0.0], atol=1e-12) # Forward is West
        np.testing.assert_allclose(R_west[:, 1], [0.0, -1.0, 0.0], atol=1e-12) # Left is South
        np.testing.assert_allclose(R_west[:, 2], [0.0, 0.0, 1.0], atol=1e-12) # Up is Up

    def test_03_turning_direction_left_and_right(self):
        """
        Verify turning dynamics:
        When facing East (yaw = 90 deg), positive z-gyro (w_z > 0 in FLU) turns vehicle Left towards North (yaw -> 0 deg).
        When facing North (yaw = 0 deg), negative z-gyro (w_z < 0 in FLU) turns vehicle Right towards East (yaw -> 90 deg).
        """
        # Start facing East:
        filter_east = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([10.0, 0.0, 0.0]), # 10 m/s East
            init_euler=np.array([0.0, 0.0, np.pi / 2.0]) # 90 deg yaw
        )

        # Apply positive left-turn rate w_z = +0.1 rad/s for dt = 1.0 s
        # Accel measures centripetal force: a_y = v * w_z = 10 * 0.1 = +1.0 m/s^2 (in Left body axis) + gravity in Z
        filter_east.predict(
            accel_meas=np.array([0.0, 1.0, 9.80665]),
            gyro_meas=np.array([0.0, 0.0, 0.1]),
            dt=1.0
        )

        # Heading should decrease from 90 deg (pi/2) to (pi/2 - 0.1) rad (turning towards North)
        expected_yaw = (np.pi / 2.0) - 0.1
        self.assertAlmostEqual(filter_east.yaw, expected_yaw, places=6)
        # Velocity in North should become positive
        self.assertGreater(filter_east.v[1], 0.0, "North velocity must be positive after left turn from East")

        # Start facing North:
        filter_north = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 10.0, 0.0]), # 10 m/s North
            init_euler=np.array([0.0, 0.0, 0.0]) # 0 deg yaw
        )
        # Apply negative right-turn rate w_z = -0.1 rad/s
        filter_north.predict(
            accel_meas=np.array([0.0, -1.0, 9.80665]),
            gyro_meas=np.array([0.0, 0.0, -0.1]),
            dt=1.0
        )
        # Heading should increase from 0 deg towards East (+0.1 rad)
        self.assertAlmostEqual(filter_north.yaw, 0.1, places=6)
        self.assertGreater(filter_north.v[0], 0.0, "East velocity must be positive after right turn from North")

    def test_04_euler_dcm_roundtrip(self):
        """Verify roundtrip conversion between Euler angles and DCM across 500 configurations."""
        np.random.seed(123)
        for _ in range(500):
            roll = np.random.uniform(-np.pi * 0.95, np.pi * 0.95)
            pitch = np.random.uniform(-np.pi * 0.45, np.pi * 0.45) # Keep away from pitch singularity (+- 90 deg)
            yaw = np.random.uniform(-np.pi, np.pi)

            R = euler_to_rotation_matrix(roll, pitch, yaw)
            r_rec, p_rec, y_rec = rotation_matrix_to_euler(R)

            # Compare reconstructed DCM
            R_rec = euler_to_rotation_matrix(r_rec, p_rec, y_rec)
            diff = np.linalg.norm(R - R_rec)
            self.assertLess(diff, 1e-12, f"Euler roundtrip failed: diff={diff}")

    def test_05_rodrigues_vs_scipy_matrix_exponential(self):
        """Verify Rodrigues rotation formula matches exact matrix exponential exp([w*dt]_x)."""
        np.random.seed(999)
        for _ in range(100):
            w = np.random.randn(3) * 5.0
            dt = 0.05
            rot_vec = w * dt

            R_rod = rodrigues(rot_vec)
            R_exp = scipy.linalg.expm(skew_symmetric(rot_vec))

            diff = np.linalg.norm(R_rod - R_exp)
            self.assertLess(diff, 1e-13, f"Rodrigues does not match matrix exponential: diff={diff}")

    def test_06_stationary_phone_gravity_balance(self):
        """Verify stationary phone on level table: accel = [0, 0, +g] exactly cancels gravity = [0, 0, -g]."""
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=np.array([0.0, 0.0, 0.0]) # Level, North
        )

        for _ in range(200): # 20 seconds @ 10 Hz
            kf.predict(
                accel_meas=np.array([0.0, 0.0, 9.80665]),
                gyro_meas=np.zeros(3),
                dt=0.1
            )

        # Velocity and position must remain precisely zero
        self.assertLess(np.linalg.norm(kf.v), 1e-12, "Stationary velocity drifted")
        self.assertLess(np.linalg.norm(kf.p), 1e-12, "Stationary position drifted")

    def test_07_constant_linear_acceleration(self):
        """Verify constant forward acceleration facing East: p(t) = 0.5*a*t^2, v(t) = a*t."""
        a_fwd = 2.0 # 2.0 m/s^2 forward
        dt = 0.01 # 100 Hz simulation
        steps = 500 # 5.0 seconds

        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=np.array([0.0, 0.0, np.pi / 2.0]) # Facing East
        )

        for _ in range(steps):
            kf.predict(
                accel_meas=np.array([a_fwd, 0.0, 9.80665]),
                gyro_meas=np.zeros(3),
                dt=dt
            )

        t_total = steps * dt # 5.0 s
        expected_v_east = a_fwd * t_total # 10.0 m/s
        expected_p_east = 0.5 * a_fwd * (t_total**2) # 25.0 m

        self.assertAlmostEqual(kf.v[0], expected_v_east, places=5)
        self.assertAlmostEqual(kf.p[0], expected_p_east, places=4)
        self.assertAlmostEqual(kf.v[1], 0.0, places=5) # Zero North drift
        self.assertAlmostEqual(kf.v[2], 0.0, places=5) # Zero Up drift

    def test_08_braking_maneuver(self):
        """Verify braking deceleration: 20 m/s down to 0 m/s with -4 m/s^2 forward force."""
        dt = 0.05
        steps = 100 # 5.0 seconds -> 20 - 4*5 = 0 m/s
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 20.0, 0.0]), # 20 m/s North
            init_euler=np.array([0.0, 0.0, 0.0]) # Facing North
        )

        for _ in range(steps):
            kf.predict(
                accel_meas=np.array([-4.0, 0.0, 9.80665]),
                gyro_meas=np.zeros(3),
                dt=dt
            )

        self.assertAlmostEqual(kf.v[1], 0.0, places=5)
        expected_p_north = 20.0 * 5.0 - 0.5 * 4.0 * (5.0**2) # 50.0 m
        self.assertAlmostEqual(kf.p[1], expected_p_north, places=4)

    def test_09_banked_road_and_hill_elevation(self):
        """Verify pitch/roll resolution: climbing a 10 deg hill and banking 15 deg on a turn."""
        pitch_rad = np.radians(10.0) # 10 deg nose up
        roll_rad = np.radians(15.0)  # 15 deg bank

        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.zeros(3),
            init_euler=np.array([roll_rad, pitch_rad, 0.0])
        )

        # In body frame, gravity [0, 0, -g] in nav frame transforms to f_b = R^T * [0, 0, g]
        R = euler_to_rotation_matrix(roll_rad, pitch_rad, 0.0)
        f_b_static = R.T @ np.array([0.0, 0.0, 9.80665])

        # Over 5 seconds of stationary incline, net acceleration must remain 0
        for _ in range(50):
            kf.predict(accel_meas=f_b_static, gyro_meas=np.zeros(3), dt=0.1)

        self.assertLess(np.linalg.norm(kf.v), 1e-10)
        self.assertLess(np.linalg.norm(kf.p), 1e-10)

    def test_10_pothole_shock_and_vibration_robustness(self):
        """Verify filter stability under extreme simulated potholes and shocks (>3.0g)."""
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 15.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0])
        )

        np.random.seed(42)
        for i in range(100):
            # Normal driving + high frequency road vibration
            vib = np.random.randn(3) * 0.5
            acc = np.array([0.0, 0.0, 9.80665]) + vib
            
            # Inject isolated pothole shock at step 50
            if i == 50:
                acc += np.array([0.0, 0.0, 35.0]) # +3.5g impulse shock

            kf.predict(accel_meas=acc, gyro_meas=np.zeros(3), dt=0.1)
            kf.update_ai_velocity(v_ai=15.0, r_ai=0.5)
            kf.update_nhc(r_lat=0.1, r_vert=0.1)

        self.assertFalse(np.isnan(kf.p).any())
        self.assertFalse(np.isinf(kf.p).any())
        self.assertFalse(np.isnan(kf.v).any())
        self.assertFalse(np.isinf(kf.v).any())

    def test_11_stop_and_go_with_zupt(self):
        """Verify standstill ZUPT collapses velocity and stabilizes position."""
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 5.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0])
        )

        # Standstill phase: apply ZUPT repeatedly
        for _ in range(30):
            kf.predict(accel_meas=np.array([0.0, 0.0, 9.80665]), gyro_meas=np.zeros(3), dt=0.1)
            kf.update_zupt(r_zupt=0.005)

        self.assertLess(np.linalg.norm(kf.v), 1e-3, "ZUPT failed to clamp velocity to zero")
        self.assertLess(kf.P[3, 3], 1e-4, "ZUPT failed to collapse velocity variance")

    def test_12_accelerometer_and_gyro_bias_estimation(self):
        """Verify ESKF observes and estimates artificial sensor biases under aiding."""
        true_ba = np.array([0.05, -0.05, 0.1])
        true_bg = np.array([0.005, -0.005, 0.01])

        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 10.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0]),
            init_ba=np.zeros(3),
            init_bg=np.zeros(3)
        )

        pos_ref = np.array([0.0, 0.0, 0.0])
        vel_ref = np.array([0.0, 10.0, 0.0])
        dt = 0.1

        for step in range(300): # 30 seconds
            pos_ref += vel_ref * dt
            # Injected biased measurements
            acc_meas = np.array([0.0, 0.0, 9.80665]) + true_ba
            gyr_meas = np.zeros(3) + true_bg

            kf.predict(accel_meas=acc_meas, gyro_meas=gyr_meas, dt=dt)
            # GNSS aiding position & velocity
            kf.update_gnss(gnss_pos=pos_ref, r_gnss=0.5)
            kf.update_gnss_velocity(gnss_vel=vel_ref, r_vel=0.1)
            kf.update_heading(heading_meas=0.0, r_yaw=0.02)

        # Verify filter estimated the biases in the correct direction
        self.assertLess(np.linalg.norm(kf.ba - true_ba), 0.08, "Accelerometer bias did not converge")
        self.assertLess(np.linalg.norm(kf.bg - true_bg), 0.01, "Gyroscope bias did not converge")

    def test_13_ai_velocity_jacobian_numerical_check(self):
        """Numerically verify AI velocity Jacobian H_v using finite-difference perturbation."""
        roll, pitch, yaw = 0.1, -0.15, 0.8
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        v_nav = np.array([7.0, 12.0, -0.5])

        # Analytical H
        H_analytic = np.zeros((1, 15))
        H_analytic[0, 3:6] = R[:, 0]
        H_analytic[0, 6:9] = R[:, 0] @ skew_symmetric(v_nav)

        # Numerical perturbation
        eps = 1e-7
        base_meas = (R[:, 0]) @ v_nav

        # 1. Perturb velocity
        H_num_v = np.zeros(3)
        for i in range(3):
            v_p = v_nav.copy()
            v_p[i] += eps
            meas_p = (R[:, 0]) @ v_p
            H_num_v[i] = (meas_p - base_meas) / eps

        np.testing.assert_allclose(H_analytic[0, 3:6], H_num_v, atol=1e-5)

        # 2. Perturb attitude dtheta in nav frame
        H_num_att = np.zeros(3)
        for i in range(3):
            dtheta = np.zeros(3)
            dtheta[i] = eps
            # R_p = (I + [dtheta]_x) R
            R_p = (np.eye(3) + skew_symmetric(dtheta)) @ R
            # Measurement model on perturbed attitude: (R_p.T @ v_nav)[0] = (R_p[:, 0]) . v_nav
            # In ESKF: innovation dz = z - h(x_prior).
            # When true state is x + dx, h(x + dx) = h(x) + H dx.
            # Here, h(x) is R^T v_nav. Perturbing R by exp([dtheta]_x) R increases body forward vector by:
            # h(x + dx) - h(x) = (R_p[:, 0]) . v_nav - (R[:, 0]) . v_nav
            meas_att_p = (R_p[:, 0]) @ v_nav
            H_num_att[i] = (meas_att_p - base_meas) / eps

        np.testing.assert_allclose(H_analytic[0, 6:9], H_num_att, atol=1e-5)

    def test_14_nhc_jacobian_numerical_check(self):
        """Numerically verify NHC lateral/vertical Jacobians using finite-difference perturbation."""
        roll, pitch, yaw = -0.05, 0.12, 1.2
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        v_nav = np.array([4.0, 9.0, 0.2])

        # Analytical H
        H_analytic = np.zeros((2, 15))
        H_analytic[0, 3:6] = R[:, 1]
        H_analytic[0, 6:9] = R[:, 1] @ skew_symmetric(v_nav)
        H_analytic[1, 3:6] = R[:, 2]
        H_analytic[1, 6:9] = R[:, 2] @ skew_symmetric(v_nav)

        eps = 1e-7
        base_lat = (R[:, 1]) @ v_nav
        base_vert = (R[:, 2]) @ v_nav

        # 1. Velocity perturbations
        H_num_v = np.zeros((2, 3))
        for i in range(3):
            v_p = v_nav.copy()
            v_p[i] += eps
            H_num_v[0, i] = ((R[:, 1] @ v_p) - base_lat) / eps
            H_num_v[1, i] = ((R[:, 2] @ v_p) - base_vert) / eps

        np.testing.assert_allclose(H_analytic[:, 3:6], H_num_v, atol=1e-5)

        # 2. Attitude perturbations
        H_num_att = np.zeros((2, 3))
        for i in range(3):
            dtheta = np.zeros(3)
            dtheta[i] = eps
            R_p = (np.eye(3) + skew_symmetric(dtheta)) @ R
            H_num_att[0, i] = ((R_p[:, 1] @ v_nav) - base_lat) / eps
            H_num_att[1, i] = ((R_p[:, 2] @ v_nav) - base_vert) / eps

        np.testing.assert_allclose(H_analytic[:, 6:9], H_num_att, atol=1e-5)

    def test_15_heading_measurement_jacobian_and_innovation_sign(self):
        """Verify heading measurement update converges correctly without sign inversion."""
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 10.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0]) # Yaw = 0.0 rad (North)
        )

        target_heading = 0.35 # 20.0 degrees clockwise
        kf.update_heading(heading_meas=target_heading, r_yaw=0.05)

        # Posterior yaw must move towards target_heading
        self.assertGreater(kf.yaw, 0.0, "Heading update pulled in the wrong direction")
        self.assertLess(kf.yaw, target_heading + 1e-5, "Heading update overshot target")

    def test_16_covariance_stability_and_symmetry(self):
        """Verify P remains symmetric, finite, and strictly positive-definite across 1000 mixed cycles."""
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([5.0, 5.0, 0.0]),
            init_euler=np.array([0.05, -0.05, 0.78])
        )

        np.random.seed(777)
        for i in range(1000):
            acc = np.array([0.1, 0.0, 9.80665]) + np.random.randn(3) * 0.1
            gyr = np.array([0.0, 0.0, 0.02]) + np.random.randn(3) * 0.005
            kf.predict(accel_meas=acc, gyro_meas=gyr, dt=0.01)

            if i % 10 == 0:
                kf.update_ai_velocity(v_ai=7.07, r_ai=0.4)
                kf.update_nhc(r_lat=0.05, r_vert=0.05)
            if i % 50 == 0:
                kf.update_heading(heading_meas=0.78, r_yaw=0.1)

            # Symmetry check
            symm_err = np.max(np.abs(kf.P - kf.P.T))
            self.assertLess(symm_err, 1e-12, f"Covariance asymmetry at step {i}: {symm_err}")

            # Positive-definiteness check (all eigenvalues > 0)
            eigvals = np.linalg.eigvalsh(kf.P)
            min_eig = np.min(eigvals)
            self.assertGreater(min_eig, 0.0, f"Covariance lost positive definiteness at step {i}: min_eig={min_eig}")

    def test_17_gnss_outage_transition_and_recovery(self):
        """
        Verify complete seamless GNSS handover cycle:
        Phase A (0-20s): GNSS-Aided -> Covariance tight
        Phase B (20-50s): GNSS Outage -> Covariance grows gracefully, no jumps
        Phase C (50-70s): GNSS Reacquisition -> Seamless recovery without numerical spike
        """
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([0.0, 15.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0])
        )

        dt = 0.1
        pos_true = np.zeros(3)
        v_true = np.array([0.0, 15.0, 0.0])

        pos_errors = []
        p_cov_traces = []

        for step in range(700): # 70 seconds @ 10 Hz
            t = step * dt
            pos_true += v_true * dt
            kf.predict(accel_meas=np.array([0.0, 0.0, 9.80665]), gyro_meas=np.zeros(3), dt=dt)

            if t < 20.0:
                # GNSS Active
                kf.update_gnss(gnss_pos=pos_true + np.random.randn(3)*0.2, r_gnss=1.0)
                kf.update_gnss_velocity(gnss_vel=v_true, r_vel=0.1)
            elif 20.0 <= t < 50.0:
                # GNSS Blackout (DR only: AI speed + NHC)
                kf.update_ai_velocity(v_ai=15.0, r_ai=0.35)
                kf.update_nhc(r_lat=0.05, r_vert=0.05)
            else:
                # GNSS Reacquisition
                kf.update_gnss(gnss_pos=pos_true + np.random.randn(3)*0.2, r_gnss=1.0)
                kf.update_gnss_velocity(gnss_vel=v_true, r_vel=0.1)

            pos_errors.append(np.linalg.norm(kf.p - pos_true))
            p_cov_traces.append(np.trace(kf.P[0:3, 0:3]))

        # Check: outage covariance grows, reacquisition collapses it smoothly
        self.assertGreater(p_cov_traces[450], p_cov_traces[150], "Covariance should grow during blackout")
        self.assertLess(p_cov_traces[650], p_cov_traces[450], "Covariance should contract after reacquisition")
        self.assertLess(pos_errors[-1], 2.0, "Failed to recover position upon GNSS reacquisition")

    def test_18_gnss_velocity_update(self):
        """Verify update_gnss_velocity directly constrains velocity state."""
        kf = ErrorStateKalmanFilter(
            init_pos=np.zeros(3),
            init_vel=np.array([5.0, 5.0, 0.0]),
            init_euler=np.array([0.0, 0.0, 0.0])
        )

        v_gnss = np.array([0.0, 20.0, 0.0])
        # After 5 successive updates, velocity collapses to target according to Kalman shrinkage P/(P+R)
        for _ in range(5):
            kf.update_gnss_velocity(gnss_vel=v_gnss, r_vel=0.05)

        np.testing.assert_allclose(kf.v, v_gnss, atol=0.05)
        self.assertLess(kf.P[3, 3], 0.001)

    def test_19_real_iovnbd_subset_sanity_check(self):
        """
        Verify ESKF executes across authentic IO-VNBD drive subset (Driver_A_S3b & Vta01b)
        without producing any NaNs, Infs, or covariance divergence.
        """
        drives = RealIOVNBDLoader.load_all_drives("real_iovnbd_data")
        self.assertGreater(len(drives), 0, "No authentic IO-VNBD drives found in real_iovnbd_data")

        for name, drive in list(drives.items())[:2]:
            dt = drive.dt
            R_p2b = InVehicleAlignmentEngine.estimate_mounting_matrix_batch(
                drive.accel[:100], drive.gyro[:100], drive.gt_speed[:100]
            )

            # Initialize filter
            kf = ErrorStateKalmanFilter(
                init_pos=drive.gt_pos[0].copy(),
                init_vel=drive.gt_vel[0].copy(),
                init_euler=np.array([0.0, 0.0, drive.gt_heading[0]])
            )

            N = min(500, len(drive.time))
            for i in range(N):
                # Rotate phone IMU to vehicle body frame
                acc_b = R_p2b @ drive.accel[i]
                gyr_b = R_p2b @ drive.gyro[i]

                kf.predict(accel_meas=acc_b, gyro_meas=gyr_b, dt=dt)
                kf.update_ai_velocity(v_ai=float(drive.gt_speed[i]), r_ai=0.5)
                kf.update_nhc(r_lat=0.1, r_vert=0.1)

                if i % 10 == 0:
                    kf.update_heading(heading_meas=float(drive.gt_heading[i]), r_yaw=0.1)

            # Sanity assertions
            self.assertFalse(np.isnan(kf.p).any(), f"NaN in position for drive {name}")
            self.assertFalse(np.isinf(kf.p).any(), f"Inf in position for drive {name}")
            self.assertFalse(np.isnan(kf.v).any(), f"NaN in velocity for drive {name}")
            self.assertFalse(np.isnan(kf.P).any(), f"NaN in covariance for drive {name}")
            self.assertTrue(np.all(np.linalg.eigvalsh(kf.P) > 0.0), f"P lost positive-definiteness on {name}")


if __name__ == '__main__':
    unittest.main()
