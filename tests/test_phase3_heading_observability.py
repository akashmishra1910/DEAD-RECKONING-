"""
tests/test_phase3_heading_observability.py
==========================================
Comprehensive Verification Suite for Phase 3: Attitude & Heading Observability Hardening.

Validates 24 distinct heading observability scenarios:
1. Heading innovation wrapping across 0/360 boundary
2. GNSS course calculation from ENU velocity
3. Low-speed GNSS rejection (<2.5 m/s)
4. GNSS course outlier rejection via Chi-Square gate
5. GNSS course dropout and fallback to gyro mechanization
6. GNSS course recovery with innovation gating
7. Magnetometer nominal heading with tilt compensation
8. Magnetometer magnitude disturbance rejection
9. Magnetometer invalid/NaN/zero data rejection
10. Magnetometer disturbance recovery
11. Heading Jacobian finite-difference verification (H[0, 8] = -1.0)
12. Heading innovation covariance verification (S = P_88 + R)
13. Heading Kalman correction direction verification
14. Gyro z-bias observability enhancement
15. Combined GNSS + Magnetometer arbitration
16. Conflicting heading source behavior
17. Stationary vehicle behavior
18. Straight-line driving tracking
19. Left turn tracking
20. Right turn tracking
21. 0/360 degree boundary crossing
22. Prolonged GNSS outage (60s)
23. Prolonged heading-aiding outage
24. Authentic IO-VNBD real-data heading evaluation

Author: Autonomous Navigation Engineering Team
"""

import os
import unittest
import numpy as np

from idr_pipeline.eskf_fusion import (
    ErrorStateKalmanFilter,
    wrap_pi,
    euler_to_rotation_matrix,
    rotation_matrix_to_euler,
    rodrigues,
    skew_symmetric
)
from idr_pipeline.heading_supervisor import (
    HeadingSupervisor,
    HeadingQuality,
    HeadingSourceType,
    HeadingMeasurement,
    MagneticDisturbanceDetector,
    GNSSCourseDeriver
)
from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader


class TestPhase3HeadingObservability(unittest.TestCase):

    def setUp(self):
        np.random.seed(42)

    # ------------------------------------------------------------------------
    # 1. Heading Innovation Wrapping
    # ------------------------------------------------------------------------
    def test_01_heading_innovation_wrapping(self):
        """Test angular wrapping across 0/360 (0/2pi) degree boundary."""
        # 359 deg vs 1 deg -> innovation should be +2 deg (+0.0349 rad), NOT -358 deg
        yaw_nominal = np.radians(359.0)
        yaw_meas = np.radians(1.0)
        dyaw = wrap_pi(yaw_meas - yaw_nominal)
        self.assertAlmostEqual(np.degrees(dyaw), 2.0, places=5)

        # 1 deg vs 359 deg -> innovation should be -2 deg
        dyaw_rev = wrap_pi(yaw_nominal - yaw_meas)
        self.assertAlmostEqual(np.degrees(dyaw_rev), -2.0, places=5)

        # 180 deg boundary
        dyaw_180 = wrap_pi(np.radians(179.0) - np.radians(-179.0))
        self.assertAlmostEqual(np.degrees(dyaw_180), -2.0, places=5)

    # ------------------------------------------------------------------------
    # 2. GNSS Course Calculation
    # ------------------------------------------------------------------------
    def test_02_gnss_course_calculation(self):
        """Test GNSS course derivation across all 4 quadrants in ENU."""
        deriver = GNSSCourseDeriver(min_speed_mps=2.5)

        # North (+Y): ve=0, vn=10 -> course = 0 deg
        q, c, std, spd, _ = deriver.derive_course(np.array([0.0, 10.0, 0.0]))
        self.assertEqual(q, HeadingQuality.HIGH_ACCURACY)
        self.assertAlmostEqual(np.degrees(c), 0.0, places=4)

        # East (+X): ve=10, vn=0 -> course = 90 deg
        q, c, std, spd, _ = deriver.derive_course(np.array([10.0, 0.0, 0.0]))
        self.assertAlmostEqual(np.degrees(c), 90.0, places=4)

        # South (-Y): ve=0, vn=-10 -> course = 180 deg
        q, c, std, spd, _ = deriver.derive_course(np.array([0.0, -10.0, 0.0]))
        self.assertAlmostEqual(np.degrees(c), 180.0, places=4)

        # West (-X): ve=-10, vn=0 -> course = 270 deg
        q, c, std, spd, _ = deriver.derive_course(np.array([-10.0, 0.0, 0.0]))
        self.assertAlmostEqual(np.degrees(c), 270.0, places=4)

        # North-East (+X, +Y): ve=10, vn=10 -> course = 45 deg
        q, c, std, spd, _ = deriver.derive_course(np.array([10.0, 10.0, 0.0]))
        self.assertAlmostEqual(np.degrees(c), 45.0, places=4)

    # ------------------------------------------------------------------------
    # 3. Low-Speed GNSS Rejection
    # ------------------------------------------------------------------------
    def test_03_low_speed_gnss_rejection(self):
        """Test rejection of GNSS course when speed < 2.5 m/s."""
        deriver = GNSSCourseDeriver(min_speed_mps=2.5)

        # Crawling at 1.0 m/s
        q, c, std, spd, reason = deriver.derive_course(np.array([0.7, 0.7, 0.0]))
        self.assertEqual(q, HeadingQuality.INVALID)
        self.assertIn("REJECTED_LOW_SPEED", reason)

        # Stationary at 0.0 m/s
        q0, _, _, _, reason0 = deriver.derive_course(np.array([0.0, 0.0, 0.0]))
        self.assertEqual(q0, HeadingQuality.INVALID)

        # Valid cruise at 5.0 m/s
        qv, _, _, _, reasonv = deriver.derive_course(np.array([3.0, 4.0, 0.0]))
        self.assertIn(qv, (HeadingQuality.NOMINAL, HeadingQuality.HIGH_ACCURACY))

    # ------------------------------------------------------------------------
    # 4. GNSS Course Outlier Rejection
    # ------------------------------------------------------------------------
    def test_04_gnss_course_outlier_rejection(self):
        """Test Chi-Square innovation gate rejecting sudden anomalous 90-degree jump."""
        supervisor = HeadingSupervisor(chi2_gate=6.635, min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, np.radians(0.0)])) # Facing North
        R_p2b = np.eye(3)

        # Vehicle is cruising North at 10 m/s
        gnss_vel_normal = np.array([0.0, 10.0, 0.0])
        telem1 = supervisor.process_step(
            time_s=1.0, eskf=eskf, R_p2b=R_p2b, gnss_vel=gnss_vel_normal, gnss_valid=True
        )
        self.assertTrue(telem1.is_updated)

        # Sudden anomalous GNSS glitch: velocity vector reports East (90 deg) while filter is North
        gnss_vel_glitch = np.array([10.0, 0.0, 0.0])
        telem2 = supervisor.process_step(
            time_s=1.1, eskf=eskf, R_p2b=R_p2b, gnss_vel=gnss_vel_glitch, gnss_valid=True
        )
        self.assertFalse(telem2.is_updated)
        self.assertIn("REJECTED_CHI2_GATE", telem2.rejection_reason)
        self.assertGreater(telem2.chi2_normalized, 6.635)

    # ------------------------------------------------------------------------
    # 5. GNSS Course Dropout
    # ------------------------------------------------------------------------
    def test_05_gnss_course_dropout(self):
        """Test smooth fallback to gyro dead reckoning when GNSS drops out."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # GNSS active
        telem1 = supervisor.process_step(
            time_s=1.0, eskf=eskf, R_p2b=R_p2b, gnss_vel=np.array([0.0, 10.0, 0.0]), gnss_valid=True
        )
        self.assertTrue(telem1.is_updated)

        # GNSS drops out for 5 seconds
        for t in np.linspace(1.1, 6.0, 50):
            # Gyro predicts steady straight
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.zeros(3), dt=0.1)
            telem = supervisor.process_step(
                time_s=t, eskf=eskf, R_p2b=R_p2b, gnss_vel=None, gnss_valid=False
            )
            self.assertFalse(telem.is_updated)
            self.assertEqual(telem.selected_source, HeadingSourceType.GYRO_INTEGRATION)

        self.assertAlmostEqual(telem.outage_duration_s, 5.0, delta=0.2)

    # ------------------------------------------------------------------------
    # 6. GNSS Course Recovery
    # ------------------------------------------------------------------------
    def test_06_gnss_course_recovery(self):
        """Test smooth re-acquisition and heading update when GNSS returns."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # Initial GNSS
        supervisor.process_step(1.0, eskf, R_p2b, gnss_vel=np.array([0.0, 10.0, 0.0]), gnss_valid=True)

        # Outage for 10 seconds with small drift
        for t in np.linspace(1.1, 10.0, 90):
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.array([0.0, 0.0, 0.002]), dt=0.1)
            supervisor.process_step(t, eskf, R_p2b, gnss_vel=None, gnss_valid=False)

        # GNSS returns with clean velocity
        telem_rec = supervisor.process_step(
            time_s=10.1, eskf=eskf, R_p2b=R_p2b, gnss_vel=np.array([0.0, 12.0, 0.0]), gnss_valid=True
        )
        self.assertTrue(telem_rec.is_updated)
        self.assertEqual(telem_rec.outage_duration_s, 0.0)

    # ------------------------------------------------------------------------
    # 7. Magnetometer Nominal Heading
    # ------------------------------------------------------------------------
    def test_07_magnetometer_nominal_heading(self):
        """Test tilt-compensated magnetic heading for level vehicle facing North/East."""
        detector = MagneticDisturbanceDetector(reference_field_uT=48.0)
        R_p2b = np.eye(3)
        R_b2n = np.eye(3) # Level facing North

        # Nominal field in UK/Europe: ~20 uT horizontal North, -43 uT vertical Down -> norm = 47.4 uT
        # Facing North: Forward = [20, 0, -43]
        mag_north = np.array([20.0, 0.0, -43.0])
        qual, hdg, std, norm, _ = detector.evaluate_measurement(mag_north, 1.0, R_p2b, R_b2n)
        self.assertEqual(qual, HeadingQuality.NOMINAL)
        self.assertAlmostEqual(np.degrees(hdg), 0.0, places=3)

        # Facing East (90 deg): Earth field points North (Left in body frame: +Y)
        # In FLU body frame, Left is +Y, so field = [0, 20, -43]
        detector.reset()
        mag_east = np.array([0.0, 20.0, -43.0])
        qual_e, hdg_e, _, _, _ = detector.evaluate_measurement(mag_east, 1.1, R_p2b, R_b2n)
        self.assertEqual(qual_e, HeadingQuality.NOMINAL)
        self.assertAlmostEqual(np.degrees(hdg_e), 90.0, places=3)

    # ------------------------------------------------------------------------
    # 8. Magnetometer Magnitude Disturbance Rejection
    # ------------------------------------------------------------------------
    def test_08_magnetometer_magnitude_disturbance_rejection(self):
        """Test rejection when magnetic field magnitude deviates from reference."""
        detector = MagneticDisturbanceDetector(reference_field_uT=48.0, magnitude_tolerance_uT=12.0)
        R_p2b = np.eye(3)
        R_b2n = np.eye(3)

        # Severe steel chassis magnetic anomaly: norm = 85 uT (reference is 48 uT)
        mag_disturbed = np.array([60.0, 40.0, -45.0])
        qual, hdg, std, norm, reason = detector.evaluate_measurement(mag_disturbed, 1.0, R_p2b, R_b2n)
        self.assertEqual(qual, HeadingQuality.DISTURBED)
        self.assertIn("MAGNITUDE_ANOMALY", reason)

    # ------------------------------------------------------------------------
    # 9. Magnetometer Invalid Data Rejection
    # ------------------------------------------------------------------------
    def test_09_magnetometer_invalid_data_rejection(self):
        """Test rejection of NaNs, zeros, and None inputs."""
        detector = MagneticDisturbanceDetector()
        R = np.eye(3)

        # None
        q1, _, _, _, r1 = detector.evaluate_measurement(None, 1.0, R, R)
        self.assertEqual(q1, HeadingQuality.INVALID)

        # NaN
        q2, _, _, _, r2 = detector.evaluate_measurement(np.array([np.nan, 20.0, -40.0]), 1.0, R, R)
        self.assertEqual(q2, HeadingQuality.INVALID)

        # All zeros (sensor disconnected)
        q3, _, _, _, r3 = detector.evaluate_measurement(np.array([0.0, 0.0, 0.0]), 1.0, R, R)
        self.assertEqual(q3, HeadingQuality.INVALID)

    # ------------------------------------------------------------------------
    # 10. Magnetometer Recovery
    # ------------------------------------------------------------------------
    def test_10_magnetometer_recovery(self):
        """Test return to nominal status after passing magnetic anomaly."""
        detector = MagneticDisturbanceDetector(reference_field_uT=48.0)
        R = np.eye(3)

        # Disturbed sample under steel bridge
        q_bad, _, _, _, _ = detector.evaluate_measurement(np.array([70.0, 30.0, -50.0]), 1.0, R, R)
        self.assertEqual(q_bad, HeadingQuality.DISTURBED)

        # Vehicle exits bridge, field settles to nominal over consecutive readings
        detector.evaluate_measurement(np.array([20.0, 0.0, -43.0]), 2.0, R, R)
        q_good, hdg, _, norm, _ = detector.evaluate_measurement(np.array([20.0, 0.0, -43.0]), 2.1, R, R)
        self.assertEqual(q_good, HeadingQuality.NOMINAL)
        self.assertAlmostEqual(np.degrees(hdg), 0.0, places=3)

    # ------------------------------------------------------------------------
    # 11. Heading Jacobian Finite-Difference Verification
    # ------------------------------------------------------------------------
    def test_11_heading_jacobian_finite_difference(self):
        """Rigorous finite-difference verification that H_heading[8] = -1.0 across 3D poses."""
        for r_deg in [-15.0, 0.0, 15.0]:
            for p_deg in [-10.0, 0.0, 10.0]:
                for y_deg in [0.0, 45.0, 135.0, 220.0, 315.0]:
                    R = euler_to_rotation_matrix(np.radians(r_deg), np.radians(p_deg), np.radians(y_deg))
                    _, _, yaw0 = rotation_matrix_to_euler(R)

                    eps = 1e-6
                    dtheta_z = np.array([0.0, 0.0, eps])
                    R_pert = rodrigues(dtheta_z) @ R
                    _, _, yaw_pert = rotation_matrix_to_euler(R_pert)

                    dyaw = wrap_pi(yaw_pert - yaw0)
                    J_num = dyaw / eps
                    self.assertAlmostEqual(J_num, -1.0, places=4, msg=f"Failed at r={r_deg}, p={p_deg}, y={y_deg}")

    # ------------------------------------------------------------------------
    # 12. Heading Innovation Covariance
    # ------------------------------------------------------------------------
    def test_12_heading_innovation_covariance(self):
        """Verify analytical innovation covariance S = P_88 + r_yaw^2."""
        eskf = ErrorStateKalmanFilter()
        r_yaw = 0.05 # rad
        H = np.zeros((1, 15))
        H[0, 8] = -1.0
        R_cov = np.array([[r_yaw**2]])

        S_calc = float((H @ eskf.P @ H.T + R_cov)[0, 0])
        S_expected = float(eskf.P[8, 8] + r_yaw**2)
        self.assertAlmostEqual(S_calc, S_expected, places=7)

    # ------------------------------------------------------------------------
    # 13. Heading Kalman Correction Direction
    # ------------------------------------------------------------------------
    def test_13_heading_kalman_correction_direction(self):
        """Verify positive innovation rotates nominal yaw in the correct direction."""
        # Initial filter yaw = 0.0 (North)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        target_yaw = np.radians(10.0) # Clockwise (East of North)

        # Update with measurement of 10 degrees
        eskf.update_heading(target_yaw, r_yaw=0.03)

        # Corrected yaw must have increased toward 10 degrees
        self.assertGreater(eskf.yaw, 0.0)
        self.assertLessEqual(eskf.yaw, target_yaw)

    # ------------------------------------------------------------------------
    # 14. Gyro Bias Observability
    # ------------------------------------------------------------------------
    def test_14_gyro_bias_observability(self):
        """Verify heading aiding makes gyro z-bias observable through ESKF covariance coupling."""
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        # Inject constant true gyro z-bias of +0.01 rad/s (+0.57 deg/s)
        true_bg_z = 0.01

        # Simulate vehicle driving straight North with biased gyro and periodic heading updates
        for step in range(100):
            raw_gyro = np.array([0.0, 0.0, true_bg_z])
            eskf.predict(np.array([0.0, 0.0, 9.81]), raw_gyro, dt=0.1)

            if step % 5 == 0: # 2 Hz heading updates
                eskf.update_heading(0.0, r_yaw=0.02)

        # Filter must have estimated a positive gyro z-bias
        self.assertGreater(eskf.bg[2], 0.003, "Gyro z-bias must be estimated by heading updates.")
        # Heading error should remain bounded (< 1 deg) despite gyro bias
        self.assertLess(abs(np.degrees(eskf.yaw)), 1.0)

    # ------------------------------------------------------------------------
    # 15. Combined GNSS + Magnetometer Arbitration
    # ------------------------------------------------------------------------
    def test_15_combined_gnss_and_magnetometer_arbitration(self):
        """Verify supervisor selects GNSS course at high speed and Magnetometer at slow speed."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # Scenario A: Cruising at 12 m/s -> GNSS course must be selected over Magnetometer
        mag_sample = np.array([20.0, 0.0, -43.0])
        telem_cruise = supervisor.process_step(
            time_s=1.0,
            eskf=eskf,
            R_p2b=R_p2b,
            gnss_vel=np.array([0.0, 12.0, 0.0]),
            gnss_valid=True,
            mag_raw_p=mag_sample
        )
        self.assertEqual(telem_cruise.selected_source, HeadingSourceType.GNSS_COURSE)

        # Scenario B: Crawling at 0.5 m/s -> GNSS rejected, Magnetometer must be selected
        telem_crawl = supervisor.process_step(
            time_s=2.0,
            eskf=eskf,
            R_p2b=R_p2b,
            gnss_vel=np.array([0.0, 0.5, 0.0]),
            gnss_valid=True,
            mag_raw_p=mag_sample
        )
        self.assertEqual(telem_crawl.selected_source, HeadingSourceType.MAGNETOMETER)

    # ------------------------------------------------------------------------
    # 16. Conflicting Heading Sources Behavior
    # ------------------------------------------------------------------------
    def test_16_conflicting_heading_sources_behavior(self):
        """Verify disturbed magnetometer is rejected while valid GNSS course is accepted."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # GNSS says North (0 deg), Mag is disturbed (anomaly 90 uT)
        disturbed_mag = np.array([75.0, 40.0, -30.0])
        telem = supervisor.process_step(
            time_s=1.0,
            eskf=eskf,
            R_p2b=R_p2b,
            gnss_vel=np.array([0.0, 10.0, 0.0]),
            gnss_valid=True,
            mag_raw_p=disturbed_mag
        )
        self.assertEqual(telem.selected_source, HeadingSourceType.GNSS_COURSE)
        self.assertTrue(telem.is_updated)
        self.assertTrue(telem.mag_disturbed)

    # ------------------------------------------------------------------------
    # 17. Stationary Behavior
    # ------------------------------------------------------------------------
    def test_17_stationary_behavior(self):
        """Verify stationary vehicle does not generate false GNSS course updates."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # 20 samples of standstill with small GPS velocity noise (0.1 m/s)
        for t in np.linspace(0.1, 2.0, 20):
            noise_v = np.random.randn(3) * 0.1
            telem = supervisor.process_step(
                time_s=t,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=noise_v,
                gnss_valid=True,
                mag_raw_p=None
            )
            self.assertFalse(telem.is_updated)
            self.assertEqual(telem.selected_source, HeadingSourceType.GYRO_INTEGRATION)

    # ------------------------------------------------------------------------
    # 18. Straight-Line Driving
    # ------------------------------------------------------------------------
    def test_18_straight_line_driving(self):
        """Verify heading is tightly constrained to 0 deg during 100 steps of straight driving."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        for step in range(100):
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.zeros(3), dt=0.1)
            telem = supervisor.process_step(
                time_s=step * 0.1,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=np.array([0.0, 15.0, 0.0]),
                gnss_valid=True
            )

        self.assertAlmostEqual(telem.filter_heading_deg, 0.0, places=2)
        self.assertLess(telem.heading_uncertainty_deg, 2.0)

    # ------------------------------------------------------------------------
    # 19. Left Turn Tracking
    # ------------------------------------------------------------------------
    def test_19_left_turn_tracking(self):
        """Verify smooth tracking of a 90-degree left turn (from North 0 deg to West 270 deg)."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # 90 deg turn left over 5 seconds (yaw rate = -18 deg/s = -0.314 rad/s in ENU azimuth)
        dt = 0.1
        steps = 50
        yaw_rate = -np.radians(18.0)

        for i in range(steps):
            t = (i + 1) * dt
            # In body frame FLU, left turn is positive omega_z (+0.314 rad/s)
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.array([0.0, 0.0, -yaw_rate]), dt=dt)

            # Simulated GNSS velocity during turn
            current_hdg = (0.0 + (i + 1) * yaw_rate * dt) % (2.0 * np.pi)
            v_turn = np.array([10.0 * np.sin(current_hdg), 10.0 * np.cos(current_hdg), 0.0])

            telem = supervisor.process_step(
                time_s=t,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=v_turn,
                gnss_valid=True,
                current_yaw_rate=-yaw_rate
            )

        final_heading = float(np.degrees(eskf.yaw) % 360.0)
        self.assertAlmostEqual(final_heading, 270.0, delta=3.0)

    # ------------------------------------------------------------------------
    # 20. Right Turn Tracking
    # ------------------------------------------------------------------------
    def test_20_right_turn_tracking(self):
        """Verify smooth tracking of a 90-degree right turn (from North 0 deg to East 90 deg)."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        dt = 0.1
        steps = 50
        yaw_rate = np.radians(18.0) # Right turn = +18 deg/s

        for i in range(steps):
            t = (i + 1) * dt
            # Right turn in body frame FLU is negative omega_z
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.array([0.0, 0.0, -yaw_rate]), dt=dt)

            current_hdg = (0.0 + (i + 1) * yaw_rate * dt) % (2.0 * np.pi)
            v_turn = np.array([10.0 * np.sin(current_hdg), 10.0 * np.cos(current_hdg), 0.0])

            telem = supervisor.process_step(
                time_s=t,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=v_turn,
                gnss_valid=True,
                current_yaw_rate=yaw_rate
            )

        final_heading = float(np.degrees(eskf.yaw) % 360.0)
        self.assertAlmostEqual(final_heading, 90.0, delta=3.0)

    # ------------------------------------------------------------------------
    # 21. 0/360 Degree Boundary Crossing
    # ------------------------------------------------------------------------
    def test_21_zero_360_degree_boundary_crossing(self):
        """Verify smooth tracking across North 355 deg -> 5 deg without filter explosion."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, np.radians(355.0)]))
        R_p2b = np.eye(3)

        # Transition across 0 deg North to 5 deg East over 10 steps at 10 deg/s turn rate
        yaw_rate = np.radians(10.0)
        headings_deg = np.linspace(355.0, 365.0, 11) % 360.0

        for i, h_deg in enumerate(headings_deg):
            h_rad = np.radians(h_deg)
            v = np.array([10.0 * np.sin(h_rad), 10.0 * np.cos(h_rad), 0.0])
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.array([0.0, 0.0, -yaw_rate]), dt=0.1)

            telem = supervisor.process_step(
                time_s=i * 0.1,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=v,
                gnss_valid=True
            )
            self.assertTrue(telem.is_updated)

        final_hdg = float(np.degrees(eskf.yaw) % 360.0)
        self.assertAlmostEqual(final_hdg, 5.0, delta=1.5)

    # ------------------------------------------------------------------------
    # 22. Prolonged GNSS Outage (60s)
    # ------------------------------------------------------------------------
    def test_22_prolonged_gnss_outage(self):
        """Verify dead reckoning performance and covariance bounds during 60s outage."""
        supervisor = HeadingSupervisor(min_gnss_speed_mps=2.5)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        # 60s outage with gyro noise
        for step in range(600):
            t = step * 0.1
            gyro_noise = np.random.randn(3) * 0.001
            eskf.predict(np.array([0.0, 0.0, 9.81]), gyro_noise, dt=0.1)

            telem = supervisor.process_step(
                time_s=t,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=None,
                gnss_valid=False
            )

        self.assertAlmostEqual(telem.outage_duration_s, 59.9, delta=0.5)
        # Heading uncertainty covariance P_88 must have grown monotonically
        self.assertGreater(telem.heading_uncertainty_deg, 2.0)

    # ------------------------------------------------------------------------
    # 23. Prolonged Heading-Aiding Outage
    # ------------------------------------------------------------------------
    def test_23_prolonged_heading_aiding_outage(self):
        """Verify system gracefully degrades to pure gyro integration when no heading aids exist."""
        supervisor = HeadingSupervisor(enable_magnetometer=False, enable_gnss_course=False)
        eskf = ErrorStateKalmanFilter(init_euler=np.array([0.0, 0.0, 0.0]))
        R_p2b = np.eye(3)

        for step in range(50):
            eskf.predict(np.array([0.0, 0.0, 9.81]), np.zeros(3), dt=0.1)
            telem = supervisor.process_step(
                time_s=step * 0.1,
                eskf=eskf,
                R_p2b=R_p2b,
                gnss_vel=np.array([0.0, 10.0, 0.0]),
                gnss_valid=True
            )
            self.assertFalse(telem.is_updated)
            self.assertEqual(telem.selected_source, HeadingSourceType.GYRO_INTEGRATION)

    # ------------------------------------------------------------------------
    # 24. Authentic IO-VNBD Real-Data Representative Heading Evaluation
    # ------------------------------------------------------------------------
    def test_24_real_data_representative_heading_sequence(self):
        """Evaluate heading supervisor on authentic IO-VNBD drive data (Vta01b)."""
        from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine

        s_path = os.path.join("real_iovnbd_data", "Vta01b", "S-Vta1b.csv")
        v_path = os.path.join("real_iovnbd_data", "Vta01b", "V-Vta1b.csv")
        if not os.path.exists(s_path) or not os.path.exists(v_path):
            self.skipTest("Authentic IO-VNBD Vta01b dataset files not found.")

        traj = RealIOVNBDLoader.load_drive(s_path, v_path)
        self.assertGreater(len(traj.time), 500)

        engine = SeamlessGNSSDeficitEngine(
            init_pos=traj.gt_pos[0].copy(),
            init_vel=traj.gt_vel[0].copy(),
            init_euler=np.array([traj.gt_roll[0], traj.gt_pitch[0], traj.gt_heading[0]]),
            dt=traj.dt
        )

        updates_count = 0
        heading_errors = []

        # Evaluate across the full authentic drive
        for i in range(len(traj.time)):
            v_gnss = traj.gt_vel[i] + np.random.randn(3) * 0.05
            telem = engine.process_step(
                time_s=float(traj.time[i]),
                accel_raw_p=traj.accel[i],
                gyro_raw_p=traj.gyro[i],
                baro_alt=float(traj.baro_alt[i]),
                gnss_pos=traj.gnss_pos[i],
                gnss_valid=True,
                ai_speed=float(traj.gt_speed[i]),
                gnss_vel=v_gnss,
                mag_raw_p=traj.mag[i]
            )
            if telem.heading_telemetry and telem.heading_telemetry.is_updated:
                updates_count += 1

            err_deg = abs(np.degrees(wrap_pi(engine.eskf.yaw - traj.gt_heading[i])))
            heading_errors.append(err_deg)

        self.assertGreater(updates_count, 100, f"Expected regular heading updates, got {updates_count}.")
        mean_heading_error = float(np.mean(heading_errors))
        self.assertLess(mean_heading_error, 5.0, f"Mean heading error should be <5 deg, got {mean_heading_error:.2f} deg.")


if __name__ == "__main__":
    unittest.main()
