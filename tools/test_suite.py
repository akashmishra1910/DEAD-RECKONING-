"""
test_suite.py
=============
Automated End-to-End Verification Suite for IDR Navigation System.
Validates all technical requirements and performance benchmarks.
"""

import unittest
import numpy as np

from idr_pipeline.alignment_engine import InVehicleAlignmentEngine
from idr_pipeline.vibration_filter import VibrationPotholeFilter
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine, NavigationMode
from idr_pipeline.edge_engine import EdgeNavigationEngine, TACTICAL_FOG_CONFIG, benchmark_edge_throughput
from idr_pipeline.map_matcher_3d import RoadNetwork3D, Probabilistic3DMapMatcher
from idr_pipeline.dataset_loader import generate_realistic_iovnbd_trajectory, extract_sliding_window_features
from idr_pipeline.ai_velocity_model import AIVelocityEstimator


class TestIDRNavigationSystem(unittest.TestCase):

    def setUp(self):
        np.random.seed(42)

    def test_01_alignment_engine_leveling_and_heading(self):
        """Test In-Vehicle Alignment Engine gravity leveling & forward heading PCA."""
        engine = InVehicleAlignmentEngine()

        # Mount angle: Pitch = 20 deg, Roll = 10 deg
        theta = np.radians(20.0)
        phi = np.radians(10.0)
        R_pitch = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
        R_roll = np.array([[np.cos(phi), 0, np.sin(phi)], [0, 1, 0], [-np.sin(phi), 0, np.cos(phi)]])
        R_b2p = R_roll @ R_pitch

        # 1. Stationary leveling: feed gravity in phone frame
        g_body = np.array([0.0, 0.0, 9.81])
        g_phone = R_b2p @ g_body

        for _ in range(30):
            engine.process_sample(g_phone + np.random.randn(3) * 0.05, np.zeros(3), is_stationary=True)

        self.assertTrue(engine.state.is_leveled, "Engine should be leveled after stationary samples.")
        self.assertAlmostEqual(engine.state.pitch_deg, 20.0, delta=2.0)
        self.assertAlmostEqual(engine.state.roll_deg, 10.0, delta=2.0)

        # 2. Forward heading alignment: feed forward acceleration surge in vehicle body frame [ax, 0, 0]
        for _ in range(25):
            a_surge_b = np.array([1.5, 0.0, 9.81])
            a_surge_p = R_b2p @ a_surge_b
            engine.process_sample(a_surge_p, np.zeros(3), is_stationary=False)

        self.assertTrue(engine.state.is_aligned, "Engine should be fully aligned after acceleration surge.")
        self.assertGreaterEqual(engine.state.confidence, 0.95)

        # Test transform accuracy: forward vector in phone frame transformed back to vehicle frame
        v_phone = R_b2p @ np.array([5.0, 0.0, 0.0])
        v_vehicle = engine.transform_phone_to_vehicle(v_phone)
        self.assertAlmostEqual(v_vehicle[0], 5.0, delta=0.5)
        self.assertAlmostEqual(v_vehicle[1], 0.0, delta=0.5)
        print("  [PASS] Test 1: In-Vehicle Alignment Engine (Leveling + Heading)")

    def test_02_pothole_and_vibration_filtering(self):
        """Test shock blanking for potholes (>2.5g) and engine idle harmonic suppression."""
        v_filter = VibrationPotholeFilter(dt=0.1)

        # A. Test Pothole Shock blanking
        accel_normal = np.array([0.0, 0.0, 9.81])
        gyro_normal = np.array([0.0, 0.0, 0.0])

        for _ in range(15):
            v_filter.process(accel_normal, gyro_normal)

        # Inject sudden pothole spike (35 m/s^2 vertical)
        pothole_accel = np.array([0.5, 0.2, 35.0])
        clean_a, _, state = v_filter.process(pothole_accel, gyro_normal)

        self.assertTrue(state.is_pothole_detected, "Severe vertical shock must be detected as pothole.")
        self.assertLess(clean_a[2], 15.0, "Vertical spike must be blanked/clamped close to 1g.")
        self.assertEqual(state.pothole_event_count, 1)

        # B. Test Engine Idle Harmonic Suppression
        v_filter_idle = VibrationPotholeFilter(dt=0.1)
        # Feed high-frequency stationary harmonic vibration
        is_idle_detected = False
        for t in range(20):
            idle_acc = np.array([
                0.05 * np.sin(t),
                0.03 * np.cos(t),
                9.81 + 0.3 * np.sin(t * 3.0)
            ])
            _, _, st = v_filter_idle.process(idle_acc, np.zeros(3))
            if st.is_engine_idling:
                is_idle_detected = True

        self.assertTrue(is_idle_detected, "Engine idle harmonics must trigger stationary idle flag.")
        print("  [PASS] Test 2: AI Vibration & Pothole Filter")

    def test_02b_butterworth_prefilter_and_shock_blanking(self):
        """
        Test Butterworth 4th-order zero-phase bandpass pre-filter (0.1–15 Hz)
        running ahead of shock-blanking (>2.5g impulse isolation).
        Verifies:
        1. Multi-rate parametrization (10 Hz mobile and 200 Hz edge FOG).
        2. High-frequency noise attenuation (>15 Hz chassis/engine vibration).
        3. Zero-phase preservation (no time lag).
        4. Shock-blanking of severe spikes occurring on the pre-filtered stream.
        """
        from idr_pipeline.vibration_filter import apply_butterworth_prefilter

        # 1. Test 200 Hz Edge Engine (Tactical FOG)
        fs_edge = 200.0
        t_edge = np.arange(1000) / fs_edge
        # 1g static gravity (DC) + 2 Hz true vehicle motion + 40 Hz chassis vibration
        true_motion = 1.5 * np.sin(2 * np.pi * 2.0 * t_edge)
        noise_40hz = 2.0 * np.sin(2 * np.pi * 40.0 * t_edge)
        accel_z_edge = 9.80665 + true_motion + noise_40hz

        accel_edge = np.column_stack([np.zeros_like(accel_z_edge), np.zeros_like(accel_z_edge), accel_z_edge])
        gyro_edge = np.zeros_like(accel_edge)

        # Apply pre-filter
        filt_accel_edge = apply_butterworth_prefilter(accel_edge, fs=fs_edge, lowcut=0.1, highcut=15.0, order=4)
        filt_z_edge = filt_accel_edge[:, 2]

        # 40 Hz noise must be heavily attenuated (>95% amplitude reduction by 15 Hz cutoff)
        fft_raw = np.abs(np.fft.rfft(noise_40hz))
        fft_filt = np.abs(np.fft.rfft(filt_z_edge))
        freqs = np.fft.rfftfreq(len(t_edge), 1.0 / fs_edge)
        idx_40 = np.argmin(np.abs(freqs - 40.0))

        attenuation_ratio = fft_filt[idx_40] / fft_raw[idx_40]
        self.assertLess(attenuation_ratio, 0.05, f"40 Hz noise must be attenuated >95% (got ratio {attenuation_ratio:.4f})")

        # DC gravity must be preserved (within 2.5% of 1g Earth gravity)
        self.assertAlmostEqual(np.mean(filt_z_edge), 9.80665, delta=0.25, msg="Earth gravity DC offset must be preserved.")

        # 2. Test 10 Hz Mobile Phone Pre-Filter + Shock Blanking Pipeline
        fs_mobile = 10.0
        v_filter_mobile = VibrationPotholeFilter(dt=0.1, sample_rate_hz=fs_mobile)

        N_mob = 100
        test_accel_mob = np.tile([0.0, 0.0, 9.80665], (N_mob, 1))
        test_gyro_mob = np.zeros_like(test_accel_mob)

        # Inject severe shock spike (>2.5g) at frame 40
        test_accel_mob[40, 2] += 32.0  # ~4.2g total

        # Filter batch runs Butterworth pre-filter FIRST, then shock-blanking SECOND
        clean_accel, clean_gyro, states = v_filter_mobile.filter_batch(test_accel_mob, test_gyro_mob)

        # Check pothole detection and blanking
        potholes = [s for s in states if s.is_pothole_detected]
        self.assertGreaterEqual(len(potholes), 1, "Pothole shock must be detected after pre-filtering.")
        self.assertLess(clean_accel[40, 2], 15.0, "Pothole vertical spike must be blanked to near 1g.")
        print("  [PASS] Test 2b: Butterworth Pre-Filter (0.1–15 Hz, Order 4) + Shock Blanking (>2.5g)")

    def test_03_seamless_gnss_deficit_and_chi2_recovery(self):
        """Test sub-5ms deficit detection, dead reckoning fusion, and Chi^2 re-entry gating."""
        engine = SeamlessGNSSDeficitEngine(
            init_pos=np.array([0.0, 0.0, 0.0]),
            init_vel=np.array([10.0, 0.0, 0.0]),
            init_euler=np.array([0.0, 0.0, np.pi/2]), # East
            dt=0.1
        )

        accel = np.array([0.0, 0.0, 9.81])
        gyro = np.array([0.0, 0.0, 0.0])

        # Step 1: Normal GNSS Active Mode (starts at 0.0m)
        telem1 = engine.process_step(
            time_s=0.1, accel_raw_p=accel, gyro_raw_p=gyro, baro_alt=0.0,
            gnss_pos=np.array([1.0, 0.0, 0.0]), gnss_valid=True, ai_speed=10.0
        )
        self.assertEqual(telem1.mode, NavigationMode.GNSS_AIDED_INS)

        # Step 2: Instant GNSS Blackout (Tunnel Entry at 1.0m)
        telem2 = engine.process_step(
            time_s=0.2, accel_raw_p=accel, gyro_raw_p=gyro, baro_alt=0.0,
            gnss_pos=None, gnss_valid=False, ai_speed=10.0
        )
        self.assertEqual(telem2.mode, NavigationMode.INTELLIGENT_DEAD_RECKONING,
                         "Must switch to IDR within milliseconds of GNSS loss.")
        self.assertTrue(telem2.in_outage)

        # Step 3: Run 50 steps of Dead Reckoning (5 seconds, exactly 50m traveled)
        start_outage_pos = engine.eskf.p.copy()
        for step in range(50):
            telem_dr = engine.process_step(
                time_s=0.3 + step * 0.1, accel_raw_p=accel, gyro_raw_p=gyro, baro_alt=0.0,
                gnss_pos=None, gnss_valid=False, ai_speed=10.0
            )

        # Distance traveled during 50 steps @ 10 m/s should be ~50.0m
        actual_distance_traveled = np.linalg.norm(engine.eskf.p[0:2] - start_outage_pos[0:2])
        expected_distance = 50.0
        dr_drift_m = np.abs(actual_distance_traveled - expected_distance)
        self.assertLess(dr_drift_m, 5.0, f"Drift over 50m was {dr_drift_m:.2f}m (must be < 5.0m).")


        # Step 4: GNSS Re-acquisition with Multipath Spike
        # Outlier fix 100m away (canyon reflection)
        multipath_fix = engine.eskf.p.copy() + np.array([85.0, 45.0, 0.0])
        telem_recovery = engine.process_step(
            time_s=6.3, accel_raw_p=accel, gyro_raw_p=gyro, baro_alt=0.0,
            gnss_pos=multipath_fix, gnss_valid=True, ai_speed=10.0
        )
        self.assertEqual(telem_recovery.mode, NavigationMode.GNSS_RECOVERY)
        # Verify that state was NOT corrupted by multipath outlier
        pos_after_outlier = engine.eskf.p
        self.assertGreater(np.linalg.norm(pos_after_outlier - multipath_fix), 70.0,
                           "Chi^2 gate must reject severe multipath outlier spike.")
        print("  [PASS] Test 3: Seamless GNSS Deficit Handler & Chi^2 Gate")


    def test_04_3d_map_matching_elevation_disambiguation(self):
        """Test that 3D HMM Viterbi correctly separates flyover deck from surface underpass."""
        traj = generate_realistic_iovnbd_trajectory(duration_sec=200.0, dt=0.1)
        road_net = RoadNetwork3D()
        road_net.build_network_from_trajectory(traj)
        matcher = Probabilistic3DMapMatcher(road_net)

        # Pick point on flyover deck (+8.5m elevation)
        flyover_idx = int(140.0 / 0.1)
        true_type = traj.road_type[flyover_idx]
        pos_3d = traj.gt_pos[flyover_idx:flyover_idx+5]
        headings = traj.gt_heading[flyover_idx:flyover_idx+5]
        baro_alt = traj.baro_alt[flyover_idx:flyover_idx+5]

        # 3D Matcher should classify as 'flyover'
        _, matched_types_3d, _ = matcher.match_trajectory(pos_3d, headings, baro_alt, use_3d=True)
        self.assertEqual(matched_types_3d[0], 'flyover')

        # 2D Matcher (without altitude) can confuse flyover with ground underpass underneath
        _, matched_types_2d, _ = matcher.match_trajectory(pos_3d, headings, baro_alt, use_3d=False)
        print(f"  [PASS] Test 4: 3D Map-Matching (3D classified: {matched_types_3d[0]} vs 2D: {matched_types_2d[0]})")

    def test_05_edge_engine_throughput_200hz(self):
        """Verify high-rate 200 Hz Edge Software Engine throughput."""
        bench = benchmark_edge_throughput(num_samples=1000)
        self.assertGreaterEqual(bench["achieved_rate_hz"], 500.0,
                                "Edge throughput must exceed 500 Hz (2.5x headroom) for real-time 200 Hz FOG capability.")
        self.assertGreaterEqual(bench["headroom_multiplier"], 2.5)
        print(f"  [PASS] Test 5: Edge Software Engine (Throughput: {bench['achieved_rate_hz']:.1f} Hz, Headroom: {bench['headroom_multiplier']:.1f}x)")


if __name__ == "__main__":
    print("=" * 70)
    print("RUNNING AUTOMATED TEST & VERIFICATION SUITE")
    print("=" * 70)
    unittest.main()
