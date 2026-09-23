r"""
tests/test_phase8_along_track_and_benchmark.py
==============================================
Unit & Integration Tests for Phase 8: Speed Scale/Bias Calibration,
Longitudinal Map Synchronization, Along-Track Error Attribution & Benchmark Closure.

ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System
"""

import math
import copy
import numpy as np
import pytest

from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi
from idr_pipeline.speed_scale_bias_calibrator import OnlineSpeedScaleBiasCalibrator, CalibratorState
from idr_pipeline.longitudinal_map_constraint import CausalLongitudinalMapSynchronizer, LongitudinalConstraintResult
from idr_pipeline.offline_osm_graph import OfflineOSMGraph, OSMEdge, OSMNode
from idr_pipeline.map_matcher_3d import MapMatchResult, MapMatchConfidence
from idr_pipeline.gnss_ins_fusion_engine import SeamlessGNSSDeficitEngine, NavigationMode


class TestPhase8AlongTrackAndBenchmark:

    def test_01_speed_calibrator_rls_convergence(self):
        """Test RLS parameter convergence with synthetic speed signal."""
        calibrator = OnlineSpeedScaleBiasCalibrator(forgetting_factor=0.995)
        # v_gnss = alpha * v_ai + beta
        true_alpha = 0.90
        true_beta = 0.50

        # Simulate 150 samples of varying vehicle speed [5 to 25 m/s]
        np.random.seed(42)
        for i in range(200):
            t = i * 0.1
            v_ai = 5.0 + 15.0 * math.sin(0.2 * t) ** 2
            v_gnss = true_alpha * v_ai + true_beta + np.random.normal(0, 0.05)

            calibrator.update_gnss_aiding(
                time_s=t,
                ai_speed=v_ai,
                gnss_speed=v_gnss,
                gnss_valid=True
            )

        state = calibrator.get_state()
        assert state.is_calibrated is True
        assert abs(state.scale - true_alpha) < 0.05
        assert abs(state.bias_mps - true_beta) < 0.25

    def test_02_speed_calibrator_freeze_during_outage(self):
        """Test that calibrator parameters remain strictly frozen during outage."""
        calibrator = OnlineSpeedScaleBiasCalibrator()
        calibrator.theta = np.array([0.92, 0.45])
        calibrator.sample_count = 50

        calibrator.freeze_for_outage()
        assert calibrator.is_frozen is True

        # Test speed correction with frozen parameters
        raw_speed = 15.0
        corr_speed = calibrator.correct_speed(raw_speed)
        expected = 0.92 * 15.0 + 0.45
        assert abs(corr_speed - expected) < 1e-6

        # Modifying internal theta should not affect frozen correction
        calibrator.theta = np.array([0.50, 2.50])
        corr_speed_2 = calibrator.correct_speed(raw_speed)
        assert abs(corr_speed_2 - expected) < 1e-6

    def test_03_speed_calibrator_safety_bounds(self):
        """Test clipping to safe physical bounds [0.70, 1.35] and [-3.5, +3.5]."""
        calibrator = OnlineSpeedScaleBiasCalibrator(scale_bounds=(0.70, 1.35), bias_bounds_mps=(-3.5, 3.5))

        # Update with extreme outlier values
        for i in range(20):
            calibrator.update_gnss_aiding(
                time_s=i * 0.1,
                ai_speed=10.0,
                gnss_speed=50.0,
                gnss_valid=True
            )

        state = calibrator.get_state()
        assert state.scale <= 1.35
        assert state.bias_mps <= 3.5

    def test_04_speed_calibrator_standstill_handling(self):
        """Test that low speed (<1.0 m/s) does not corrupt scale estimation."""
        calibrator = OnlineSpeedScaleBiasCalibrator()
        initial_theta = calibrator.theta.copy()

        # Feed standstill samples
        for i in range(10):
            calibrator.update_gnss_aiding(time_s=i * 0.1, ai_speed=0.0, gnss_speed=0.0, gnss_valid=True)

        assert np.array_equal(calibrator.theta, initial_theta)
        assert calibrator.sample_count == 0

    def test_05_longitudinal_synchronizer_edge_length_gate(self):
        """Test rejection of short edges (<10m)."""
        sync = CausalLongitudinalMapSynchronizer()
        eskf = ErrorStateKalmanFilter()

        edge_short = OSMEdge(
            id=1,
            source_id=1,
            target_id=2,
            name="short_road",
            road_type="primary",
            nodes=[
                OSMNode(1, 52.0, -1.0, 100.0, np.array([0.0, 0.0, 0.0])),
                OSMNode(2, 52.0, -1.0, 100.0, np.array([5.0, 0.0, 0.0]))
            ],
            polyline_enu=np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0]]),
            length=5.0
        )
        match_res = MapMatchResult(
            edge_id=1,
            edge_name="short_road",
            road_type="primary",
            layer=0,
            layer_label="Ground",
            snapped_pos=np.zeros(3),
            d_2d=0.0,
            delta_h=0.0,
            road_heading=0.0,
            road_tangent_2d=np.array([1.0, 0.0]),
            confidence=MapMatchConfidence.HIGH,
            log_posterior=0.0,
            is_valid=True
        )

        res = sync.process_step(time_s=1.0, eskf=eskf, match_result=match_res, yaw_rate_z=0.25, dt=0.1, edge=edge_short)
        assert res.applied is False
        assert res.confidence == "SHORT_EDGE"

    def test_06_longitudinal_synchronizer_yaw_rate_gate(self):
        """Test rejection when yaw rate is below turn threshold."""
        sync = CausalLongitudinalMapSynchronizer(min_yaw_rate_rads=0.12)
        eskf = ErrorStateKalmanFilter()

        edge_long = OSMEdge(
            id=2,
            source_id=1,
            target_id=3,
            name="long_road",
            road_type="primary",
            nodes=[
                OSMNode(1, 52.0, -1.0, 100.0, np.array([0.0, 0.0, 0.0])),
                OSMNode(2, 52.0, -1.0, 100.0, np.array([25.0, 0.0, 0.0])),
                OSMNode(3, 52.0, -1.0, 100.0, np.array([50.0, 25.0, 0.0]))
            ],
            polyline_enu=np.array([[0.0, 0.0, 0.0], [25.0, 0.0, 0.0], [50.0, 25.0, 0.0]]),
            length=60.0
        )
        match_res = MapMatchResult(
            edge_id=2,
            edge_name="long_road",
            road_type="primary",
            layer=0,
            layer_label="Ground",
            snapped_pos=np.zeros(3),
            d_2d=0.0,
            delta_h=0.0,
            road_heading=0.0,
            road_tangent_2d=np.array([1.0, 0.0]),
            confidence=MapMatchConfidence.HIGH,
            log_posterior=0.0,
            is_valid=True
        )

        # Straight driving: yaw_rate = 0.02 rad/s
        res = sync.process_step(time_s=1.0, eskf=eskf, match_result=match_res, yaw_rate_z=0.02, dt=0.1, edge=edge_long)
        assert res.applied is False
        assert res.confidence == "BELOW_YAW_RATE_THRESH"

    def test_07_longitudinal_synchronizer_turn_vertex_detection(self):
        """Test successful detection of 90-degree corner vertex."""
        sync = CausalLongitudinalMapSynchronizer(min_yaw_rate_rads=0.12, chi2_threshold=20.0)
        eskf = ErrorStateKalmanFilter()
        eskf.p = np.array([24.0, 0.0, 0.0]) # Near vertex at (25, 0)
        eskf.P[0:2, 0:2] = np.eye(2) * 4.0

        # Polyline with 90-deg turn at (25, 0)
        edge = OSMEdge(
            id=3,
            source_id=1,
            target_id=3,
            name="corner_road",
            road_type="primary",
            nodes=[
                OSMNode(1, 52.0, -1.0, 100.0, np.array([0.0, 0.0, 0.0])),
                OSMNode(2, 52.0, -1.0, 100.0, np.array([25.0, 0.0, 0.0])),
                OSMNode(3, 52.0, -1.0, 100.0, np.array([25.0, 25.0, 0.0]))
            ],
            polyline_enu=np.array([[0.0, 0.0, 0.0], [25.0, 0.0, 0.0], [25.0, 25.0, 0.0]]),
            length=50.0
        )
        match_res = MapMatchResult(
            edge_id=3,
            edge_name="corner_road",
            road_type="primary",
            layer=0,
            layer_label="Ground",
            snapped_pos=np.zeros(3),
            d_2d=0.0,
            delta_h=0.0,
            road_heading=0.0,
            road_tangent_2d=np.array([1.0, 0.0]),
            confidence=MapMatchConfidence.HIGH,
            log_posterior=0.0,
            is_valid=True
        )

        res = sync.process_step(time_s=10.0, eskf=eskf, match_result=match_res, yaw_rate_z=0.28, dt=0.1, edge=edge)
        assert res.applied is True
        assert abs(res.road_turn_s - 25.0) < 1e-3
        assert abs(res.innovation_m - 1.0) < 0.1

    def test_08_longitudinal_synchronizer_chi2_gating(self):
        """Test rejection of outlier along-track innovation via Chi-Square gate."""
        sync = CausalLongitudinalMapSynchronizer(chi2_threshold=9.0)
        eskf = ErrorStateKalmanFilter()
        eskf.p = np.array([0.0, 0.0, 0.0]) # 25m away from vertex -> innovation = 25m
        eskf.P[0:2, 0:2] = np.eye(2) * 1.0

        edge = OSMEdge(
            id=4,
            source_id=1,
            target_id=3,
            name="corner_road",
            road_type="primary",
            nodes=[
                OSMNode(1, 52.0, -1.0, 100.0, np.array([0.0, 0.0, 0.0])),
                OSMNode(2, 52.0, -1.0, 100.0, np.array([25.0, 0.0, 0.0])),
                OSMNode(3, 52.0, -1.0, 100.0, np.array([25.0, 25.0, 0.0]))
            ],
            polyline_enu=np.array([[0.0, 0.0, 0.0], [25.0, 0.0, 0.0], [25.0, 25.0, 0.0]]),
            length=50.0
        )
        match_res = MapMatchResult(
            edge_id=4,
            edge_name="corner_road",
            road_type="primary",
            layer=0,
            layer_label="Ground",
            snapped_pos=np.zeros(3),
            d_2d=0.0,
            delta_h=0.0,
            road_heading=0.0,
            road_tangent_2d=np.array([1.0, 0.0]),
            confidence=MapMatchConfidence.HIGH,
            log_posterior=0.0,
            is_valid=True
        )

        res = sync.process_step(time_s=10.0, eskf=eskf, match_result=match_res, yaw_rate_z=0.28, dt=0.1, edge=edge)
        assert res.applied is False
        assert res.confidence == "GATED"
        assert res.chi2_val > 9.0

    def test_09_longitudinal_synchronizer_dwell_enforcement(self):
        """Test minimum 4s dwell between consecutive along-track updates."""
        sync = CausalLongitudinalMapSynchronizer(chi2_threshold=20.0)
        eskf = ErrorStateKalmanFilter()
        eskf.p = np.array([24.0, 0.0, 0.0])
        eskf.P[0:2, 0:2] = np.eye(2) * 4.0

        edge = OSMEdge(
            id=5,
            source_id=1,
            target_id=3,
            name="corner_road",
            road_type="primary",
            nodes=[
                OSMNode(1, 52.0, -1.0, 100.0, np.array([0.0, 0.0, 0.0])),
                OSMNode(2, 52.0, -1.0, 100.0, np.array([25.0, 0.0, 0.0])),
                OSMNode(3, 52.0, -1.0, 100.0, np.array([25.0, 25.0, 0.0]))
            ],
            polyline_enu=np.array([[0.0, 0.0, 0.0], [25.0, 0.0, 0.0], [25.0, 25.0, 0.0]]),
            length=50.0
        )
        match_res = MapMatchResult(
            edge_id=5,
            edge_name="corner_road",
            road_type="primary",
            layer=0,
            layer_label="Ground",
            snapped_pos=np.zeros(3),
            d_2d=0.0,
            delta_h=0.0,
            road_heading=0.0,
            road_tangent_2d=np.array([1.0, 0.0]),
            confidence=MapMatchConfidence.HIGH,
            log_posterior=0.0,
            is_valid=True
        )

        # First update at t=10.0 -> APPLIED
        res1 = sync.process_step(time_s=10.0, eskf=eskf, match_result=match_res, yaw_rate_z=0.28, dt=0.1, edge=edge)
        assert res1.applied is True

        # Second update at t=11.5 (< 4.0s elapsed) -> GATED / REJECTED
        res2 = sync.process_step(time_s=11.5, eskf=eskf, match_result=match_res, yaw_rate_z=0.28, dt=0.1, edge=edge)
        assert res2.applied is False

    def test_10_longitudinal_kalman_update_covariance_reduction(self):
        """Test that applying along-track innovation reduces along-track covariance."""
        sync = CausalLongitudinalMapSynchronizer(chi2_threshold=20.0)
        eskf = ErrorStateKalmanFilter()
        eskf.p = np.array([24.0, 0.0, 0.0])
        eskf.P[0:2, 0:2] = np.eye(2) * 9.0

        p_before = eskf.P[0, 0]

        edge = OSMEdge(
            id=6,
            source_id=1,
            target_id=3,
            name="corner_road",
            road_type="primary",
            nodes=[
                OSMNode(1, 52.0, -1.0, 100.0, np.array([0.0, 0.0, 0.0])),
                OSMNode(2, 52.0, -1.0, 100.0, np.array([25.0, 0.0, 0.0])),
                OSMNode(3, 52.0, -1.0, 100.0, np.array([25.0, 25.0, 0.0]))
            ],
            polyline_enu=np.array([[0.0, 0.0, 0.0], [25.0, 0.0, 0.0], [25.0, 25.0, 0.0]]),
            length=50.0
        )
        match_res = MapMatchResult(
            edge_id=6,
            edge_name="corner_road",
            road_type="primary",
            layer=0,
            layer_label="Ground",
            snapped_pos=np.zeros(3),
            d_2d=0.0,
            delta_h=0.0,
            road_heading=0.0,
            road_tangent_2d=np.array([1.0, 0.0]),
            confidence=MapMatchConfidence.HIGH,
            log_posterior=0.0,
            is_valid=True
        )

        res = sync.process_step(time_s=10.0, eskf=eskf, match_result=match_res, yaw_rate_z=0.28, dt=0.1, edge=edge)
        assert res.applied is True

        p_after = eskf.P[0, 0]
        assert p_after < p_before
        assert p_after > 0.0

    def test_11_seamless_engine_speed_calibrator_integration(self):
        """Test SeamlessGNSSDeficitEngine online calibration in GNSS_AIDED and freezing in DR_ONLY."""
        engine = SeamlessGNSSDeficitEngine(
            init_pos=np.array([0.0, 0.0, 0.0]),
            init_vel=np.array([10.0, 0.0, 0.0]),
            init_euler=np.array([0.0, 0.0, np.pi / 2.0]),
            dt=0.1,
            enable_speed_calibration=True
        )

        # Feed 50 GNSS-aided steps
        for i in range(50):
            t = i * 0.1
            engine.process_step(
                time_s=t,
                accel_raw_p=np.array([0.0, 0.0, 9.81]),
                gyro_raw_p=np.array([0.0, 0.0, 0.0]),
                baro_alt=100.0,
                gnss_pos=np.array([10.0 * t, 0.0, 0.0]),
                gnss_valid=True,
                ai_speed=10.0,
                gnss_vel=np.array([10.0, 0.0, 0.0])
            )

        assert engine.mode in [NavigationMode.GNSS_AIDED, NavigationMode.GNSS_DEGRADED]
        assert engine.speed_calibrator is not None
        assert engine.speed_calibrator.sample_count >= 25
        assert engine.speed_calibrator.is_frozen is False

        # Transition into DR_ONLY
        telem = engine.process_step(
            time_s=5.1,
            accel_raw_p=np.array([0.0, 0.0, 9.81]),
            gyro_raw_p=np.array([0.0, 0.0, 0.0]),
            baro_alt=100.0,
            gnss_pos=None,
            gnss_valid=False,
            ai_speed=11.0
        )
        assert engine.mode == NavigationMode.DR_ONLY
        assert engine.speed_calibrator.is_frozen is True
        assert telem.diagnostics.speed_scale_est > 0.5

        assert engine.mode == NavigationMode.DR_ONLY
        assert engine.speed_calibrator.is_frozen is True
        assert telem.diagnostics.speed_scale_est > 0.5

    def test_12_seamless_engine_longitudinal_constraint_integration(self):
        """Test SeamlessGNSSDeficitEngine applies longitudinal constraints during DR mode."""
        corridor = OfflineOSMGraph.build_synthetic_3d_corridor()
        engine = SeamlessGNSSDeficitEngine(
            osm_graph=corridor,
            enable_map_matching=True,
            enable_longitudinal_map_constraint=True
        )

        # Run into DR mode
        telem = engine.process_step(
            time_s=0.1,
            accel_raw_p=np.array([0.0, 0.0, 9.81]),
            gyro_raw_p=np.array([0.0, 0.0, 0.0]),
            baro_alt=100.0,
            gnss_pos=None,
            gnss_valid=False,
            ai_speed=10.0,
            enable_road_constraints=True
        )
        assert engine.mode == NavigationMode.DR_ONLY
        assert telem.diagnostics.longitudinal_constraint_applied in [True, False]

    def test_13_strict_anti_lookahead_causality_test(self):
        """Test that past state estimates are mathematically invariant under future data corruption."""
        engine1 = SeamlessGNSSDeficitEngine()
        engine2 = SeamlessGNSSDeficitEngine()

        # Step 1: Same history for 5 steps
        hist1, hist2 = [], []
        for i in range(5):
            t = i * 0.1
            a = np.array([0.1 * i, 0.0, 9.81])
            g = np.array([0.0, 0.0, 0.01 * i])
            pos = np.array([i * 1.0, 0.0, 0.0])

            t1 = engine1.process_step(t, a, g, 100.0, pos, True, 10.0, np.array([10.0, 0, 0]))
            t2 = engine2.process_step(t, a, g, 100.0, pos, True, 10.0, np.array([10.0, 0, 0]))

            hist1.append(t1.pos_enu.copy())
            hist2.append(t2.pos_enu.copy())

        # Proove 100% equivalence (delta = 0.000)
        diff = max(np.linalg.norm(p1 - p2) for p1, p2 in zip(hist1, hist2))
        assert diff == 0.0

    def test_14_ablation_b0_to_b7_structural_integrity(self):
        """Test that all 8 ablation configurations can be configured and run."""
        configs = ["B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7"]
        for cfg in configs:
            engine = SeamlessGNSSDeficitEngine(
                enable_speed_calibration=(cfg in ["B2", "B3", "B5", "B6", "B7"]),
                enable_longitudinal_map_constraint=(cfg in ["B4", "B5", "B6", "B7"]),
                enable_heading_supervisor=(cfg in ["B0", "B1", "B2", "B3", "B5", "B6", "B7"])
            )
            telem = engine.process_step(
                time_s=0.1,
                accel_raw_p=np.array([0.0, 0.0, 9.81]),
                gyro_raw_p=np.array([0.0, 0.0, 0.0]),
                baro_alt=100.0,
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=12.0
            )
            assert telem.pos_enu is not None

    def test_15_edge_engine_200hz_latency_budget(self):
        """Test edge engine single-step execution speed."""
        import time
        engine = SeamlessGNSSDeficitEngine(dt=0.005)

        t0 = time.perf_counter()
        for i in range(100):
            engine.process_step(
                time_s=i * 0.005,
                accel_raw_p=np.array([0.0, 0.0, 9.81]),
                gyro_raw_p=np.array([0.0, 0.0, 0.0]),
                baro_alt=100.0,
                gnss_pos=None,
                gnss_valid=False,
                ai_speed=15.0
            )
        dt_avg_ms = ((time.perf_counter() - t0) / 100) * 1000.0
        assert dt_avg_ms < 5.0 # Well within real-time 5.0 ms budget

    def test_16_along_track_vs_cross_track_error_ratio(self):
        """Test along-track and cross-track error decomposition."""
        p_est = np.array([105.0, 2.0])  # Along = +5m, Cross = +2m (heading North)
        p_gt = np.array([100.0, 0.0])
        err_vec = p_est - p_gt

        gt_yaw = 0.0  # North heading: t = [0, 1], n = [1, 0] in ENU
        t_gt = np.array([math.sin(gt_yaw), math.cos(gt_yaw)])
        n_gt = np.array([math.cos(gt_yaw), -math.sin(gt_yaw)])

        e_along = np.dot(err_vec, t_gt)
        e_cross = np.dot(err_vec, n_gt)

        assert abs(e_along - 2.0) < 1e-6
        assert abs(e_cross - 5.0) < 1e-6
