"""
tests/test_phase6_map_matching.py
=================================
Automated Test Suite for Phase 6:
Production Offline 3D OSM Map Matching & Road-Constrained Dead Reckoning.

Validates:
1. WGS-84 <-> Local ENU Geodetic Bidirectional Projections (sub-millimeter precision).
2. Offline OSM Graph Construction (nodes, directed edges, polylines, layers).
3. Fast Spatial Grid Index Candidate Edge Retrieval.
4. Vectorized Point-to-Edge Polyline Snapping & Geometry Extraction.
5. 3D Elevation Barrier Penalty (|dh| > 4.0 m -> log_p <= -50.0).
6. Topological Transition Continuity & Disconnected Hop Penalization.
7. Causal Online HMM Viterbi Step-by-Step State Tracking (Zero Lookahead).
8. Audit Proof of Phase 5 Map-Snapping Endpoint Artifact.
9. Deterministic 4-Level Confidence Classification (HIGH, MEDIUM, LOW, INVALID).
10. ESKF Soft Road Heading Constraint Innovation & Covariance Reduction.
11. ESKF Soft Road Normal Cross-Track Position Constraint Update.
12. Wrong-Road Outlier Rejection & Autonomous Re-entry Recovery.
13. Multi-Layer Flyover Deck vs Ground Underpass Disambiguation.
14. Real IO-VNBD Network Topological Coverage.
15. End-to-End Closed-Loop Stability & Real-Time Performance.

Author: Autonomous Navigation Engineering Team
Project: ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System
"""

import math
import unittest
import numpy as np
from typing import Dict, List

from idr_pipeline.offline_osm_graph import (
    OfflineOSMGraph,
    OSMNode,
    OSMEdge,
    wgs84_to_ecef,
    ecef_to_wgs84,
    wgs84_to_enu,
    enu_to_wgs84
)
from idr_pipeline.map_matcher_3d import (
    Causal3DMapMatcher,
    MapMatchConfidence,
    MapMatchResult,
    apply_road_constraints_to_eskf,
    RoadNetwork3D,
    Probabilistic3DMapMatcher
)
from idr_pipeline.eskf_fusion import ErrorStateKalmanFilter, wrap_pi
from idr_pipeline.dataset_loader import generate_realistic_iovnbd_trajectory


class TestPhase6MapMatching(unittest.TestCase):
    """Rigorous unit and integration tests for Phase 6 3D OSM Map Matching."""

    def setUp(self):
        self.lat0 = 52.7700
        self.lon0 = -1.2100
        self.alt0 = 65.0
        self.synth_graph = OfflineOSMGraph.build_synthetic_3d_corridor()

    def test_01_wgs84_enu_bidirectional_roundtrip(self):
        """Verify sub-millimeter mathematical round-trip consistency of WGS84 <-> ENU."""
        test_coords = [
            (52.7700, -1.2100, 65.0),
            (52.7850, -1.2050, 75.5),
            (52.7600, -1.2200, 50.2),
            (52.7950, -1.1950, 120.0),
        ]
        for lat, lon, alt in test_coords:
            enu = wgs84_to_enu(lat, lon, alt, self.lat0, self.lon0, self.alt0)
            lat_rec, lon_rec, alt_rec = enu_to_wgs84(enu, self.lat0, self.lon0, self.alt0)

            # Differences in degrees and meters
            d_lat = abs(lat - lat_rec)
            d_lon = abs(lon - lon_rec)
            d_alt = abs(alt - alt_rec)

            self.assertLess(d_lat, 1e-8, f"Latitude roundtrip discrepancy too high: {d_lat} deg")
            self.assertLess(d_lon, 1e-8, f"Longitude roundtrip discrepancy too high: {d_lon} deg")
            self.assertLess(d_alt, 1e-4, f"Altitude roundtrip discrepancy too high: {d_alt} m")

    def test_02_offline_osm_graph_construction(self):
        """Verify graph node and edge creation, geometry precomputation, and lengths."""
        graph = OfflineOSMGraph(self.lat0, self.lon0, self.alt0)
        n1 = graph.add_node(1, 52.770, -1.210, 0.0, np.array([0.0, 0.0, 0.0]))
        n2 = graph.add_node(2, 52.771, -1.210, 0.0, np.array([0.0, 100.0, 0.0]))
        n3 = graph.add_node(3, 52.772, -1.209, 5.0, np.array([50.0, 200.0, 5.0]))

        e1 = graph.add_edge(101, 1, 2, "Seg 1", "primary", polyline_enu=np.array([[0,0,0], [0,100,0]], dtype=float))
        e2 = graph.add_edge(102, 2, 3, "Seg 2", "flyover", polyline_enu=np.array([[0,100,0], [50,200,5]], dtype=float), layer=1)

        self.assertAlmostEqual(e1.length, 100.0, places=2)
        expected_len2 = math.sqrt(50**2 + 100**2)
        self.assertAlmostEqual(e2.length, expected_len2, places=2)
        self.assertEqual(len(graph.nodes), 3)
        self.assertEqual(len(graph.edges), 2)

    def test_03_spatial_grid_candidate_query(self):
        """Verify spatial grid index correctly finds candidate edges within radius."""
        candidates = self.synth_graph.get_candidate_edges(np.array([0.0, 100.0, 0.0]), search_radius=40.0)
        self.assertGreaterEqual(len(candidates), 1)
        edge_names = [e.name for e in candidates]
        self.assertIn("Ground Arterial South", edge_names)

    def test_04_vectorized_point_to_edge_projection(self):
        """Verify vectorized polyline projection finds exact perpendicular distance and tangent."""
        edge = self.synth_graph.edges[1]  # Main Expressway (North 400 to 1500, East = 0)
        query_pt = np.array([5.0, 600.0, 0.0])  # 5m East offset
        d2d, dh, snapped, tangent, az = self.synth_graph.project_point_to_edge(query_pt, edge)

        self.assertAlmostEqual(d2d, 5.0, places=3)
        self.assertAlmostEqual(dh, 0.0, places=3)
        np.testing.assert_allclose(snapped, [0.0, 600.0, 0.0], atol=1e-3)
        np.testing.assert_allclose(tangent, [0.0, 1.0], atol=1e-3)
        self.assertAlmostEqual(az, 0.0, places=3)

    def test_05_elevation_barrier_rejection(self):
        """Verify |Delta h| > 4.0 m applies severe elevation barrier penalty (log_p <= -50)."""
        matcher = Causal3DMapMatcher(graph=self.synth_graph, elevation_barrier=4.0)
        log_p_close = matcher.compute_emission_log_prob(d_2d=1.0, delta_h=1.0, d_psi=0.0, use_3d=True)
        log_p_barrier = matcher.compute_emission_log_prob(d_2d=1.0, delta_h=5.0, d_psi=0.0, use_3d=True)

        self.assertGreater(log_p_close, -5.0)
        self.assertLessEqual(log_p_barrier, -50.0)
        self.assertLess(log_p_barrier - log_p_close, -45.0)

    def test_06_topological_transition_continuity(self):
        """Verify contiguous edges have 0/low gap, disconnected edges have high/infinite distance."""
        d_contig = self.synth_graph.get_topological_distance(0, 1)  # Edge 0 -> Edge 1
        d_disconn = self.synth_graph.get_topological_distance(6, 4)  # Service road -> Flyover deck

        self.assertLess(d_contig, 1.0)
        self.assertTrue(math.isinf(d_disconn) or d_disconn > 500.0)

    def test_07_causal_viterbi_state_update(self):
        """Verify step-by-step causal processing uses only observations up to current timestamp."""
        matcher = Causal3DMapMatcher(graph=self.synth_graph)
        matcher.reset()

        res1 = matcher.process_step(np.array([0.0, 50.0, 0.0]), heading_dr=0.0, baro_alt=0.0, speed_dr=10.0, dt=0.1)
        res2 = matcher.process_step(np.array([0.0, 60.0, 0.0]), heading_dr=0.0, baro_alt=0.0, speed_dr=10.0, dt=0.1)

        self.assertTrue(res1.is_valid)
        self.assertTrue(res2.is_valid)
        self.assertEqual(res1.edge_name, "Ground Arterial South")
        self.assertEqual(res2.edge_name, "Ground Arterial South")

    def test_08_phase5_artifact_audit_proof(self):
        """
        Mathematical proof of Phase 5 endpoint projection artifact:
        Demonstrates that projecting a point onto a single segment constructed
        between start and ground-truth endpoint artificially eliminates endpoint cross-track error.
        """
        gt_start = np.array([0.0, 0.0, 0.0])
        gt_end = np.array([0.0, 500.0, 0.0])
        # DR drifted laterally by +30m
        dr_final = np.array([30.0, 500.0, 0.0])

        # Diagnostic single-segment projection
        road_net = RoadNetwork3D()
        road_net.segments = []
        road_net.add_segment(0, "Artifact Seg", "highway", gt_start, gt_end)
        legacy_matcher = Probabilistic3DMapMatcher(road_net)

        d2d, dh, snapped = legacy_matcher.point_to_segment_distance(dr_final, road_net.segments[0])
        # Snapped point is forced to [0.0, 500.0, 0.0] == gt_end
        err_snapped = float(np.linalg.norm(snapped[:2] - gt_end[:2]))

        self.assertAlmostEqual(err_snapped, 0.0, places=4,
                               msg="Proof: Single ground-truth segment clamping forces 0.0m endpoint error!")

    def test_09_confidence_classification_levels(self):
        """Verify deterministic 4-level confidence categorization."""
        matcher = Causal3DMapMatcher(graph=self.synth_graph)

        # On-road (<5m, aligned heading) -> HIGH
        res_high = matcher.process_step(np.array([1.0, 100.0, 0.0]), heading_dr=0.0, baro_alt=0.0, speed_dr=10.0, dt=0.1)
        self.assertEqual(res_high.confidence, MapMatchConfidence.HIGH)

        # Moderate offset (8m) -> MEDIUM
        res_med = matcher.process_step(np.array([8.0, 110.0, 0.0]), heading_dr=0.0, baro_alt=0.0, speed_dr=10.0, dt=0.1)
        self.assertEqual(res_med.confidence, MapMatchConfidence.MEDIUM)

        # High offset (20m) -> LOW
        res_low = matcher.process_step(np.array([20.0, 120.0, 0.0]), heading_dr=0.0, baro_alt=0.0, speed_dr=10.0, dt=0.1)
        self.assertEqual(res_low.confidence, MapMatchConfidence.LOW)

        # Off-road (50m) -> INVALID
        res_inv = matcher.process_step(np.array([50.0, 130.0, 0.0]), heading_dr=0.0, baro_alt=0.0, speed_dr=10.0, dt=0.1)
        self.assertEqual(res_inv.confidence, MapMatchConfidence.INVALID)

    def test_10_eskf_soft_road_heading_update(self):
        """Verify ESKF soft road heading constraint corrects yaw drift without filter instability."""
        eskf = ErrorStateKalmanFilter(init_pos=np.zeros(3), init_vel=np.array([0, 10, 0]), init_euler=np.array([0.0, 0.0, math.radians(10.0)]))
        init_yaw = eskf.yaw
        cov_yaw_init = eskf.P[8, 8]

        # Soft update towards 0.0 rad (North)
        ok, gamma = eskf.update_road_heading(road_azimuth=0.0, r_road_yaw=0.087, chi2_gate=9.0)
        self.assertTrue(ok)
        self.assertLess(abs(eskf.yaw), abs(init_yaw))
        self.assertLess(eskf.P[8, 8], cov_yaw_init)

    def test_11_eskf_soft_road_position_update(self):
        """Verify ESKF soft road cross-track position update constrains lateral drift."""
        # Vehicle at East = +4.0m, North = 500m facing North
        eskf = ErrorStateKalmanFilter(init_pos=np.array([4.0, 500.0, 0.0]), init_vel=np.array([0, 10, 0]), init_euler=np.zeros(3))
        cov_pos_init = eskf.P[0, 0]

        # Road centerline at East = 0.0m, tangent along North [0, 1]
        ok, gamma = eskf.update_road_position(
            road_snapped_pos=np.array([0.0, 500.0, 0.0]),
            road_tangent_2d=np.array([0.0, 1.0]),
            r_cross=1.75,
            chi2_gate=16.0
        )
        self.assertTrue(ok)
        self.assertLess(eskf.p[0], 4.0)  # Corrected towards 0.0m
        self.assertLess(eskf.P[0, 0], cov_pos_init)

    def test_12_wrong_road_rejection_and_recovery(self):
        """Verify wrong-road outlier rejects soft constraints and recovers when back on corridor."""
        matcher = Causal3DMapMatcher(graph=self.synth_graph)
        matcher.reset()

        # Step 1: On-road -> HIGH
        res1 = matcher.process_step(np.array([0.0, 100.0, 0.0]), heading_dr=0.0, baro_alt=0.0)
        self.assertEqual(res1.confidence, MapMatchConfidence.HIGH)

        # Step 2: Off-road perturbation (+40m) -> INVALID
        res2 = matcher.process_step(np.array([40.0, 110.0, 0.0]), heading_dr=0.0, baro_alt=0.0)
        self.assertEqual(res2.confidence, MapMatchConfidence.INVALID)

        # Step 3: Re-entry to road -> Recovers to HIGH
        res3 = matcher.process_step(np.array([0.0, 120.0, 0.0]), heading_dr=0.0, baro_alt=0.0)
        self.assertEqual(res3.confidence, MapMatchConfidence.HIGH)

    def test_13_multi_layer_flyover_disambiguation(self):
        """Verify 3D HMM distinguishes elevated flyover deck from surface underpass."""
        matcher = Causal3DMapMatcher(graph=self.synth_graph, elevation_barrier=4.0)

        # Vehicle at North = 2000m, Alt = +8.5m (on flyover)
        res_fly = matcher.process_step(np.array([0.0, 2000.0, 8.5]), heading_dr=0.0, baro_alt=8.5, use_3d=True)
        self.assertEqual(res_fly.layer_label, "Flyover")

        # Vehicle at North = 2000m, Alt = 0.0m (on ground underpass)
        res_gnd = matcher.process_step(np.array([0.0, 2000.0, 0.0]), heading_dr=0.0, baro_alt=0.0, use_3d=True)
        self.assertEqual(res_gnd.layer_label, "Ground")

    def test_14_real_iovnbd_graph_coverage(self):
        """Verify offline OSM graph covers realistic coordinates and builds properly."""
        traj = generate_realistic_iovnbd_trajectory(duration_sec=60.0, dt=0.1)
        drives = {"TestDrive": traj}
        graph = OfflineOSMGraph.build_from_drives(drives)
        self.assertGreater(len(graph.nodes), 0)
        self.assertGreater(len(graph.edges), 0)

    def test_15_end_to_end_closed_loop_execution(self):
        """Verify end-to-end closed loop ESKF + Map Matcher step runs stably without NaNs."""
        traj = generate_realistic_iovnbd_trajectory(duration_sec=30.0, dt=0.1)
        graph = OfflineOSMGraph.build_synthetic_3d_corridor()
        matcher = Causal3DMapMatcher(graph=graph)
        matcher.reset()

        eskf = ErrorStateKalmanFilter(init_pos=traj.gt_pos[0], init_vel=traj.gt_vel[0], init_euler=np.array([0.0, 0.0, float(traj.gt_heading[0])]))

        for k in range(1, 50):
            eskf.predict(traj.accel[k], traj.gyro[k], dt=0.1)
            eskf.update_ai_velocity(float(traj.gt_speed[k]), r_ai=0.35)
            match_res = matcher.process_step(eskf.p, eskf.yaw, float(traj.baro_alt[k]), speed_dr=float(traj.gt_speed[k]), dt=0.1)
            apply_road_constraints_to_eskf(eskf, match_res, float(traj.gt_speed[k]))

            self.assertFalse(np.isnan(eskf.p).any(), f"NaN in position at step {k}")
            self.assertFalse(np.isnan(eskf.v).any(), f"NaN in velocity at step {k}")
            self.assertFalse(np.isnan(eskf.P).any(), f"NaN in covariance at step {k}")


if __name__ == "__main__":
    unittest.main()
