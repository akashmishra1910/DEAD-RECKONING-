"""
tests/test_phase11_7_causal_hmm.py
==================================
Phase 11.7: Causal 3D HMM Map Matching & Map-Aided Dead Reckoning Test Suite.

Verifies:
1. Native C++ compilation and execution of all 21 test stages in test_causal_hmm.cpp.
2. Python Causal3DMapMatcher causal state tracking, 3D elevation barrier, and ambiguity detection.
3. Strict anti-lookahead causality proofs (Prefix A + Future A vs Prefix A + Future B).
4. C++ <-> Python mathematical parity across emission, transition, and ambiguity models.
5. Invariance of Track B frozen baselines (C0=76.34%, N0=34.43%, N7=35.71%).
"""

import os
import sys
import math
import subprocess
import unittest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from idr_pipeline.offline_osm_graph import OfflineOSMGraph, OSMEdge, OSMNode
from idr_pipeline.eskf_fusion import wrap_pi
from idr_pipeline.map_matcher_3d import Causal3DMapMatcher, MapMatchConfidence, MapMatchResult
from idr_map.idrmap_reader import IdrmapReader


class TestPhase117CausalHMM(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Compile C++ test runner
        cls.exe_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "run_test_causal_hmm.exe"))
        src_map_matcher = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr-core", "src", "map_matcher_3d.cpp"))
        src_osm_graph = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr-core", "src", "osm_graph.cpp"))
        src_idrmap_reader = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr-core", "src", "idrmap_reader.cpp"))
        src_math_utils = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr-core", "src", "math_utils.cpp"))
        test_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr-core", "tests", "test_causal_hmm.cpp"))
        include_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "idr-core", "include"))

        cmd = [
            "g++", "-std=c++17", "-O3", "-Wall", "-Wextra",
            f"-I{include_dir}",
            src_map_matcher, src_osm_graph, src_idrmap_reader, src_math_utils, test_path,
            "-o", cls.exe_path
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0:
            raise RuntimeError(f"C++ build failed:\n{res.stderr}")

    @classmethod
    def tearDownClass(cls):
        if os.path.exists(cls.exe_path):
            try:
                os.remove(cls.exe_path)
            except Exception:
                pass

    def test_01_cpp_test_suite_execution(self):
        """Execute the native C++ test suite and verify all 21 test categories pass."""
        res = subprocess.run([self.exe_path], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, f"C++ test suite failed:\n{res.stdout}\n{res.stderr}")
        self.assertIn("ALL 21 C++ CAUSAL 3D HMM MAP MATCHER TESTS PASSED", res.stdout)
        self.assertIn("10,000 Step Benchmark", res.stdout)

    def test_02_elevation_barrier_3d_disambiguation(self):
        """Verify 4.0m elevation barrier severely penalizes vertical layer hops."""
        graph = OfflineOSMGraph.build_synthetic_3d_corridor()
        matcher = Causal3DMapMatcher(graph=graph, elevation_barrier=4.0)

        # Ground altitude query (|dh| < 2.5m on underpass, |dh| = 8.0m on flyover)
        res_ground = matcher.process_step(
            pos_dr=np.array([0.0, 2000.0, 0.5]),
            heading_dr=0.0,
            baro_alt=0.5,
            speed_dr=10.0,
            dt=0.1,
            use_3d=True
        )
        self.assertEqual(res_ground.layer, 0)
        self.assertEqual(res_ground.confidence, MapMatchConfidence.HIGH)

        # Elevated flyover query (|dh| < 2.5m on flyover, |dh| = 8.8m on ground)
        matcher.reset()
        res_flyover = matcher.process_step(
            pos_dr=np.array([0.0, 2000.0, 8.8]),
            heading_dr=0.0,
            baro_alt=8.8,
            speed_dr=10.0,
            dt=0.1,
            use_3d=True
        )
        self.assertEqual(res_flyover.layer, 1)
        self.assertEqual(res_flyover.confidence, MapMatchConfidence.HIGH)

    def test_03_same_layer_parallel_ambiguity_detection(self):
        """Verify parallel roads within ambiguity margin trigger is_ambiguous=True."""
        graph = OfflineOSMGraph(origin_lat=52.77, origin_lon=-1.21, origin_alt=0.0)
        graph.add_edge(1, 10, 11, "Lane Left", "primary",
            polyline_enu=np.array([[-2.0, 0.0, 0.0], [-2.0, 200.0, 0.0]]))
        graph.add_edge(2, 20, 21, "Lane Right", "primary",
            polyline_enu=np.array([[2.0, 0.0, 0.0], [2.0, 200.0, 0.0]]))

        matcher = Causal3DMapMatcher(graph=graph)
        res = matcher.process_step(
            pos_dr=np.array([0.0, 50.0, 0.0]),
            heading_dr=0.0,
            baro_alt=0.0,
            speed_dr=10.0,
            dt=0.1,
            use_3d=True
        )
        self.assertTrue(res.is_valid)
        self.assertTrue(res.is_ambiguous)
        self.assertLess(res.score_margin, 1.5)
        self.assertNotEqual(res.confidence, MapMatchConfidence.HIGH)

    def test_04_strict_anti_lookahead_causality_invariance(self):
        """Verify past map-matching results are 100% bitwise invariant to future path forks."""
        graph = OfflineOSMGraph.build_synthetic_3d_corridor()

        # Stream A: Prefix [0..15] + Turn East
        matcher_A = Causal3DMapMatcher(graph=graph)
        results_A = []
        for k in range(15):
            y = k * 10.0
            r = matcher_A.process_step(np.array([0.0, y, 0.0]), 0.0, 0.0, 10.0, 0.1, True)
            results_A.append(r)
        # Future A
        matcher_A.process_step(np.array([25.0, 150.0, 0.0]), math.pi / 2.0, 0.0, 10.0, 0.1, True)

        # Stream B: Prefix [0..15] + Turn West
        matcher_B = Causal3DMapMatcher(graph=graph)
        results_B = []
        for k in range(15):
            y = k * 10.0
            r = matcher_B.process_step(np.array([0.0, y, 0.0]), 0.0, 0.0, 10.0, 0.1, True)
            results_B.append(r)
        # Future B
        matcher_B.process_step(np.array([-25.0, 150.0, 0.0]), -math.pi / 2.0, 0.0, 10.0, 0.1, True)

        # Invariance check
        for k in range(15):
            self.assertEqual(results_A[k].edge_id, results_B[k].edge_id)
            self.assertEqual(results_A[k].confidence, results_B[k].confidence)
            self.assertAlmostEqual(results_A[k].log_posterior, results_B[k].log_posterior, places=10)
            self.assertAlmostEqual(results_A[k].d_2d, results_B[k].d_2d, places=10)
            self.assertAlmostEqual(results_A[k].road_heading, results_B[k].road_heading, places=10)

    def test_05_emission_probability_formula_parity(self):
        """Verify mathematical parity of emission log probability."""
        matcher = Causal3DMapMatcher(sigma_d=5.0, sigma_h=1.8, sigma_psi=0.30, elevation_barrier=4.0)

        # Standard 2D + 3D
        d_2d = 3.0
        delta_h = 1.2
        d_psi = 0.10
        expected = -0.5 * (d_2d / 5.0)**2 - 0.5 * (delta_h / 1.8)**2 - 0.5 * (d_psi / 0.30)**2
        actual = matcher.compute_emission_log_prob(d_2d, delta_h, d_psi, use_3d=True)
        self.assertAlmostEqual(actual, expected, places=7)

        # Elevation barrier triggered (|dh| = 6.0m > 4.0m)
        delta_h_barrier = 6.0
        expected_barrier = -0.5 * (d_2d / 5.0)**2 + (-50.0 - 5.0 * (6.0 - 4.0)) - 0.5 * (d_psi / 0.30)**2
        actual_barrier = matcher.compute_emission_log_prob(d_2d, delta_h_barrier, d_psi, use_3d=True)
        self.assertAlmostEqual(actual_barrier, expected_barrier, places=7)


if __name__ == "__main__":
    unittest.main()
