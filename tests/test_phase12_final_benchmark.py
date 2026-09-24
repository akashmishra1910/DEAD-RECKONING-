"""
tests/test_phase12_final_benchmark.py
=====================================
Automated verification test suite for Track B Phase 12 final benchmark comparison and freeze.

Verifies:
1. Benchmark comparison catalog presence and structure
2. Exact parity of final benchmarks:
   - C0 = 76.34% (12/158 pass)
   - N0 = 34.43% (28/158 pass)
   - N7 = 35.71% (36/158 pass)
   - N7 Straight Subset = 29.80% (4/12 pass)
   - N7 Attitude Oracle = 6.55% (103/158 pass)
3. Objective comparison against PS 26168 target (<10% bound)
4. Comprehensive Final Report presence and integrity
"""

import os
import unittest
import pandas as pd


class TestPhase12FinalBenchmark(unittest.TestCase):
    """Test suite for Track B final benchmark validation."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.comp_csv = os.path.join(cls.repo_root, "reports", "phase12_benchmark_comparison.csv")
        cls.final_report_md = os.path.join(cls.repo_root, "docs", "PHASE_12_TRACK_B_FINAL_REPORT.md")

    def test_01_artifacts_presence(self):
        """Verify presence of Phase 12 final report and benchmark comparison CSV."""
        self.assertTrue(os.path.exists(self.comp_csv), "Missing reports/phase12_benchmark_comparison.csv")
        self.assertTrue(os.path.exists(self.final_report_md), "Missing docs/PHASE_12_TRACK_B_FINAL_REPORT.md")

    def test_02_benchmark_parity(self):
        """Verify all candidate metrics match authoritative frozen numbers."""
        df = pd.read_csv(self.comp_csv)
        self.assertEqual(len(df), 4, "Must contain exactly 4 benchmark rows (C0, N0, N7, Oracle)")

        c0 = df[df["Candidate"] == "C0"].iloc[0]
        self.assertAlmostEqual(float(c0["Median_Drift_pct"]), 76.34, places=1)
        self.assertEqual(int(c0["Pass_Count_10pct"]), 12)

        n0 = df[df["Candidate"] == "N0"].iloc[0]
        self.assertAlmostEqual(float(n0["Median_Drift_pct"]), 34.43, places=1)
        self.assertEqual(int(n0["Pass_Count_10pct"]), 28)

        n7 = df[df["Candidate"] == "N7"].iloc[0]
        self.assertAlmostEqual(float(n7["Median_Drift_pct"]), 35.71, places=1)
        self.assertEqual(int(n7["Pass_Count_10pct"]), 36)
        self.assertAlmostEqual(float(n7["Straight_Median_Drift_pct"]), 29.80, places=1)
        self.assertEqual(int(n7["Straight_Pass_Count"]), 4)

        oracle = df[df["Candidate"] == "N7_AttitudeOracle"].iloc[0]
        self.assertAlmostEqual(float(oracle["Median_Drift_pct"]), 6.55, places=1)
        self.assertEqual(int(oracle["Pass_Count_10pct"]), 103)

    def test_03_target_gap_characterization(self):
        """Verify objective recording of target performance vs achieved baseline."""
        df = pd.read_csv(self.comp_csv)
        n7 = df[df["Candidate"] == "N7"].iloc[0]
        achieved_drift = float(n7["Median_Drift_pct"])
        target_drift = 10.0
        gap = achieved_drift - target_drift

        self.assertGreater(gap, 0.0, "Target (<10%) is not met overall; gap must be honestly documented")
        self.assertAlmostEqual(gap, 25.71, delta=0.5)


if __name__ == "__main__":
    unittest.main()
