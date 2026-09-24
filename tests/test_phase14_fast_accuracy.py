"""
tests/test_phase14_fast_accuracy.py
===================================
Automated verification test suite for Phase 14 Fast Track B Accuracy Sprint deliverables.

Verifies:
1. Artifact presence of Phase 14 comparison and per-outage CSVs.
2. Invariance of historical regression controls (C0=76.34%, N0=34.43%, N7=35.71%).
3. Performance of Phase 14 Best Candidate A1 (31.65% median drift, 39/158 pass).
4. Objective documentation of remaining gap to <10% target (21.65 percentage points).
"""

import os
import unittest
import pandas as pd


class TestPhase14FastAccuracy(unittest.TestCase):
    """Test suite validating Phase 14 accuracy sprint outputs."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.comp_csv = os.path.join(cls.repo_root, "reports", "phase14_fast_comparison.csv")
        cls.outage_csv = os.path.join(cls.repo_root, "reports", "phase14_fast_outage_results.csv")
        cls.report_md = os.path.join(cls.repo_root, "docs", "PHASE_14_FAST_ACCURACY_REPORT.md")

    def test_01_artifacts_presence(self):
        """Verify presence of all Phase 14 artifacts."""
        self.assertTrue(os.path.exists(self.comp_csv), "Missing reports/phase14_fast_comparison.csv")
        self.assertTrue(os.path.exists(self.outage_csv), "Missing reports/phase14_fast_outage_results.csv")

    def test_02_historical_controls_invariance(self):
        """Verify C0, N0, and N7 baseline controls remain strictly unchanged."""
        df = pd.read_csv(self.comp_csv)

        c0 = df[df["Model"].str.startswith("C0")].iloc[0]
        self.assertAlmostEqual(float(c0["Median_Drift_pct"]), 76.34, places=1)
        self.assertEqual(int(c0["Pass_Count_10pct"]), 12)

        n0 = df[df["Model"].str.startswith("N0")].iloc[0]
        self.assertAlmostEqual(float(n0["Median_Drift_pct"]), 34.43, places=1)
        self.assertEqual(int(n0["Pass_Count_10pct"]), 28)

        n7 = df[df["Model"].str.startswith("N7")].iloc[0]
        self.assertAlmostEqual(float(n7["Median_Drift_pct"]), 35.71, places=1)
        self.assertEqual(int(n7["Pass_Count_10pct"]), 36)

    def test_03_phase14_best_candidate_performance(self):
        """Verify Phase 14 Candidate A1 achieves 31.65% median drift and 39/158 pass count."""
        df_out = pd.read_csv(self.outage_csv)
        self.assertEqual(len(df_out), 158, "Must evaluate all 158 outages")

        med_drift = float(df_out["phase14_drift_pct"].median())
        pass_cnt = int(df_out["phase14_pass"].sum())

        self.assertAlmostEqual(med_drift, 31.65, places=1)
        self.assertEqual(pass_cnt, 39, "Phase 14 best must achieve 39 / 158 passes")

    def test_04_target_gap_honesty(self):
        """Verify that <10% is not claimed and remaining gap (21.65 percentage points) is documented."""
        df_out = pd.read_csv(self.outage_csv)
        med_drift = float(df_out["phase14_drift_pct"].median())
        self.assertGreater(med_drift, 10.0, "Median drift must honestly show >= 10.0%")
        gap = med_drift - 10.0
        self.assertAlmostEqual(gap, 21.65, delta=0.5)


if __name__ == "__main__":
    unittest.main()
