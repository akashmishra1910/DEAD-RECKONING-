"""
tests/test_phase13_accuracy_research.py
=======================================
Verification test suite for Track B Phase 13 accuracy research deliverables.

Verifies:
1. Report and comparison CSV artifact presence.
2. Invariance of frozen baseline controls (C0=76.34%, N0=34.43%, N7=35.71%).
3. Performance of best candidate N13_4 (35.65% median drift, 38/158 pass).
4. Objective documentation of the remaining gap to the <10% benchmark.
"""

import os
import unittest
import pandas as pd


class TestPhase13AccuracyResearch(unittest.TestCase):
    """Test suite for Phase 13 accuracy research and candidate comparison."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.comp_csv = os.path.join(cls.repo_root, "reports", "phase13_candidate_comparison.csv")
        cls.outage_csv = os.path.join(cls.repo_root, "reports", "phase13_outage_results.csv")
        cls.ablation_csv = os.path.join(cls.repo_root, "reports", "phase13_ablation.csv")
        cls.report_md = os.path.join(cls.repo_root, "docs", "PHASE_13_ACCURACY_RESEARCH_REPORT.md")

    def test_01_artifacts_presence(self):
        """Verify presence of all Phase 13 reports and CSV artifacts."""
        self.assertTrue(os.path.exists(self.comp_csv), "Missing reports/phase13_candidate_comparison.csv")
        self.assertTrue(os.path.exists(self.outage_csv), "Missing reports/phase13_outage_results.csv")
        self.assertTrue(os.path.exists(self.ablation_csv), "Missing reports/phase13_ablation.csv")

    def test_02_baseline_control_invariance(self):
        """Verify frozen controls C0, N0, N7 remain strictly identical to historical values."""
        df = pd.read_csv(self.comp_csv)

        c0 = df[df["Candidate"].str.startswith("C0")].iloc[0]
        self.assertAlmostEqual(float(c0["Median_Drift_pct"]), 76.34, places=1)
        self.assertEqual(int(c0["Pass_Count_10pct"]), 12)

        n0 = df[df["Candidate"].str.startswith("N0")].iloc[0]
        self.assertAlmostEqual(float(n0["Median_Drift_pct"]), 34.43, places=1)
        self.assertEqual(int(n0["Pass_Count_10pct"]), 28)

        n7 = df[df["Candidate"].str.startswith("N7")].iloc[0]
        self.assertAlmostEqual(float(n7["Median_Drift_pct"]), 35.71, places=1)
        self.assertEqual(int(n7["Pass_Count_10pct"]), 36)

    def test_03_best_candidate_n13_4_performance(self):
        """Verify candidate N13_4 achieves 38/158 pass count and 35.65% median drift."""
        df_out = pd.read_csv(self.outage_csv)
        self.assertEqual(len(df_out), 158, "Must evaluate all 158 outages")

        med_drift = float(df_out["n13_4_drift_pct"].median())
        pass_cnt = int(df_out["n13_4_pass"].sum())

        self.assertAlmostEqual(med_drift, 35.65, places=1)
        self.assertEqual(pass_cnt, 38, "N13_4 must pass exactly 38 / 158 outages")

    def test_04_target_gap_documented(self):
        """Verify that <10% target compliance is not falsely claimed and gap is recorded."""
        df_out = pd.read_csv(self.outage_csv)
        achieved_drift = float(df_out["n13_4_drift_pct"].median())
        self.assertGreater(achieved_drift, 10.0, "Causal estimator has not met <10% target; gap exists")
        gap = achieved_drift - 10.0
        self.assertAlmostEqual(gap, 25.65, delta=0.5)


if __name__ == "__main__":
    unittest.main()
