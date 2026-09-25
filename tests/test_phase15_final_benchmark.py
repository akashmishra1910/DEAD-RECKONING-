"""
tests/test_phase15_final_benchmark.py
=====================================
Automated verification test suite for Phase 15 Direct Accuracy Implementation deliverables.

ISRO / Space Applications Centre (SAC) — Problem Statement PS 26168
AI-ML Based Intelligent Dead Reckoning System

Verifies:
1. Presence of Phase 15 benchmark and per-outage CSV artifacts and documentation report.
2. Strict invariance of historical regression controls (C0=76.34%, N0=34.43%, N7=35.71%, Phase-14 A1=31.65%).
3. Performance of Phase 15 Final Estimator (30.88% median drift, 39/158 pass count).
4. Rigorous honesty regarding the remaining gap to the <10% benchmark (20.88 percentage points).
"""

import os
import unittest
import pandas as pd


class TestPhase15FinalBenchmark(unittest.TestCase):
    """Test suite validating Phase 15 direct accuracy deliverables."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.comp_csv = os.path.join(cls.repo_root, "reports", "phase15_final_benchmark.csv")
        cls.outage_csv = os.path.join(cls.repo_root, "reports", "phase15_final_outage_results.csv")
        p_path = os.path.join(cls.repo_root, "docs", "phases", "PHASE_15_FINAL_REPORT.md")
        cls.report_md = p_path if os.path.exists(p_path) else os.path.join(cls.repo_root, "docs", "PHASE_15_FINAL_REPORT.md")

    def test_01_artifacts_presence(self):
        """Verify presence of all Phase 15 deliverables."""
        self.assertTrue(os.path.exists(self.comp_csv), "Missing reports/phase15_final_benchmark.csv")
        self.assertTrue(os.path.exists(self.outage_csv), "Missing reports/phase15_final_outage_results.csv")
        self.assertTrue(os.path.exists(self.report_md), "Missing PHASE_15_FINAL_REPORT.md in docs/phases/")

    def test_02_historical_controls_invariance(self):
        """Verify C0, N0, N7, and Phase-14 A1 controls remain strictly unchanged."""
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

        p14 = df[df["Model"].str.startswith("Phase 14")].iloc[0]
        self.assertAlmostEqual(float(p14["Median_Drift_pct"]), 31.65, places=1)
        self.assertEqual(int(p14["Pass_Count_10pct"]), 39)

    def test_03_phase15_final_estimator_performance(self):
        """Verify Phase 15 Final achieves 30.88% median drift and 39/158 pass count."""
        df_out = pd.read_csv(self.outage_csv)
        self.assertEqual(len(df_out), 158, "Must evaluate all 158 authentic outages")

        med_drift = float(df_out["phase15_drift_pct"].median())
        pass_cnt = int(df_out["phase15_pass"].sum())
        med_along = float(df_out["phase15_abs_along_m"].median())
        med_cross = float(df_out["phase15_abs_cross_m"].median())

        self.assertAlmostEqual(med_drift, 30.88, places=1)
        self.assertEqual(pass_cnt, 39, "Phase 15 Final must achieve 39 / 158 passes")
        self.assertAlmostEqual(med_along, 16.50, delta=0.5)
        self.assertAlmostEqual(med_cross, 23.18, delta=0.5)

    def test_04_target_gap_honesty(self):
        """Verify that <10% target compliance is not falsely claimed and gap is recorded."""
        df_out = pd.read_csv(self.outage_csv)
        med_drift = float(df_out["phase15_drift_pct"].median())
        self.assertGreater(med_drift, 10.0, "Median drift must honestly show >= 10.0%")
        gap = med_drift - 10.0
        self.assertAlmostEqual(gap, 20.88, delta=0.5)


if __name__ == "__main__":
    unittest.main()
