"""
tests/test_phase12_1_failure_diagnostics.py
===========================================
Verification test suite for Phase 12.1 Production Candidate N7 Failure Diagnostics.

Verifies:
1. Failure mode catalog integrity (N=158)
2. Five-mode failure taxonomy counts (38 Mode 1, 34 Mode 2, 33 Mode 3, 13 Mode 5, 4 Mode 4)
3. Pass count invariance (36 / 158 passing, 122 / 158 failing)
4. Attitude oracle recovery gap (26.01% median drift gap)
5. Straight roads subset classification invariance (N=12, 29.80% median drift)
"""

import os
import unittest
import pandas as pd


class TestPhase121FailureDiagnostics(unittest.TestCase):
    """Test suite validating Phase 12.1 diagnostic findings and taxonomy integrity."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.failure_csv = os.path.join(cls.repo_root, "reports", "phase12_1_failure_modes.csv")
        p_path = os.path.join(cls.repo_root, "docs", "phases", "PHASE_12_1_N7_FAILURE_DIAGNOSTIC_REPORT.md")
        cls.report_md = p_path if os.path.exists(p_path) else os.path.join(cls.repo_root, "docs", "PHASE_12_1_N7_FAILURE_DIAGNOSTIC_REPORT.md")

    def test_01_artifact_presence(self):
        """Verify presence of Phase 12.1 CSV and documentation report."""
        self.assertTrue(os.path.exists(self.failure_csv), "Missing reports/phase12_1_failure_modes.csv")
        self.assertTrue(os.path.exists(self.report_md), "Missing PHASE_12_1_N7_FAILURE_DIAGNOSTIC_REPORT.md in docs/phases/")

    def test_02_failure_taxonomy_counts(self):
        """Verify exact distribution of failure taxonomy across all 158 outages."""
        df = pd.read_csv(self.failure_csv)
        self.assertEqual(len(df), 158, "Failure modes CSV must contain exactly 158 records")

        counts = df["primary_failure_mode"].value_counts().to_dict()

        self.assertEqual(counts.get("PASS", 0), 36, "Pass count must be exactly 36")
        self.assertEqual(counts.get("MODE_1_HEADING_OBSERVABILITY_GAP", 0), 38)
        self.assertEqual(counts.get("MODE_2_LONG_DURATION_ACCUMULATION", 0), 34)
        self.assertEqual(counts.get("MODE_3_SPEED_REGRESSOR_RESIDUAL", 0), 33)
        self.assertEqual(counts.get("MODE_5_LATERAL_SLIP_ROAD_AMBIGUITY", 0), 13)
        self.assertEqual(counts.get("MODE_4_STANDSTILL_LOW_DENOMINATOR", 0), 4)

        # Total failures
        total_failures = len(df[df["primary_failure_mode"] != "PASS"])
        self.assertEqual(total_failures, 122)

    def test_03_baseline_invariance(self):
        """Verify N7 causal baseline (35.71%) and straight subset (29.80%) remain strictly unchanged."""
        df = pd.read_csv(self.failure_csv)

        med_drift_all = float(df["drift_pct_causal"].median())
        self.assertAlmostEqual(med_drift_all, 35.71, places=1)

        straight = df[~df["is_curve_regime"]]
        self.assertEqual(len(straight), 12)
        med_drift_straight = float(straight["drift_pct_causal"].median())
        self.assertAlmostEqual(med_drift_straight, 29.80, places=1)

    def test_04_attitude_oracle_gap(self):
        """Verify the Attitude Oracle recovers >=60% of outages, demonstrating heading deficit."""
        df = pd.read_csv(self.failure_csv)
        oracle_passes = int(df["pass_10pct_oracle"].sum())
        self.assertGreaterEqual(oracle_passes, 100, "Attitude oracle must recover >= 100 / 158 outages")

        med_causal = float(df["drift_pct_causal"].median())
        med_oracle = float(df["drift_pct_oracle"].median())
        gap = med_causal - med_oracle
        self.assertAlmostEqual(gap, 29.16, delta=5.0, msg="Attitude gap should be approximately 26–29 percentage points")


if __name__ == "__main__":
    unittest.main()
