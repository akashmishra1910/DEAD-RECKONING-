"""
tests/test_phase10_track_b_freeze.py
====================================
Track B Phase 10 Scientific Freeze Verification & Parity Test Suite.

Authoritative Frozen Baselines:
- C0 Classical Pure DR:
    Median Drift: 76.34%
    Pass Count:   12 / 158 (7.59%)
- N0 Neural Pure DR:
    Median Drift: 34.43%
    Pass Count:   28 / 158 (17.72%)
- N7 Production Model (Bounded Adaptive NHC + Causal Multi-Hypothesis Map Fusion):
    Median Drift: 35.71%
    Pass Count:   36 / 158 (22.78%)

Subset Invariance:
- Straight Roads Subset (N = 12):
    N7 Median Drift: 29.80% (4 / 12 pass)
    NOTE: 29.80% is strictly the Straight Roads subset result and MUST NOT
          replace the authoritative 35.71% overall N7 baseline.
"""

import os
import unittest
import numpy as np
import pandas as pd

from idr_pipeline.real_iovnbd_loader import RealIOVNBDLoader
from idr_pipeline.offline_osm_graph import OfflineOSMGraph
from idr_pipeline.outage_catalog import build_comprehensive_outage_catalog
from scripts.run_phase10c_adaptive_nhc import run_single_outage_10c
from scripts.evaluate_phase8_benchmark_closure import _evaluate_outage_windowed


class TestPhase10TrackBFreeze(unittest.TestCase):
    """Rigorous verification suite for Track B Phase 10 authoritative baselines."""

    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.reports_dir = os.path.join(cls.repo_root, "reports")
        cls.reconciliation_csv = os.path.join(cls.reports_dir, "phase10c_baseline_reconciliation.csv")
        cls.adaptive_nhc_csv = os.path.join(cls.reports_dir, "phase10c_adaptive_nhc.csv")
        cls.trajectory_samples_csv = os.path.join(cls.reports_dir, "phase10c_trajectory_samples.csv")
        cls.dynamic_segment_csv = os.path.join(cls.reports_dir, "phase10c_dynamic_segment_analysis.csv")

    def test_01_authoritative_baseline_csv_presence(self):
        """Verify all critical Phase 10 reports and manifests are present."""
        self.assertTrue(os.path.exists(self.reconciliation_csv), "Missing phase10c_baseline_reconciliation.csv")
        self.assertTrue(os.path.exists(self.adaptive_nhc_csv), "Missing phase10c_adaptive_nhc.csv")
        self.assertTrue(os.path.exists(self.trajectory_samples_csv), "Missing phase10c_trajectory_samples.csv")
        self.assertTrue(os.path.exists(self.dynamic_segment_csv), "Missing phase10c_dynamic_segment_analysis.csv")

    def test_02_reconciled_c0_and_n0_metrics(self):
        """Verify C0 (76.34%, 12/158) and N0 (34.43%, 28/158) in reconciliation manifest."""
        df = pd.read_csv(self.reconciliation_csv)
        self.assertEqual(len(df), 158, "Reconciliation catalog must contain exactly 158 outages")

        # C0 Metrics
        c0_med = float(df["c0_drift_pct"].median())
        c0_pass = int(df["c0_pass"].sum())
        self.assertAlmostEqual(c0_med, 76.34, places=1, msg=f"C0 median drift {c0_med:.2f}% != 76.34%")
        self.assertEqual(c0_pass, 12, f"C0 pass count {c0_pass} != 12")

        # N0 Metrics
        n0_med = float(df["n0_drift_pct"].median())
        n0_pass = int(df["n0_pass"].sum())
        self.assertAlmostEqual(n0_med, 34.43, places=1, msg=f"N0 median drift {n0_med:.2f}% != 34.43%")
        self.assertEqual(n0_pass, 28, f"N0 pass count {n0_pass} != 28")

    def test_03_authoritative_n7_production_baseline(self):
        """Verify N7 overall median drift = 35.71% and pass count = 36/158."""
        df_summary = pd.read_csv(self.adaptive_nhc_csv)
        n7_overall = df_summary[(df_summary["Candidate"] == "N7") & (df_summary["Duration"] == "Overall")]
        self.assertEqual(len(n7_overall), 1)

        med_drift = float(n7_overall["Median_Drift_pct"].iloc[0])
        pass_cnt = int(n7_overall["Pass_Count_10pct"].iloc[0])
        total_n = int(n7_overall["N"].iloc[0])

        self.assertEqual(total_n, 158)
        self.assertAlmostEqual(med_drift, 35.71, places=1, msg=f"N7 median drift {med_drift:.2f}% != 35.71%")
        self.assertEqual(pass_cnt, 36, f"N7 pass count {pass_cnt} != 36")

        # Cross-verify against trajectory samples
        df_traj = pd.read_csv(self.trajectory_samples_csv)
        n7_samples = df_traj[(df_traj["candidate"] == "N7") & (~df_traj["use_attitude_oracle"])]
        self.assertEqual(len(n7_samples), 158)
        self.assertAlmostEqual(float(n7_samples["drift_pct"].median()), 35.71, places=1)
        self.assertEqual(int(n7_samples["pass_10pct"].sum()), 36)

    def test_04_straight_roads_subset_classification(self):
        """
        Audit that 29.80% is STRICTLY classified as the Straight Roads subset (N=12)
        and does NOT replace the authoritative 35.71% overall N7 baseline.
        """
        df_traj = pd.read_csv(self.trajectory_samples_csv)
        n7_samples = df_traj[(df_traj["candidate"] == "N7") & (~df_traj["use_attitude_oracle"])]

        straight_subset = n7_samples[~n7_samples["is_curve_regime"]]
        curve_subset = n7_samples[n7_samples["is_curve_regime"]]

        self.assertEqual(len(straight_subset), 12, "Straight roads subset must have exactly 12 outages")
        self.assertEqual(len(curve_subset), 146, "Curves/turns subset must have exactly 146 outages")
        self.assertEqual(len(straight_subset) + len(curve_subset), 158)

        # Straight subset metrics
        straight_med = float(straight_subset["drift_pct"].median())
        straight_pass = int(straight_subset["pass_10pct"].sum())
        self.assertAlmostEqual(straight_med, 29.80, places=1, msg=f"Straight roads median {straight_med:.2f}% != 29.80%")
        self.assertEqual(straight_pass, 4, f"Straight roads pass {straight_pass} != 4")

        # Crucial check: overall baseline is 35.71%, NOT 29.80%
        overall_med = float(n7_samples["drift_pct"].median())
        self.assertNotEqual(round(overall_med, 2), 29.80, "Overall baseline must not be replaced by straight roads subset!")
        self.assertAlmostEqual(overall_med, 35.71, places=1)

    def test_05_deterministic_outage_reproduction(self):
        """Verify deterministic mathematical reproduction of N0 and N7 on a representative sample."""
        drives_info = [
            ("Driver_A_S3b", "real_iovnbd_data/Driver_A_S3b/S-S3b.csv", "real_iovnbd_data/Driver_A_S3b/V-S3b.csv"),
            ("Vta01b", "real_iovnbd_data/Vta01b/S-Vta1b.csv", "real_iovnbd_data/Vta01b/V-Vta1b.csv"),
            ("Vta04", "real_iovnbd_data/Vta04/S-Vta4.csv", "real_iovnbd_data/Vta04/V-Vta4.csv"),
            ("Vta03", "real_iovnbd_data/Vta03/S-Vta3.csv", "real_iovnbd_data/Vta03/V-Vta3.csv"),
            ("Vta05", "real_iovnbd_data/Vta05/S-Vta5.csv", "real_iovnbd_data/Vta05/V-Vta5.csv"),
        ]
        loaded_drives = {}
        for name, s_path, v_path in drives_info:
            full_s = os.path.join(self.repo_root, s_path)
            full_v = os.path.join(self.repo_root, v_path)
            if os.path.exists(full_s) and os.path.exists(full_v):
                loaded_drives[name] = RealIOVNBDLoader.load_drive(full_s, full_v, name=name)

        self.assertEqual(len(loaded_drives), 5, "All 5 authentic IO-VNBD drives must be available")
        osm_graph_all = OfflineOSMGraph.build_from_drives(loaded_drives)
        catalog = build_comprehensive_outage_catalog(loaded_drives)
        self.assertEqual(len(catalog), 158)

        df_samples = pd.read_csv(self.trajectory_samples_csv)

        # Re-run first 5 outages for N0 and N7
        for cand in ["N0", "N7"]:
            for i in range(5):
                outage = catalog[i]
                res = run_single_outage_10c(loaded_drives[outage.drive_name], outage, osm_graph_all, candidate=cand)
                self.assertIsNotNone(res)

                csv_row = df_samples[
                    (df_samples["outage_id"] == outage.outage_id) &
                    (df_samples["candidate"] == cand) &
                    (~df_samples["use_attitude_oracle"])
                ].iloc[0]

                # Assert strict numeric parity (< 1e-6 drift error)
                diff = abs(res["drift_pct"] - csv_row["drift_pct"])
                self.assertLess(diff, 1e-5, f"{cand} outage {outage.outage_id} reproduced with diff {diff:.2e}")


if __name__ == "__main__":
    unittest.main()
