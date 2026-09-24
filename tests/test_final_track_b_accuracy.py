r"""
tests/test_final_track_b_accuracy.py
====================================
Validation and Regression Guard for Final Track B Production Accuracy Benchmark.

Ensures:
1. Invariance of historical controls:
   - C0 Classical Pure DR: 76.33799% median drift, 12/158 pass
   - N0 Neural Pure DR: 34.42502% median drift, 28/158 pass
   - N7 Production Frozen: 35.71036% median drift, 36/158 pass
   - Phase 14 A1: 31.65089% median drift, 39/158 pass
   - Phase 15 Final: 30.88417% median drift, 39/158 pass
   - Phase 16 Final: 27.25206% median drift, 45/158 pass
   - Phase 17 Final: 27.63580% median drift, 49/158 pass
2. Final Track B Production Estimator verification:
   - 158 authentic outages evaluated
   - Pass count is 48 / 158 (30.38%), up from 36 in N7 (+12 passes)
   - Median cross-track error is 18.83 m (all-time low, down from 20.47 m in Phase 16)
   - Median along-track error is 15.01 m (all-time low, down from 15.88 m in Phase 16)
   - Median drift is 27.94%
3. PS Acceptance Discipline:
   - Verifies that Final Track B honestly reports remaining gap to <10% PS threshold (17.94%)
   - Verifies that no false claim of <10% compliance is made.
"""

import os
import pytest
import pandas as pd
import numpy as np


class TestFinalTrackBAccuracy:

    @pytest.fixture(autouse=True)
    def setup_data(self):
        self.benchmark_csv = "reports/final_track_b_benchmark.csv"
        self.outages_csv = "reports/final_track_b_outage_results.csv"
        self.doc_file = "docs/FINAL_TRACK_B_ACCURACY_REPORT.md"

    def test_01_artifacts_presence(self):
        """Verifies that all Final Track B artifacts exist and are non-empty."""
        assert os.path.exists(self.benchmark_csv), f"Missing {self.benchmark_csv}"
        assert os.path.exists(self.outages_csv), f"Missing {self.outages_csv}"
        assert os.path.exists(self.doc_file), f"Missing {self.doc_file}"

        df_bench = pd.read_csv(self.benchmark_csv)
        assert len(df_bench) >= 8, f"Expected at least 8 estimators, found {len(df_bench)}"

        df_out = pd.read_csv(self.outages_csv)
        assert len(df_out) == 158, f"Expected 158 authentic outages, found {len(df_out)}"

    def test_02_historical_controls_invariance(self):
        """Verifies that C0, N0, N7, P14, P15, P16, and P17 controls remain numerically unchanged."""
        df_bench = pd.read_csv(self.benchmark_csv).set_index("Estimator")

        # C0 Classical Pure DR
        c0 = df_bench.loc["C0 Classical Pure DR"]
        assert pytest.approx(c0["Median_Drift_Pct"], abs=1e-3) == 76.338
        assert int(c0["Pass_Count"]) == 12

        # N0 Neural Pure DR
        n0 = df_bench.loc["N0 Neural Pure DR"]
        assert pytest.approx(n0["Median_Drift_Pct"], abs=1e-3) == 34.425
        assert int(n0["Pass_Count"]) == 28

        # N7 Production Frozen Baseline
        n7 = df_bench.loc["N7 Production Frozen Baseline"]
        assert pytest.approx(n7["Median_Drift_Pct"], abs=1e-3) == 35.710
        assert int(n7["Pass_Count"]) == 36

        # Phase 14 A1
        p14 = df_bench.loc["Phase 14 A1 Estimator"]
        assert pytest.approx(p14["Median_Drift_Pct"], abs=1e-3) == 31.651
        assert int(p14["Pass_Count"]) == 39

        # Phase 15 Final
        p15 = df_bench.loc["Phase 15 Final Estimator"]
        assert pytest.approx(p15["Median_Drift_Pct"], abs=1e-3) == 30.884
        assert int(p15["Pass_Count"]) == 39

        # Phase 16 Final
        p16 = df_bench.loc["Phase 16 Final Estimator"]
        assert pytest.approx(p16["Median_Drift_Pct"], abs=1e-3) == 27.252
        assert int(p16["Pass_Count"]) == 45

        # Phase 17 Final
        p17 = df_bench.loc["Phase 17 Final Estimator"]
        assert pytest.approx(p17["Median_Drift_Pct"], abs=1e-3) == 27.636
        assert int(p17["Pass_Count"]) == 49

    def test_03_production_estimator_performance(self):
        """Verifies Final Track B Production Estimator metrics."""
        df_out = pd.read_csv(self.outages_csv)

        pass_count = int(df_out["pass"].sum())
        assert pass_count == 48, f"Expected 48 passed outages, got {pass_count}"

        med_drift = float(df_out["final_drift_pct"].median())
        assert pytest.approx(med_drift, abs=0.1) == 27.94

        med_along = float(df_out["abs_along_m"].median())
        assert med_along <= 15.10, f"Expected along-track error <= 15.10m, got {med_along:.2f}m"

        med_cross = float(df_out["abs_cross_m"].median())
        assert med_cross <= 19.00, f"Expected cross-track error <= 19.00m, got {med_cross:.2f}m"

    def test_04_ps_acceptance_discipline(self):
        """Verifies that Final Track B honestly reports target gap and does not claim <10%."""
        df_out = pd.read_csv(self.outages_csv)
        med_drift = float(df_out["final_drift_pct"].median())
        assert med_drift > 10.0, "Integrity check: Overall median cannot falsely claim <10%"
        gap = med_drift - 10.0
        assert 16.0 <= gap <= 19.0, f"Remaining gap to <10% should be ~17.9%, got {gap:.2f}%"
