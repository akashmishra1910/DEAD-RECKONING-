# Project Changelog
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

All notable changes, phase completions, and model evolutions will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to strict phase-gate change control.

---

## [Phase 1] - Navigation Core Validation — 2026-09-23

### Added
- Created comprehensive Phase 1 mathematical validation suite (`tests/test_phase1_navigation_core.py`) with 19 new invariant and dynamic tests.
- Added `update_gnss_velocity` method to `ErrorStateKalmanFilter` in `idr_pipeline/eskf_fusion.py` to enable 6-DOF velocity aiding.
- Published `docs/PHASE_1_NAVIGATION_CORE_REPORT.md`.

### Fixed
- Corrected attitude error Jacobian sign in `update_ai_velocity` and `update_nhc` in `idr_pipeline/eskf_fusion.py` (`+self.R[:, i] @ skew_symmetric(self.v)`). Verified exact agreement against numerical finite difference perturbations ($<10^{-5}$).

### Verified
- 33/33 unit tests passed across `test_phase1_navigation_core.py` and `test_eskf_fusion.py`.
- 6/6 integration tests passed in `test_suite.py` (Edge throughput: 1064.3 Hz).
- All 20 mathematical and physical invariants verified (SO(3) orthonormality, det=+1.0, ENU/FLU frames, Rodrigues DCM formula, Joseph-form covariance symmetry & positive-definiteness).

---

## [Phase 0] - Baseline Freeze & Repository Forensics — 2026-09-23

### Added
- Created 8 Master Governance & Traceability Documents:
  - `docs/PROJECT_MASTER_TRACKER.md`
  - `docs/REQUIREMENTS_TRACEABILITY_MATRIX.md`
  - `docs/ARCHITECTURE.md`
  - `docs/VALIDATION_POLICY.md`
  - `docs/CHANGELOG.md`
  - `docs/KNOWN_FAILURES.md`
  - `docs/DECISIONS.md`
  - `docs/PS_COMPLIANCE_MATRIX.md`

### Fixed
- Fixed missing `Optional` import in `idr_pipeline/ai_velocity_model.py`.
- Upgraded local Python environment to `numpy==2.4.6` and `scikit-learn==1.9.1` to maintain unpickling compatibility with the frozen production GBDT models.

### Verified
- Baseline Unit Tests (`tests/test_eskf_fusion.py`): 14/14 PASSED.
- Baseline Integration & Throughput Tests (`test_suite.py`): 6/6 PASSED (Edge throughput: 1064.3 Hz).
- Benchmark Harness (`scripts/evaluate_isro_benchmarks.py`): Executed successfully on authentic driving log `Vta01b`.

---

## [Step 4C] - Authentic IO-VNBD ESKF Outage Benchmark — 2026-09-22

### Added
- Standardized multi-outage evaluation harness (`scripts/run_step4c_iovnbd_benchmark.py`) testing 10s, 30s, 60s, and 120s outages across 72 authentic IO-VNBD naturalistic drives.
- Generated 935 evaluation records, raw CSV metrics, and comparative plots (`reports/step4c_outage_results.csv`, `figures/step4c_*`).
- Published comprehensive Step 4C Benchmark Report (`ESKF_STEP4C_IOVNBD_BENCHMARK_REPORT.md`).

### Observed
- Measured authentic dead reckoning drift on real smartphone sensors without external heading aiding: 47.27%–68.81% median drift.
- Identified primary root causes: unconstrained MEMS gyroscope bias integration ($35^\circ–70^\circ$ yaw drift over 60s) and mounting alignment degradation during stationary initialization.

---

## [Step 4B] - Unified FLU Body & ENU Navigation Frame Fix — 2026-09-22

### Fixed
- Corrected legacy FRU reflection frame ($\det(\mathbf{R}_b^n) = -1.0$) to standard right-handed **FLU Body Frame** (+X Forward, +Y Left, +Z Up) and **ENU Navigation Frame** (+X East, +Y North, +Z Up).
- Enforced strict $\text{SO}(3)$ proper rotation group properties ($\det(\mathbf{R}_b^n) = +1.0$, orthogonality error $< 10^{-15}$).
- Implemented full 3D Rodrigues attitude propagation, resolving 3D pitch/roll attitude tilt issues.

---

## [Step 3E] - Machine Learning Evolution & Top-45 Frozen Checkpoint — 2026-09-21

### Added
- Completed 5 ablation experiments across 67 training drives and 5 held-out drives (`S3b`, `Vta01b`, `Vta04`, `Vtb01`, `Vw02`).
- Checkpoint 1 (39 baseline features, RMSE: 5.207 m/s).
- Checkpoint 2 (51 raw extrema features, RMSE: 5.295 m/s — rejected due to overfitting).
- Checkpoint 3B (57 jerk & angular acceleration features, RMSE: 5.010 m/s — accepted).
- Checkpoint 3C (75 frequency-domain FFT features, RMSE: 5.046 m/s — rejected).
- Checkpoint 3D/3E (Top-45 feature selection, Test RMSE: **5.001 m/s**, 0–60 km/h RMSE: **4.894 m/s**, Test $R^2$: **0.6069**).
- Model Checkpoint Frozen: `models/gbdt_speed_model_iovnbd_top45.joblib`.
