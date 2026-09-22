# Project Changelog
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

All notable changes, phase completions, and model evolutions will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to strict phase-gate change control.

---

## [Phase 3] - Attitude / Heading Observability Hardening — 2026-09-23

### Added
- Created `HeadingSupervisor`, `GNSSCourseDeriver`, and `MagneticDisturbanceDetector` in `idr_pipeline/heading_supervisor.py`.
- Implemented dynamic velocity-weighted GNSS Course-Over-Ground (COG) aiding with analytical noise covariance $\sigma_\psi = \text{clip}(\sigma_v/v_h, 1.0^\circ, 15.0^\circ)$ and strict low-speed cutoff ($v_h \ge 2.5\text{ m/s}$).
- Implemented 3D tilt-compensated magnetometer leveling in FLU body frame with multi-tier magnetic anomaly gating ($|\Delta B| > 12\ \mu\text{T}$, gradient $>35\ \mu\text{T/s}$).
- Added $\chi^2$ innovation gating ($\gamma \le 6.635$) with angular wrap protection and 2-epoch consistency confirmation for post-outage re-acquisition.
- Integrated Heading Supervisor into `SeamlessGNSSDeficitEngine.process_step` and added ESKF default argument initialization.
- Created dedicated Phase 3 test suite (`tests/test_phase3_heading_observability.py`) with 24 rigorous test cases.
- Published `docs/PHASE_3_HEADING_OBSERVABILITY_REPORT.md`.

### Verified
- 24/24 unit & validation tests passed in `test_phase3_heading_observability.py`.
- 83/83 tests passed across full regression suite (100% pass rate).
- Validated on authentic IO-VNBD drive `Vta01b`: 30s outage drift reduced from 51.2% (242.5m) to **5.68% (26.9m)**, exceeding the PS $<10\%$ benchmark.
- Verified ultra-low inference latency: $83.89\ \mu\text{s}$ per step ($11,919\text{ Hz}$ throughput capacity).

---

## [Phase 2] - Alignment & Initialization Hardening — 2026-09-23

### Added
- Upgraded `InVehicleAlignmentEngine` in `idr_pipeline/alignment_engine.py` with multi-mode alignment (`AlignmentMode`), stationary start protection, initial gyro bias estimation, and machine-readable `AlignmentQuality` diagnostics.
- Implemented robust horizontal covariance eigenvalue ratio evaluation ($\kappa = \lambda_{\text{long}} / \lambda_{\text{trans}}$) and peak surge amplitude gating ($a_{\text{surge}} \ge 0.35\text{ m/s}^2$) resolving Known Failure KF-02.
- Created comprehensive Phase 2 validation test suite (`tests/test_phase2_alignment_initialization.py`) with 20 dedicated unit and edge-case tests.
- Published `docs/PHASE_2_ALIGNMENT_INITIALIZATION_REPORT.md`.

### Fixed
- Fixed 3D horizontal eigenvalue extraction bug in `idr_pipeline/alignment_engine.py` by comparing the two active horizontal eigenvalues (`eigvals[-1]` vs `eigvals[-2]`) rather than vertical axis eigenvalue $\lambda_0 \approx 0$.
- Fixed cradle slip detection responsiveness by performing direct gravity vector checks during stationary periods ($<0.1\text{ s}$ bump response).

### Verified
- 20/20 tests passed in `test_phase2_alignment_initialization.py`.
- 59/59 tests passed across full regression suite.
- Reconciled Phase 1 documentation: 19 test methods validate all 20 enumerated mathematical invariants.

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
