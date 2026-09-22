# Project Changelog
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

All notable changes, phase completions, and model evolutions will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to strict phase-gate change control.

## [Phase 5] - Authentic GNSS-Outage Benchmark & Drift Hardening — 2026-09-23

### Added
- Created standardized multi-driver outage evaluation harness (`scripts/evaluate_phase5_outage_benchmark.py`) testing 158 authentic outages (81 $\times$ 10s, 39 $\times$ 30s, 25 $\times$ 60s, 13 $\times$ 120s) across 5 naturalistic IO-VNBD drives.
- Created deep diagnostic analysis harness (`scripts/run_phase5_deep_diagnostics.py`) analyzing 60s correlations, heading error budget, adaptive measurement noise, and 3D map constraints.
- Generated comprehensive diagnostic reports:
  - `reports/phase5_full_outage_benchmark_results.csv`
  - `reports/phase5_ablation_summary.csv`
  - `reports/phase5_60s_deep_diagnostic.csv`
  - `reports/phase5_heading_error_budget_replay.csv`
  - `reports/phase5_adaptive_r_comparison.csv`
  - `reports/phase5_map_matching_contribution.csv`
- Created dedicated Phase 5 test suite (`tests/test_phase5_outage_benchmark.py`) with 10 validation scenarios.
- Published `docs/PHASE_5_OUTAGE_BENCHMARK_REPORT.md`.

### Verified
- Full regression suite passing: **108 / 108 PASSED (100% Pass Rate)**.
- 60-Second Correlation Analysis: $r_{\text{cross}} = +0.7977$, $r_{\text{hdg}} = +0.4885$, $r_{\text{spd}} = -0.2929$, quantitatively proving heading/cross-track error is the primary error driver.
- Diagnostic Reference Heading Replay: perfect heading bounds median 60s drift from $108.21\%$ down to $24.58\%$ (proving heading drift accounts for $77.3\%$ of total error).
- Adaptive Noise Formulation: $R_v(t) = 0.35(1 + 0.5\text{Vib} + 1.2|\omega_z|)$ achieves a 4.22% median drift reduction across 158 authentic outages.
- Map Matching Contribution: corridor projection brings median drift to $0.00\%–35.18\%$ and achieves $84.0\%$ pass rate on 10s outages and $66.7\%$ on 30s outages.
- Zero future data leakage verified under randomized future sensor perturbations ($\Delta = 0.00\times 10^0$).

---

## [Phase 4] - AI Velocity Model Validation & Hardening — 2026-09-23

### Added
- Completed comprehensive feature leakage and temporal causality audit across all 45 feature dimensions (`reports/phase4_feature_leakage_audit.csv`).
- Created multi-drive generalization benchmark evaluating 10,019 authentic IO-VNBD samples (`reports/phase4_per_drive_generalization.csv`).
- Created kinematic regimes performance breakdown across speed bins and motion states (`reports/phase4_speed_regimes_breakdown.csv`).
- Created Phase 4 evaluation harness (`scripts/evaluate_phase4_ai_velocity_validation.py`).
- Implemented dedicated 15-test Phase 4 validation suite (`tests/test_phase4_ai_velocity.py`).
- Published `docs/PHASE_4_AI_VELOCITY_VALIDATION_REPORT.md`.

### Verified
- 100% causal feature extraction over sliding window $[t - 1.0\text{ s}, t]$ with zero future data leakage.
- Verified invariance under future sample perturbations ($\Delta = 0.00\times 10^0$).
- Global authentic performance on real IO-VNBD data: RMSE = $3.63\text{ m/s}$, MAE = $2.83\text{ m/s}$, Median AE = $2.31\text{ m/s}$, P95 AE = $7.20\text{ m/s}$, Bias = $+0.49\text{ m/s}$.
- Speed regimes: $3.30\text{ m/s}$ RMSE in 20–40 km/h (zero bias: $+0.06\text{ m/s}$); constant cruising RMSE = $3.22\text{ m/s}$.
- Verified ESKF measurement residual $dz = v_{\text{AI}} - \mathbf{R}_{:, 0}^T \mathbf{v}^n$ and linearized measurement Jacobian $H_{v_{\text{AI}}} = [0_{1\times 3}, \mathbf{R}_{:, 0}^T, \mathbf{R}_{:, 0}^T \lfloor\mathbf{v}^n\rfloor_\times, 0_{1\times 6}]$ via finite-difference ($<10^{-5}$).
- Verified velocity covariance reduction ($P_{v_x}^+ < P_{v_x}^-$) and adaptive measurement noise scaling.
- Benchmark profiling: feature extraction $5.66\text{ ms}$, GBDT inference $0.30\text{ ms}$ ($297\ \mu\text{s}$), total cycle $6.76\text{ ms}$ ($147.9\text{ Hz}$ throughput, 14.8x real-time margin for 10 Hz mobile).
- 15/15 Phase 4 tests passed; 98/98 full regression tests passed (100% pass rate).

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
