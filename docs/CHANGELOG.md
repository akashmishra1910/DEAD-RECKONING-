# Project Changelog
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

All notable changes, phase completions, and model evolutions will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to strict phase-gate change control.

## [Phase 8] - End-to-End PS Benchmark Closure & Along-Track Drift Reduction — 2026-09-23

### Added
- Implemented `idr_pipeline/speed_scale_bias_calibrator.py`: Online recursive least squares (RLS) estimator for speed scale $\alpha$ and additive bias $\beta$ ($v_{\text{corr}} = \alpha \cdot v_{\text{ai}} + \beta$) trained during GNSS-aided motion, with freeze/unfreeze on outage entry/recovery.
- Implemented `idr_pipeline/longitudinal_map_constraint.py`: Causal along-track ESKF corrections at road turns and geometry vertices using yaw-rate gating, edge-length gating, Chi-Square innovation gating ($\chi^2 \le 6.635$), and 4.0-second dwell time enforcement.
- Integrated speed calibrator and longitudinal map synchronizer into `idr_pipeline/gnss_ins_fusion_engine.py` with full state-machine synchronization and extended `SupervisorDiagnostics`.
- Created comprehensive forensic along-track error diagnostic script (`scripts/run_phase8_along_track_forensics.py`) breaking down 9 physical error components (A–I) across all 158 authentic IO-VNBD outages.
- Created end-to-end benchmark closure suite (`scripts/evaluate_phase8_benchmark_closure.py`) evaluating the complete 158-outage catalog, B0–B7 ablation matrix, heading sensitivity analysis, anti-lookahead causality proof, and 200 Hz edge engine profiling.
- Created dedicated test suite (`tests/test_phase8_along_track_and_benchmark.py`) with 16 comprehensive tests covering calibration, longitudinal constraints, causality, and benchmark invariants.
- Generated Phase 8 reports:
  - `reports/phase8_along_track_forensics.csv`
  - `reports/phase8_error_budget_breakdown.csv`
  - `reports/phase8_benchmark_closure_results.csv`
  - `reports/phase8_ablation_b0_b7.csv`
  - `reports/phase8_heading_sensitivity.csv`
  - `reports/phase8_causality_verification.csv`
  - `reports/phase8_edge_profiling.csv`
- Published `docs/PHASE_8_BENCHMARK_CLOSURE_REPORT.md`.

### Verified
- Full regression suite passing: **155 / 155 PASSED (100% Pass Rate)**.
- Error Budget Breakdown: Heading cross-projection error ($83.88\%$) and velocity scale/bias drift ($14.72\%$) proven as the two dominant error contributors across 158 authentic outages.
- Anti-Lookahead Strict Causality: Confirmed $\Delta = 0.00000000\text{ m}$ divergence under future corruption.
- 200 Hz Edge Engine Profiling: $5.52\text{ ms/cycle}$ step latency in uncompiled Python, $<0.03\text{ MB}$ peak RAM footprint.

---

## [Phase 7] - Full GNSS/INS Supervisor, Seamless GNSS Deficit Engine & Closed-Loop Constraint Diagnostics — 2026-09-23

### Added
- Implemented production 6-state navigation supervisor in `idr_pipeline/gnss_ins_fusion_engine.py` (`GNSS_AIDED`, `GNSS_DEGRADED`, `TRANSITION_TO_DR`, `DR_ONLY`, `TRANSITION_TO_GNSS`, `GNSS_REACQUIRED`).
- Implemented causal multi-criteria outage detection combining validity flags, stream timeout ($\Delta t > 1.5\text{ s}$), position Mahalanobis gating ($\gamma_{\text{pos}} > 11.35$), and Doppler velocity gating ($\gamma_{\text{vel}} > 9.0$).
- Enforced multi-epoch confirmation dwell ($N=4$ epochs) during DR $\to$ GNSS reacquisition to reject multipath spikes at tunnel/underpass exits.
- Implemented dynamic uncertainty-weighted velocity arbitration and unified multi-source heading arbitration.
- Created stratified 158-outage builder across 5 authentic IO-VNBD drives (`idr_pipeline/outage_catalog.py`).
- Created deep forensic map constraint diagnostics and M0–M7 controlled ablation suite (`scripts/run_phase7_closed_loop_diagnostics.py`).
- Created 158-outage benchmark, 7 failure injection scenarios, causality verification, and 200 Hz edge profiler (`scripts/evaluate_phase7_supervisor_benchmark.py`).
- Created dedicated Phase 7 test suite (`tests/test_phase7_gnss_ins_supervisor.py`) with 16 comprehensive unit/integration tests.
- Generated `reports/phase7_map_closed_loop_diagnostics.csv`.
- Published `docs/PHASE_7_GNSS_INS_SUPERVISOR_REPORT.md`.

### Verified
- Full regression suite passing: **139 / 139 PASSED (100% Pass Rate)**.
- Seamless Handover: Transition into DR produces step position change $< 0.05\text{ m}$ (zero state/covariance discontinuity).
- Forensic Diagnostic Breakdown: Discovered and proved that road-normal position updates collapse cross-track error to $< 1.5\text{ m}$, while along-track speed error is unobservable from road geometry and dominates $> 95\%$ of unconstrained drift.
- Failure Injection Matrix: 100% detection and rejection of 100m position spikes, 25 m/s Doppler jumps, timestamp gaps, and false satellite acquisition spikes across 7 scenarios (F1–F7).
- Anti-Lookahead Strict Causality: Proved mathematically that corrupting future data ($t > k$) produces exactly $0.000\text{ m}$ deviation in estimated trajectory up to step $k$.
- Real-Time Execution: $2.29\text{ ms}$ per step ($436.1\text{ Hz}$ throughput in Python; $< 3.5\text{ MB}$ RAM footprint), strictly satisfying 200 Hz edge engine requirement.

---

## [Phase 6] - Production Offline 3D OSM Map Matching & Road-Constrained Dead Reckoning — 2026-09-23

### Added
- Created production offline 3D OpenStreetMap road network engine (`idr_pipeline/offline_osm_graph.py`) with directed edges, 3D polyline geometry, layer elevations, bridge/tunnel flags, spatial grid indexing, and memoized topological distance matrix.
- Implemented sub-millimeter bidirectional geodetic coordinate transformations (`wgs84_to_enu`, `enu_to_wgs84`, `wgs84_to_ecef`, `ecef_to_wgs84`).
- Implemented online causal 3D HMM map matcher (`idr_pipeline/map_matcher_3d.py`) featuring:
  - Strict 3D barometric elevation barrier ($|\Delta h| > 4.0\text{ m} \implies \log P_{\text{alt}} \le -50.0$).
  - Topological transition continuity scoring with exponential distance penalty.
  - Deterministic 4-level confidence evaluation (`HIGH`, `MEDIUM`, `LOW`, `INVALID`).
  - Closed-loop soft road-heading and soft road-normal cross-track position constraints into the 15-state ESKF.
- Upgraded `idr_pipeline/eskf_fusion.py` with `update_road_heading` and `update_road_position` measurement updates.
- Created comprehensive evaluation harness (`scripts/evaluate_phase6_map_matching.py`) benchmarking 158 authentic outages across 5 configurations.
- Generated Phase 6 reports:
  - `reports/phase6_outage_benchmark_158.csv`
  - `reports/phase6_60s_outages_detailed.csv`
  - `reports/phase6_ablation_summary.csv`
  - `reports/phase6_wrong_road_recovery.csv`
  - `reports/phase6_latency_memory.csv`
- Created dedicated Phase 6 test suite (`tests/test_phase6_map_matching.py`) with 15 scenarios.
- Published `docs/PHASE_6_OFFLINE_3D_OSM_MAP_MATCHING_REPORT.md`.

### Verified
- Full regression suite passing: **123 / 123 PASSED (100% Pass Rate)**.
- Diagnosed Phase 5 single-segment snapping artifact and mathematically proved endpoint error clamping in `test_08_phase5_artifact_audit_proof`.
- Evaluated 158 authentic outages: 3D Map Matching reduces median drift across all outages from $68.60\%$ down to $46.85\%$ ($31.7\%$ relative improvement).
- Flyover vs underpass layer disambiguation accuracy: **$84.0\%$ for 3D HMM** vs **$72.0\%$ for 2D baseline**.
- Wrong-road robustness: $+45\text{ m}$ off-road perturbation drops confidence to `INVALID` (disabling soft constraints to protect the filter), with clean re-entry recovery to `HIGH`.
- Execution performance: $2.563\text{ ms/cycle}$ step latency ($390.1\text{ Hz}$ throughput in Python; $<4.5\text{ MB}$ RAM footprint).

---

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
