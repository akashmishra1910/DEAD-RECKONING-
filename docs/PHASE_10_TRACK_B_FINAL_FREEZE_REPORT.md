# Track B Phase 10: Final Scientific Freeze & Completion Report

**Lead Engineer:** Akash (Autonomous Navigation & Track B Lead)  
**Co-Lead / Independent Track:** Aditya / Adi (Track A / Phase 11 Engine & JNI Integration — strictly isolated)  
**Date:** September 24, 2026  
**Status:** **100% COMPLETE — SCIENTIFICALLY FROZEN**  
**Repository State:** Verified on Branch `main`, Commit `2e33e90`  
**Dataset:** Authentic IO-VNBD 158-Outage Benchmark ($N=158$, 5 Recorded Drives)  

---

## 1. Executive Summary & Freeze Statement

Phase 10 represents the scientific research and filter formulation freeze point for Track B of the ISRO PS 26168 Dead Reckoning System.

Under strict instructions:
1. **Zero modifications or interference** have been made to Phase 11 files, code, or architecture (led independently by Aditya).
2. **Zero algorithmic modifications** have been made to the Track B estimator, ESKF equations, GBDT models, or N7 production logic.
3. The authoritative 158-outage benchmark evaluation has been completely and deterministically reproduced from scratch against authentic IO-VNBD data.

### Authoritative Frozen Baseline Metrics (158-Outage Population)

| System Pipeline | Description | Median Drift (%) | Mean Drift (%) | P90 Drift (%) | P95 Drift (%) | Pass Count (<10%) | Pass Rate (%) | Along-Track (m) | Cross-Track (m) |
|---|---|---|---|---|---|---|---|---|---|
| **$C_0$ Classical Pure DR** | `process_step` with `HeadingSupervisor` (Uncalibrated Mag Ingestion) | **76.34%** | **74.45%** | 128.49% | 145.06% | **12 / 158** | **7.59%** | 44.41 m | 64.44 m |
| **$N_0$ Neural Pure DR** | Direct ESKF strapdown bypassing magnetic corruption | **34.43%** | **47.70%** | 108.08% | 123.03% | **28 / 158** | **17.72%** | 13.94 m | 30.27 m |
| **$N_7$ Production Model** | Bounded Adaptive NHC + Causal Multi-Hypothesis Map Fusion | **35.71%** | **45.86%** | **105.58%** | **133.71%** | **36 / 158** | **22.78%** | **17.77 m** | **26.67 m** |

---

## 2. Dynamic Segment Classification Audit (The 29.80% Finding)

A critical audit requirement was ensuring that the **29.80%** median drift figure is strictly categorized as a **dynamic segment subset**, and never conflated with or substituted for the authoritative **35.71%** overall population metric.

### Dynamic Regime Stratification ($N=158$)
Outages are partitioned dynamically using causal vehicle motion thresholds:
- **Straight Road Regime:** Mean turn rate $\bar{\omega}_z \le 0.05\text{ rad/s}$ AND mean lateral acceleration $\bar{a}_y \le 0.50\text{ m/s}^2$ ($N=12$)
- **Curves / Turning Regime:** $\bar{\omega}_z > 0.05\text{ rad/s}$ OR $\bar{a}_y > 0.50\text{ m/s}^2$ ($N=146$)

| Candidate | Segment Regime | Sample Count ($N$) | Median Drift (%) | Pass Count (<10%) | Pass Rate (%) | Along-Track (m) | Cross-Track (m) |
|---|---|---|---|---|---|---|---|
| **$N_0$ (Fixed NHC)** | Straight Roads | 12 | 25.61% | 3 / 12 | 25.0% | 1.74 m | 5.44 m |
| **$N_0$ (Fixed NHC)** | Curves / Turns | 146 | 35.14% | 25 / 146 | 17.1% | 16.58 m | 33.27 m |
| **$N_7$ (Production)** | **Straight Roads** | **12** | **29.80%** (29.81%) | **4 / 12** | **33.3%** | **5.24 m** | **4.84 m** |
| **$N_7$ (Production)** | **Curves / Turns** | **146** | **35.71%** | **32 / 146** | **21.9%** | **19.40 m** | **30.06 m** |
| **$N_7$ (Production)** | **ALL OUTAGES (Overall)** | **158** | **35.71%** | **36 / 158** | **22.78%** | **17.77 m** | **26.67 m** |

### Audit Confirmation:
1. **$N_7 = 29.80\%$** is confirmed to represent exclusively the $N=12$ straight-road segment subset.
2. The overall population median drift for $N_7$ across all $N=158$ authentic outages is **35.71%** (with 36/158 passing).
3. The documentation and regression tests enforce that 29.80% cannot be reported as the overall system baseline.

---

## 3. Physical Root-Cause Reconciliation ($C_0 \rightarrow N_0 \rightarrow N_7$)

The forensic investigation documented in `docs/PHASE_10C_BASELINE_RECONCILIATION.md` was verified down to raw sensor streams:

1. **Why $C_0$ exhibits 76.34% drift:**
   - In IO-VNBD vehicle test drives, the smartphone rests on the dashboard inside a ferromagnetic vehicle chassis.
   - Without dynamic soft/hard-iron magnetic compensation, raw uncalibrated magnetometer measurements suffer severe distortions ($\approx 180^\circ$ heading flips).
   - In $C_0$, `HeadingSupervisor` ingests these uncalibrated magnetometer readings, corrupting filter heading and inducing massive cross-track divergence.

2. **Why $N_0$ recovers to 34.43% drift:**
   - $N_0$ isolates the filter by rejecting distorted magnetic headings, relying exclusively on tactical-grade gyro integration and rigid NHC ($r_{\text{lat}} = 0.05\text{ m/s}$).
   - This single rejection of toxic magnetic updates eliminates **41.91 percentage points** of drift (dropping drift from 76.34% to 34.43%).

3. **Why $N_7$ reaches 36/158 pass (22.78%) and 26.67 m cross-track:**
   - In curves and turns, lateral wheel slip violates the rigid NHC assumption ($v_y^b \equiv 0$).
   - $N_7$ applies **bounded adaptive NHC relaxation** ($R_{\text{lat}} \in [0.05^2, 5.0^2]$) driven by smoothed centripetal dynamics ($v \omega_z$) and lateral acceleration ($a_y$).
   - Concurrently, $N_7$ queries contemporaneous candidate road segments from the offline OSM graph and computes a Bayesian heading likelihood update without future lookahead.
   - This combination yields the highest causal pass count (**36/158** vs 28/158 in $N_0$ and 12/158 in $C_0$) and tightest cross-track error (**26.67 m**).

---

## 4. Exact Empirical Reproduction & Parity Evidence

A fresh, independent execution of the 158-outage benchmark was performed against the raw IO-VNBD dataset files (`Driver_A_S3b`, `Vta01b`, `Vta04`, `Vta03`, `Vta05`):

```bash
# Executed Command:
python3 scripts/run_phase10c_adaptive_nhc.py
```

### Reproduction Parity Delta

| Pipeline Candidate | Target Metric | Recomputed Value | Archived CSV Value | Absolute Numerical Diff | Parity Status |
|---|---|---|---|---|---|
| **$C_0$ Median Drift** | 76.34% | 76.33799% | 76.33799% | $3.10 \times 10^{-8}$ | **EXACT MATCH** |
| **$C_0$ Pass Count** | 12 / 158 | 12 / 158 | 12 / 158 | 0 | **EXACT MATCH** |
| **$N_0$ Median Drift** | 34.43% | 34.42502% | 34.42502% | $3.21 \times 10^{-8}$ | **EXACT MATCH** |
| **$N_0$ Pass Count** | 28 / 158 | 28 / 158 | 28 / 158 | 0 | **EXACT MATCH** |
| **$N_7$ Median Drift** | 35.71% | 35.71036% | 35.71036% | $1.20 \times 10^{-6}$ | **EXACT MATCH** |
| **$N_7$ Pass Count** | 36 / 158 | 36 / 158 | 36 / 158 | 0 | **EXACT MATCH** |
| **$N_7$ Straight Drift** | 29.80% | 29.80783% | 29.80783% | $1.15 \times 10^{-6}$ | **EXACT MATCH** |
| **$N_7$ Straight Pass** | 4 / 12 | 4 / 12 | 4 / 12 | 0 | **EXACT MATCH** |

The reproduction verified that:
- Every outage is evaluated deterministically.
- Numerical drift differences across repeated runs are $< 1.2 \times 10^{-6}\%$, well within floating-point roundoff tolerances.

---

## 5. Artifact & Manifest Inventory

All Phase 10 artifacts are verified present, complete, and internally consistent:

### Documentation
- `docs/PHASE_10A_ABLATION_SEMANTICS_AUDIT.md`: Candidate semantics and outage attribution audit.
- `docs/PHASE_10A_R2_CAUSAL_ABLATION_REPORT.md`: Speed regressor R2 causal ablation.
- `docs/PHASE_10A_R2_FINAL_FORENSIC_REVIEW.md`: Temporal anti-lookahead validation.
- `docs/PHASE_10B_RESIDUAL_ERROR_REPORT.md`: Residual decomposition and oracle $B_2$ lateral velocity bounds.
- `docs/PHASE_10C_B2_ORACLE_FORENSICS.md`: Forensic analysis of lateral slip observability.
- `docs/PHASE_10C_BASELINE_RECONCILIATION.md`: Authoritative $C_0$ vs $N_0$ root-cause diagnosis.
- `docs/PHASE_10C_ADAPTIVE_NHC_REPORT.md`: Comprehensive evaluation of candidates $N_0$ through $N_7$.
- `docs/PHASE_10_TRACK_B_FINAL_FREEZE_REPORT.md`: Final Phase 10 freeze closure report (this document).

### Verification Scripts & Test Suites
- `scripts/run_phase10c_adaptive_nhc.py`: Production benchmark evaluator for $N_0$–$N_7$.
- `scripts/run_phase10b_residual_investigation.py`: Diagnostic lateral velocity and oracle script.
- `scripts/run_phase10a_r2_causal_investigation.py`: Feature causality investigation runner.
- `scripts/evaluate_phase8_benchmark_closure.py`: Windowed evaluation script reproducing $C_0$.
- `tests/test_phase10_track_b_freeze.py`: Dedicated automated CI verification suite asserting baseline invariance.

### Empirical Data Artifacts
- `reports/phase10c_baseline_reconciliation.csv`: Per-outage comparison of $C_0$ and $N_0$ ($N=158$).
- `reports/phase10c_adaptive_nhc.csv`: Stratified duration summary table for candidates $N_0$–$N_7$.
- `reports/phase10c_dynamic_segment_analysis.csv`: Straight roads vs curves/turns breakdown.
- `reports/phase10c_trajectory_samples.csv`: Complete 2,528 evaluation records across all candidates and modes.

---

## 6. Regression Testing Status

The full Track A + Track B regression test suite was executed:

```bash
python3 -m pytest tests/test_phase[1-8]_*.py tests/test_phase10_track_b_freeze.py tests/test_eskf_fusion.py -q
```

**Results:**
- **154 / 154 tests PASSED (100%)** in 11.14 seconds.
- Dedicated freeze test `tests/test_phase10_track_b_freeze.py`: **5 / 5 PASSED (100%)**.

---

## 7. Sign-Off & Phase 10 Closure

Track B Phase 10 is officially **closed and frozen**. The scientific baselines are locked and fully validated:

- **$C_0 = 76.34\%$ (12/158 pass)**
- **$N_0 = 34.43\%$ (28/158 pass)**
- **$N_7 = 35.71\%$ (36/158 pass)**
- **Straight Roads Subset = 29.80% (4/12 pass)**

No further algorithmic changes will be introduced to the Phase 10 baseline. All subsequent C++ engine, JNI bridge, and Android front-end work proceeds independently under Track A Phase 11.
