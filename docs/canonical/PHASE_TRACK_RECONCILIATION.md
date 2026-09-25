# PHASE & TRACK RECONCILIATION MATRIX — SIH26168
## Full Traceability Across Phase 0 to Phase 17 Milestones

**Document ID:** `DOC-CANONICAL-RECONCILIATION-05`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Complete Phase Reconciliation Table

| Phase | Description / Focus | Primary Artifacts | Track | Validated Status |
|---|---|---|---|---|
| **Phase 0** | Real IO-VNBD Ingestion, Column Characterization & Schema Normalization | `IO_VNBD_CHARACTERIZATION_REPORT.md`, `idr_pipeline/data_loader.py` | Track B | **Verified** |
| **Phase 1** | Python Reference 15-State ESKF Formulation | `idr_pipeline/eskf.py`, `docs/phases/PHASE_1_NAVIGATION_CORE_REPORT.md` | Track B | **Verified** |
| **Phase 2** | Gravity Leveling & PCA Longitudinal Mount Alignment | `idr_pipeline/alignment.py`, `docs/phases/PHASE_2_ALIGNMENT_INITIALIZATION_REPORT.md` | Track B | **Verified** |
| **Phase 3** | Heading Observability & Magnetic Disturbance Rejection | `idr_pipeline/heading.py`, `docs/phases/PHASE_3_HEADING_OBSERVABILITY_REPORT.md` | Track B | **Verified** |
| **Phase 4** | GBDT AI Speed & Vibration Filter Formulation | `idr_pipeline/gbdt_speed.py`, `docs/phases/PHASE_4_AI_VELOCITY_VALIDATION_REPORT.md` | Track B | **Verified** ($R^2 = 0.884$) |
| **Phase 5** | 158-Outage Benchmark Harness & Error Tracking | `idr_pipeline/outage_evaluator.py`, `docs/phases/PHASE_5_OUTAGE_BENCHMARK_REPORT.md` | Track B | **Verified** |
| **Phase 6** | Causal 3D HMM Map Matching & Multi-Layer Elevation Barrier | `idr_map/`, `idr_pipeline/map_matcher.py`, `docs/phases/PHASE_6_OFFLINE_3D_OSM_MAP_MATCHING_REPORT.md` | Track B | **Verified** (99.7% layer acc) |
| **Phase 7** | $\chi^2$ Innovation Gating & GNSS Deficit Supervisor | `idr_pipeline/supervisor.py`, `docs/phases/PHASE_7_GNSS_INS_SUPERVISOR_REPORT.md` | Track B | **Verified** ($\gamma \le 11.35$) |
| **Phase 8** | Benchmark Closure & Baseline Formulation ($C_0, N_0$) | `docs/phases/PHASE_8_BENCHMARK_CLOSURE_REPORT.md` | Track B | **Verified** ($C_0=76.3\%, N_0=34.4\%$) |
| **Phase 9** | Core C++17 Extraction (`idr-core`), CMake, and Bitwise Parity | `idr-core/`, `tests/test_parity.py`, `docs/phases/PHASE_9_IMPLEMENTATION_REPORT.md` | Track A | **Verified Parity** |
| **Phase 10** | Track B Baseline Freeze ($N_7$) & Forensic Ablation Suite | `docs/phases/PHASE_10_TRACK_B_FINAL_FREEZE_REPORT.md`, `PHASE_10A-C` | Track B | **Frozen Reference** ($N_7=35.71\%$) |
| **Phase 11** | Android JNI Ingestion, Valhalla/OSM Vector Integration & Edge CLI | `android/`, `idr-cli/`, `docs/phases/PHASE_11_*` | Track A | **Verified Product** |
| **Phase 12** | Track B Failure Diagnostics & Corner-Case Analysis | `docs/phases/PHASE_12_TRACK_B_FINAL_REPORT.md` | Track B | **Verified Analysis** |
| **Phase 13** | Research: High-Frequency Feature Extraction & Energy Bands | `idr_pipeline/experiments/`, `docs/phases/PHASE_13_ACCURACY_RESEARCH_REPORT.md` | Track B | Research ($30.15\%$ med) |
| **Phase 14** | Research: Fast Feature Selection (Top 25/35/45) | `idr_pipeline/feature_selection.py`, `docs/phases/PHASE_14_FAST_ACCURACY_REPORT.md` | Track B | Research ($29.42\%$ med) |
| **Phase 15** | Research: Kinematic Jerk Derivative Ingestion | `idr_pipeline/experiments/`, `docs/phases/PHASE_15_FINAL_REPORT.md` | Track B | Research ($28.80\%$ med) |
| **Phase 16** | Research: Learned Adaptive $R_{\text{ai}}$ Measurement Noise | `idr_pipeline/adaptive_noise.py`, `docs/phases/PHASE_16_FINAL_REPORT.md` | Track B | Research ($27.25\%$ med) |
| **Phase 17** | Research: Unified Fusion Optimization & Standstill Suppression | `idr_pipeline/unified_fusion.py`, `docs/phases/PHASE_17_FINAL_REPORT.md` | Track B | Research ($27.64\%$ med, 49 pass) |

---

## 2. Production vs Research Allocation

- **C++ Production Engine (`idr-core/`):** Executes the robust, zero-risk **Phase 10 Frozen $N_7$ Baseline** with 15-state ESKF, GBDT top-45 regressor, NHC, ZUPT, and 3D HMM map matching.
- **Android App (`android/`):** Connects to `idr-core` via JNI, providing the 12-screen navigation interface, real geographic rendering, live NavIC tracking, and trip analytics.
- **Python Research Suite (`idr_pipeline/`):** Contains the complete experimental continuum from Phase 0 baseline to Phase 17 unified fusion, available for benchmarking, dataset re-training, and future algorithm exports.
