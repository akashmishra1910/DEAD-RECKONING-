# Project Master Tracker
## ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Current Phase:** PHASE 4 — AI VELOCITY MODEL VALIDATION & HARDENING (COMPLETED / READY FOR PHASE 5)  
**Overall Status:** IN PROGRESS (Phase 4 AI Velocity Model Validated, Leakage Audited, and Frozen; Ready for Phase 5 AI Fusion / Adaptive Noise)  
**Last Updated:** 2026-09-23  
**Active Git Commit:** `phase-04-ai-velocity-validation`  
**Execution Environment:** Windows (x86_64) | Python 3.11.0 | NumPy 2.4.6 | Scikit-Learn 1.9.1 | SciPy 1.17.1 | Joblib 1.4.2 | Java 25.0.2 | MinGW GCC 6.3.0  

---

## 1. Phase Progression Overview

| Phase | Phase Name | Status | Gate Status | Blockers / Critical Notes |
|:---:|---|:---:|:---:|---|
| **Phase 0** | Repository Forensics & Baseline Freeze | **COMPLETED** | **PASSED** | Environment audited, unit/integration tests verified, 8 master docs established. |
| **Phase 1** | Navigation Core Validation | **COMPLETED** | **PASSED** | Mathematical invariants verified (SO(3), ENU/FLU, Rodrigues, Jacobians, P symmetry; 33/33 tests passed). |
| **Phase 2** | Alignment & Initialization Hardening | **COMPLETED** | **PASSED** | Multi-mode alignment engine, stationary start protection (KF-02 resolved), AlignmentQuality diagnostics, 20/20 tests passed. |
| **Phase 3** | Attitude / Heading Observability | **COMPLETED** | **PASSED** | Multi-source HeadingSupervisor, GNSS course deriver, 3D mag disturbance detector, multi-epoch reacquisition, 24/24 tests passed (83/83 total regression). |
| **Phase 4** | AI Velocity Model Validation | **COMPLETED** | **PASSED** | Top-45 Huber GBDT validated across 10,019 authentic IO-VNBD samples (RMSE 3.63 m/s, zero leakage, 15/15 tests passed, 98/98 full regression). |
| **Phase 5** | AI-Based Adaptive Measurement Noise | NOT STARTED | PENDING | Learned $R_v(t)$ conditioned on vibration energy, jerk, and turn rate. |
| **Phase 6** | 3D Offline OSM Map-Matching | NOT STARTED | PENDING | Multi-layer topological graph, Viterbi HMM, $4\text{ m}$ vertical elevation barrier. |
| **Phase 7** | Full GNSS / INS Supervisor | NOT STARTED | PENDING | Seamless state machine, $\chi^2$ innovation gate ($\gamma \le 11.35$), outage handover. |
| **Phase 8** | Two-Wheeler Mode | NOT STARTED | PENDING | Roll-compensated NHC and lateral dynamics for motorcycles/scooters. |
| **Phase 9** | Portable C++ Core (`idr-core`) | NOT STARTED | PENDING | C++17 standalone library with CMake, sample-rate agnostic, 200 Hz capable. |
| **Phase 10** | GBDT Edge Deployment | NOT STARTED | PENDING | Transpiled branchless static C++ decision tables matching Python Top45 bit-for-bit. |
| **Phase 11** | Android Application (`idr-android`) | NOT STARTED | PENDING | Kotlin foreground service, NDK JNI bridge, 100 Hz sensor buffer, 10 Hz UI. |
| **Phase 12** | NavIC Integration | NOT STARTED | PENDING | Android `GnssMeasurement` constellation decoding (`CONSTELLATION_IRNSS`). |
| **Phase 13** | Offline Mode | NOT STARTED | PENDING | Zero network calls, local MBTiles/OSM graph, standalone weights. |
| **Phase 14** | Replay / Demonstration Mode | NOT STARTED | PENDING | Interactive simulation cockpit for real IO-VNBD logs with live sensor toggles. |
| **Phase 15** | Performance Engineering | NOT STARTED | PENDING | Execution profiling: $<1\text{ ms}$ latency, $<2.5\text{ MB}$ RAM footprint verification. |
| **Phase 16** | Complete Benchmark Matrix | NOT STARTED | PENDING | Reproducible multi-outage evaluation (10s, 30s, 60s, 120s) across all configs. |
| **Phase 17** | Authentic IO-VNBD Validation | NOT STARTED | PENDING | 72-drive comprehensive naturalistic evaluation without synthetic leakage. |
| **Phase 18** | PS Compliance Validation | NOT STARTED | PENDING | Evidence-based evaluation against all ISRO sponsor criteria. |
| **Phase 19** | Final System Integration | NOT STARTED | PENDING | End-to-end integration tests spanning Python, C++, JNI, and Android. |
| **Phase 20** | Final Demo Package & Delivery | NOT STARTED | PENDING | Final delivery package, APK, CLI binary, reproducibility docs, demo script. |

---

## 2. Test Execution Summary

### Regression Test Suite:
- `test_suite.py`: **6 Passed / 0 Failed** (Edge engine throughput: 1817.4 Hz).
- `tests/test_eskf_fusion.py`: **14 Passed / 0 Failed**.
- `tests/test_phase1_navigation_core.py`: **19 Passed / 0 Failed**.
- `tests/test_phase2_alignment_initialization.py`: **20 Passed / 0 Failed**.
- `tests/test_phase3_heading_observability.py`: **24 Passed / 0 Failed**.
- `tests/test_phase4_ai_velocity.py`: **15 Passed / 0 Failed**.
- **Total Regression Suite:** **98 Passed / 0 Failed (100% Pass Rate)**.

### Files Modified/Created in Phase 4
- `models/gbdt_speed_model_iovnbd_top45.joblib` (FROZEN: Verified 45 features, zero data leakage, $297\ \mu\text{s}$ inference).
- `reports/phase4_feature_leakage_audit.csv` (NEW: 45/45 features causality & window audit).
- `reports/phase4_per_drive_generalization.csv` (NEW: 10,019 samples across 5 authentic drives).
- `reports/phase4_speed_regimes_breakdown.csv` (NEW: Kinematic breakdown by speed bin & motion type).
- `scripts/evaluate_phase4_ai_velocity_validation.py` (NEW: Evaluation & benchmark script).
- `tests/test_phase4_ai_velocity.py` (NEW: 15 dedicated mathematical, physical & temporal validation tests).
- `docs/PHASE_4_AI_VELOCITY_VALIDATION_REPORT.md` (NEW: Comprehensive Phase 4 report).

---

## 3. Next Actions
1. **Standby for Phase 5 Authorization**: Proceed to Phase 5 — AI-Based Adaptive Measurement Noise (Learned $R_v(t)$ conditioned on vibration energy, jerk, and turn rate).
