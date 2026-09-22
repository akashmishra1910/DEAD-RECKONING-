# Project Master Tracker
## ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Current Phase:** PHASE 2 — ALIGNMENT & INITIALIZATION HARDENING (COMPLETED / READY FOR PHASE 3)  
**Overall Status:** IN PROGRESS (Phase 2 Hardening Passed; Ready for Phase 3 Heading Observability)  
**Last Updated:** 2026-09-23  
**Active Git Commit:** `phase-02-alignment`  
**Execution Environment:** Windows (x86_64) | Python 3.11.0 | NumPy 2.4.6 | Scikit-Learn 1.9.1 | SciPy 1.17.1 | Joblib 1.4.2 | Java 25.0.2 | MinGW GCC 6.3.0  

---

## 1. Phase Progression Overview

| Phase | Phase Name | Status | Gate Status | Blockers / Critical Notes |
|:---:|---|:---:|:---:|---|
| **Phase 0** | Repository Forensics & Baseline Freeze | **COMPLETED** | **PASSED** | Environment audited, unit/integration tests verified, 8 master docs established. |
| **Phase 1** | Navigation Core Validation | **COMPLETED** | **PASSED** | Mathematical invariants verified (SO(3), ENU/FLU, Rodrigues, Jacobians, P symmetry; 33/33 tests passed). |
| **Phase 2** | Alignment & Initialization Hardening | **COMPLETED** | **PASSED** | Multi-mode alignment engine, stationary start protection (KF-02 resolved), AlignmentQuality diagnostics, 20/20 tests passed. |
| **Phase 3** | Attitude / Heading Observability | NOT STARTED | PENDING | Magnetic heading + disturbance rejection, GNSS course, road heading. |
| **Phase 4** | AI Velocity Model Validation | NOT STARTED | PENDING | Top45 Huber GBDT verification, feature scaling, 0–60 km/h metric interpretation. |
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

## 2. Current Phase 0 Status Details

### Completed Tasks
- [x] Full recursive repository forensic audit (identified 128 items, 32 python scripts, 6 real driving datasets, 6 model checkpoints).
- [x] Audited runtime environment: Windows OS, Python 3.11.0, upgraded NumPy to 2.4.6 and Scikit-Learn to 1.9.1 to ensure bit-level compatibility with frozen checkpoints.
- [x] Fixed typing import bug (`Optional`) in `idr_pipeline/ai_velocity_model.py`.
- [x] Verified unit test suite (`tests/test_eskf_fusion.py` — 14/14 tests PASSED).
- [x] Verified full integration suite (`test_suite.py` — 6/6 tests PASSED, 1064 Hz edge throughput achieved).
- [x] Verified benchmark evaluation harness (`scripts/evaluate_isro_benchmarks.py` — executed on authentic drive `Vta01b`).
- [x] Established the 8 master engineering governance documents in `docs/`.

### In Progress
- [ ] Phase 0 Gate sign-off and baseline freeze confirmation.

### Blocked
- *None.*

### Tests Passed / Failed
- `test_suite.py`: **6 Passed / 0 Failed** (Edge engine throughput: 1064.3 Hz).
- `tests/test_eskf_fusion.py`: **14 Passed / 0 Failed**.
- `tests/test_phase1_navigation_core.py`: **19 Passed / 0 Failed**.
- `tests/test_phase2_alignment_initialization.py`: **20 Passed / 0 Failed**.
- Overall regression test suite: **59 Passed / 0 Failed** (100% pass rate).

### Files Modified in Phase 2
- `idr_pipeline/alignment_engine.py` (Multi-mode state machine, stationary start protection, gyro bias estimation, AlignmentQuality diagnostics, SO(3) SVD re-orthogonalization).
- `tests/test_phase2_alignment_initialization.py` (NEW: 20 dedicated unit tests).
- `docs/PHASE_2_ALIGNMENT_INITIALIZATION_REPORT.md` (NEW).
- `docs/PROJECT_MASTER_TRACKER.md` (Updated).
- `docs/CHANGELOG.md` (Updated).
- `docs/KNOWN_FAILURES.md` (Updated: KF-02 resolved).
- `docs/DECISIONS.md` (Updated: ADR-008 recorded).
- `docs/REQUIREMENTS_TRACEABILITY_MATRIX.md` (Updated).
- `docs/PS_COMPLIANCE_MATRIX.md` (Updated).

---

## 3. Next Actions
1. **Standby for Phase 3 Authorization**: Implement Heading Observability & Heading Aiding (`HeadingMeasurementManager`, Magnetometer anomaly gating, GNSS Course-Over-Ground aiding).
