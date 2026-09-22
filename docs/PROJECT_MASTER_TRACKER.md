# Project Master Tracker
## ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Current Phase:** PHASE 0 — REPOSITORY FORENSICS & BASELINE FREEZE  
**Overall Status:** IN PROGRESS (Phase 0 Baseline Complete; Ready for Phase 1 Navigation Core Validation)  
**Last Updated:** 2026-09-23  
**Active Git Commit:** `f58b9210fd51e80291df5d96e98fcbe8fdb7be8d`  
**Execution Environment:** Windows (x86_64) | Python 3.11.0 | NumPy 2.4.6 | Scikit-Learn 1.9.1 | SciPy 1.17.1 | Joblib 1.4.2 | Java 25.0.2 | MinGW GCC 6.3.0  

---

## 1. Phase Progression Overview

| Phase | Phase Name | Status | Gate Status | Blockers / Critical Notes |
|:---:|---|:---:|:---:|---|
| **Phase 0** | Repository Forensics & Baseline Freeze | **IN PROGRESS** | **READY TO PASS** | Environment audited, unit/integration tests verified, 8 master docs established. |
| **Phase 1** | Navigation Core Validation | NOT STARTED | PENDING | Mathematical invariants (Rodrigues SO(3), ENU/FLU, error injection, P symmetry). |
| **Phase 2** | Alignment & Initialization Hardening | NOT STARTED | PENDING | Multi-mode alignment (stationary, slow start, surge, confidence metrics). |
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
- `test_suite.py`: **6 Passed / 0 Failed** (Run duration: 9.99s).
- `tests/test_eskf_fusion.py`: **14 Passed / 0 Failed** (Run duration: 0.355s).
- Overall baseline: **20 Passed / 0 Failed**.

### Files Modified in Phase 0
- `idr_pipeline/ai_velocity_model.py` (added `Optional` to typing imports).
- `docs/PROJECT_MASTER_TRACKER.md` (NEW).
- `docs/REQUIREMENTS_TRACEABILITY_MATRIX.md` (NEW).
- `docs/ARCHITECTURE.md` (NEW).
- `docs/VALIDATION_POLICY.md` (NEW).
- `docs/CHANGELOG.md` (NEW).
- `docs/KNOWN_FAILURES.md` (NEW).
- `docs/DECISIONS.md` (NEW).
- `docs/PS_COMPLIANCE_MATRIX.md` (NEW).

---

## 3. Next Actions
1. **Pass Phase 0 Gate**: Sign off baseline documentation and test records.
2. **Launch Phase 1 (Navigation Core Validation)**: Conduct complete mathematical audit of error-state kinematics, Rodrigues propagation, covariance updates, and observability in `idr_pipeline/eskf_fusion.py`.
