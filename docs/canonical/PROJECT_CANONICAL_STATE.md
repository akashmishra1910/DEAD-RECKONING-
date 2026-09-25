# PROJECT CANONICAL STATE — SIH26168 (ISRO)
## AI/ML Based Intelligent Dead Reckoning System for Seamless Navigation

**Document ID:** `DOC-CANONICAL-STATE-01`  
**Version:** 2.0.0 (Canonical Release)  
**Date:** September 25, 2026  
**Status:** AUTHORITATIVE — Frozen Architecture & Verified Metrics  

---

## 1. Executive Summary

This repository hosts the complete, validated software deliverables for **SIH 2026 Problem Statement 26168 (ISRO / Department of Space)**: *AI-ML based Intelligent Dead Reckoning system for seamless navigation*.

The system addresses the fundamental failure of GNSS in satellite-denied environments (urban canyons, flyovers, tunnels, underpasses, parking structures) by combining:
1. **Low-latency On-Device IMU Ingestion** (100 Hz phone sensor pipeline / ~200 Hz external IMU engine).
2. **AI Forward Speed & Vibration Filtering** (Transpiled branchless GBDT regressor).
3. **15-State Error-State Kalman Filter (ESKF)** with Non-Holonomic Constraints (NHC) and Zero-Velocity Updates (ZUPT).
4. **Causal 3D HMM Multi-Layer Map Matching** with elevation disambiguation barriers preventing impossible flyover transitions.
5. **Chi-Square Innovation-Gated GNSS Deficit Supervisor** for multipath and tunnel-exit glitch rejection.

---

## 2. Product Deliverables & Dual-Track Separation

```
========================================================================================
                                SIH26168 SYSTEM ARCHITECTURE
========================================================================================

                 [ TRACK A: PRODUCTION DELIVERABLES ]
                                    │
    ┌───────────────────────────────┴───────────────────────────────┐
    │                                                               │
┌───▼───────────────────────────┐               ┌───────────────────▼───────────────────┐
│ idr-core (C++17 Engine)       │               │ idr-android (Android App)             │
│ - Portable C++17 Core Engine  │ ──JNI Bridge─►│ - 12-Screen Global Geographic Product │
│ - Sample-rate agnostic        │               │ - Valhalla / OSM Vector Integration   │
│ - Zero external ML runtime    │               │ - Live NavIC Constellation Monitor    │
│ - Frozen N7 Validated Reference│              │ - 100% Offline Multi-Layer Map Engine │
└───────────────────────────────┘               └───────────────────────────────────────┘
    │
    └───► idr-cli (Edge / External IMU CLI)
          - Runs at ~200 Hz on desktop/embedded Linux/FOG-grade IMU streams

────────────────────────────────────────────────────────────────────────────────────────

                 [ TRACK B: ALGORITHMIC RESEARCH & BENCHMARKS ]
                                    │
    ┌───────────────────────────────┴───────────────────────────────┐
    │                                                               │
┌───▼───────────────────────────┐               ┌───────────────────▼───────────────────┐
│ idr_pipeline (Python Engine)  │               │ Experimental Research (Phase 13-17)   │
│ - Reference 15-state ESKF     │               │ - Phase 16 Adaptive GBDT (27.25% med) │
│ - Transpiler & Feature Extr.  │               │ - Phase 17 Unified Fusion (27.64% med)│
│ - 158-Outage Benchmark Engine │               │ - Retained in Python research suite   │
└───────────────────────────────┘               └───────────────────────────────────────┘
========================================================================================
```

### Track A: Production Product (C++ / Android / CLI)
- **Engine Core:** `idr-core/` (C++17, CMake, zero external ML dependencies).
- **Mobile Front-End:** `android/` (Kotlin + Jetpack Compose 12-Screen Global Geographic Navigation App).
- **Edge CLI:** `idr-cli/` (Desktop & embedded terminal runner for live file/socket external IMU feeds).
- **Active Algorithmic Profile:** **Frozen N7 Production Reference** ($C_0 = 76.34\%, N_0 = 34.43\%, N_7 = 35.71\%$).

### Track B: Algorithmic Research & Evaluation (Python)
- **Pipeline Package:** `idr_pipeline/` (Python research reference for training, feature extraction, simulation, and benchmark replay).
- **Ablation & Research Phases:** Documented in `docs/phases/` and tested via `tests/`.
- **Best Research Results:** Phase 16 Adaptive GBDT ($27.25\%$ median error, $45/158$ passes); Phase 17 Unified Fusion ($27.64\%$ median error, $49/158$ passes).
- **Parity Principle:** Research algorithms remain clearly marked in `idr_pipeline/` until full C++ transpilation and bitwise-parity regression tests are executed.

---

## 3. Authoritative Benchmark Invariants

The evaluation across all 158 real GNSS outages from the mandated **IO-VNBD** dataset is strictly preserved and honest:

| Metric / Configuration | C0 (Raw DR) | N0 (AI-Speed + ESKF) | N7 (Production C++ Ref) | Phase 16 (Python Res) | Phase 17 (Unified Res) |
|---|---|---|---|---|---|
| **Median Drift %** | 76.34% | 34.43% | 35.71% | 27.25% | 27.64% |
| **Mean Drift %** | 94.69% | 40.70% | 41.52% | 32.74% | 33.17% |
| **Passes (<10% Bound)** | 5 / 158 | 32 / 158 | 29 / 158 | 45 / 158 | 49 / 158 |
| **Drift @ 60s Outage** | 5.82% | 2.45% | 2.01% | 1.84% | 1.82% |
| **Drift @ 120s Outage** | 8.14% | 3.12% | 2.88% | 2.41% | 2.38% |
| **Layer Disambiguation** | N/A | N/A | 99.7% – 100% | 99.8% | 100.0% |
| **Inference Latency** | <0.05 ms | <0.18 ms | 0.20 ms | ~1.2 ms | ~1.4 ms |

> **Honesty Mandate:** The ISRO benchmark threshold of $<10\%$ drift across **all 158 diverse outages** is challenging for pure phone-grade IMU without wheel odometry; while short and medium outages achieve $<2.88\%$, the overall dataset-wide median error is $35.71\%$ in the C++ engine and $27.25\%$ in Python research. All documentation explicitly acknowledges this reality.

---

## 4. Key Repository Structure

- `idr-core/`: Portable C++17 library (Alignment, GBDT, ESKF, 3D Map Matching, Deficit Supervisor).
- `idr-cli/`: Standalone CLI runner consuming external CSV/binary IMU streams.
- `android/`: 12-Screen Jetpack Compose Android Navigation Application with JNI integration.
- `idr_pipeline/`: Python pipeline (Feature Extraction, GBDT Transpilation, ESKF, Replay Harness).
- `models/`:
  - `production/`: Transpiled and deployment joblib models (`gbdt_speed_model_iovnbd_top45.joblib`).
  - `experiments/`: Research and feature-selection ablations (`top25`, `top35`, `frequency_experiment`, etc.).
- `data/`:
  - `fixtures/`: Test datasets and sample `.idrmap` files (`prayagraj_corridor.idrmap`, etc.).
- `docs/`:
  - `canonical/`: Authoritative master documentation suite.
  - `phases/`: Historical Phase 0 to Phase 17 milestone and ablation reports.
  - `archive/`: Legacy documentation and historical draft material.
- `tools/`: Edge dashboard, web app visualization, and data conversion scripts.
- `tests/`: 263 Unit and Integration regression tests (All Passing).
