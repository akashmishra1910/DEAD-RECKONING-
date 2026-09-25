# Intelligent Dead Reckoning (IDR) & 3D Map-Matching System
## High-Precision Autonomous Navigation Engine for GNSS-Denied Environments
### Indian Space Research Organisation (ISRO) / Department of Space — PS 26168

[![SIH 2026](https://img.shields.io/badge/SIH-2026-blue.svg)](https://www.sih.gov.in/)
[![Problem Statement](https://img.shields.io/badge/ISRO-PS%2026168-orange.svg)](https://www.isro.gov.in/)
[![Regression Suite](https://img.shields.io/badge/Regression-263%2F263%20PASSED-brightgreen.svg)]()
[![Core Engine](https://img.shields.io/badge/Core-C%2B%2B17%20%7C%20Python%203.11-green.svg)]()
[![Target Platforms](https://img.shields.io/badge/Target-Android%20(10Hz)%20%7C%20Edge%20(200Hz)-purple.svg)]()
[![Mandated Dataset](https://img.shields.io/badge/Evaluated%20On-Authentic%20IO--VNBD-red.svg)](https://github.com/onyekpeu/IO-VNBD)

---

## 1. Authoritative Documentation Suite

All system specifications, forensic registers, benchmark matrices, and compliance audits are maintained in [`docs/canonical/`](docs/canonical/):

- 📘 [**Project Canonical State**](docs/canonical/PROJECT_CANONICAL_STATE.md) — Master product scope & dual-track governance.
- 📋 [**Project Forensic Inventory**](docs/canonical/PROJECT_FORENSIC_INVENTORY.md) — Directory-by-directory catalog of all assets.
- ⚙️ [**Track A Authoritative State**](docs/canonical/TRACK_A_AUTHORITATIVE_STATE.md) — C++17 core engine, edge CLI & Android product.
- 🔬 [**Track B Authoritative State**](docs/canonical/TRACK_B_AUTHORITATIVE_STATE.md) — Algorithmic research, ML models & evaluation.
- 🔄 [**Phase & Track Reconciliation**](docs/canonical/PHASE_TRACK_RECONCILIATION.md) — Traceability across Phase 0 to Phase 17 milestones.
- 📊 [**Authoritative Benchmark Register**](docs/canonical/AUTHORITATIVE_BENCHMARK_REGISTER.md) — 158-outage IO-VNBD quantitative ledger.
- 🛡️ [**PS Compliance Matrix**](docs/canonical/PS_COMPLIANCE_MATRIX_CURRENT.md) — Step-by-step ISRO PS 26168 compliance checklist.
- 🔍 [**Requirements Traceability**](docs/canonical/REQUIREMENTS_TRACEABILITY_CURRENT.md) — Requirement-to-code-to-test mapping.
- ⚖️ [**Final Product Gap Analysis**](docs/canonical/FINAL_PRODUCT_GAP_ANALYSIS.md) — Transparent evaluation of capabilities and gaps.
- 📜 [**Claims & Evidence Register**](docs/canonical/CLAIMS_AND_EVIDENCE.md) — Test-backed quantitative performance ledger.
- 🏗️ [**Final System Architecture**](docs/canonical/FINAL_ARCHITECTURE.md) — Comprehensive block diagrams and data flows.
- ✅ [**Final Release Checklist**](docs/canonical/FINAL_RELEASE_CHECKLIST.md) — Release verification matrix.
- 🌳 [**Final Project Tree**](docs/canonical/FINAL_PROJECT_TREE.md) — Canonical clean repository structure.
- 📝 [**Project Cleanup Report**](docs/canonical/PROJECT_CLEANUP_REPORT.md) — Detailed cleanup and canonicalization report.

---

## 2. System Architecture

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

---

## 3. Authoritative Performance Highlights

| Metric | C0 (Raw DR) | N0 (AI+ESKF) | N7 (Production C++) | Phase 16 (Python Res) |
|---|:---:|:---:|:---:|:---:|
| **Median Drift across 158 Outages** | 76.34% | 34.43% | **35.71%** | **27.25%** |
| **Drift @ 60s Outage** | 5.82% | 2.45% | **2.01%** | **1.84%** |
| **Drift @ 120s Outage** | 8.14% | 3.12% | **2.88%** | **2.41%** |
| **3D Flyover Layer Disambiguation** | N/A | N/A | **99.7% – 100%** | **99.8%** |
| **Core Latency per Sample** | <0.05 ms | <0.18 ms | **0.20 ms** | ~1.2 ms |
| **Memory Footprint (RSS)** | <1.0 MB | <2.0 MB | **<2.4 MB** | ~65 MB |

---

## 4. Quickstart & Verification

### Running the Python Regression Suite (263 Tests)
```bash
python -m pytest -q
```

### Compiling and Testing the Portable C++ Engine (`idr-core`)
```bash
cd idr-core
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest --output-on-failure
```

### Running the Standalone Edge CLI (`idr-cli`)
```bash
./idr-cli --input data/fixtures/test_imu_stream.csv --rate 200 --output trajectory_out.csv
```

### Building the Android Navigation App (`android/`)
```bash
cd android
./gradlew assembleDebug testDebugUnitTest
```
