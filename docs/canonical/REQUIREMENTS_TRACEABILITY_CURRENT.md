# REQUIREMENTS TRACEABILITY MATRIX — CURRENT
## System Requirements to Design, Code, and Verification Mapping

**Document ID:** `DOC-CANONICAL-TRACEABILITY-08`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Traceability Table

| Req ID | Requirement Statement | Architectural Module | Implementation Files | Automated Verification Test |
|---|---|---|---|---|
| **REQ-ALG-01** | Estimate vehicle speed from IMU without wheel odometry | AI Speed Regressor | `idr-core/src/gbdt_speed.cpp`, `idr_pipeline/gbdt_speed.py` | `tests/test_gbdt_speed.py`, `tests/test_parity.py` |
| **REQ-ALG-02** | Propagate 15-state ESKF with quaternion attitude | ESKF Engine | `idr-core/src/eskf.cpp`, `idr_pipeline/eskf.py` | `tests/test_eskf.py`, `tests/test_covariance.py` |
| **REQ-ALG-03** | Apply NHC to restrain non-forward motion | Body Frame NHC Update | `idr-core/src/eskf.cpp` | `tests/test_nhc.py` |
| **REQ-ALG-04** | Detect standstill and clamp velocity drift | ZUPT Detector | `idr-core/src/eskf.cpp` | `tests/test_zupt.py` |
| **REQ-ALG-05** | Disambiguate 3D road layers (Flyover / Ground / Tunnel) | 3D HMM Map Matcher | `idr-core/src/map_matcher.cpp`, `idr_map/` | `tests/test_map_matcher.py`, `tests/test_elevation_barrier.py` |
| **REQ-ALG-06** | Reject GNSS multipath and spurious position fixes | Innovation Supervisor | `idr-core/src/supervisor.cpp` | `tests/test_supervisor.py` |
| **REQ-CORE-01** | Portable C++17 library with zero Android dependencies | Core Engine | `idr-core/CMakeLists.txt`, `idr-core/include/idr/` | `tests/test_cpp_engine.py`, CTest harness |
| **REQ-CORE-02** | Sample-rate agnostic streaming | Sensor Adapter | `idr-core/src/sensor_adapter.cpp` | `tests/test_sensor_adapter.py` |
| **REQ-EDGE-01** | Process external IMU logs at ~200 Hz via CLI | CLI Runner | `idr-cli/src/main.cpp` | `tests/test_cli_runner.py` |
| **REQ-APP-01** | 10 Hz Android UI updates decoupled from 100 Hz sensors | App Synchronizer | `android/app/.../NavigationPipelineSynchronizer.kt` | `android/app/.../NavigationPipelineSynchronizerTest.kt` |
| **REQ-APP-02** | Real-time NavIC satellite constellation tracking | GNSS Ingestion | `android/app/.../GnssIngestionManager.kt` | `android/app/.../GnssIngestionManagerTest.kt` |
| **REQ-APP-03** | End-to-end trip navigation with OSM/Valhalla canvas | Navigation UI | `android/app/.../MainNavigationScreen.kt` | Android UI instrumentation tests |
| **REQ-APP-04** | 100% offline multi-layer map rendering | Map Canvas | `android/app/.../RealGeographicMapCanvas.kt` | Android UI tests |

---

## 2. Test Suite Summary

- Total Python Tests: **263 tests passing** (`python -m pytest -q`).
- Android Unit Tests: **All passing** (`./gradlew testDebugUnitTest`).
- C++ Bitwise Parity: **Verified within floating-point tolerance ($10^{-5}$)** between C++ and Python references.
