# COMPREHENSIVE REPOSITORY AUDIT

**Document ID:** `AUDIT-REPO-MASTER-01`  
**Date:** September 27, 2026  
**Status:** COMPLETED & VERIFIED  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Executive Overview

This audit provides a complete structural, algorithmic, and architectural inventory of the repository across Python research, C++ production core, and Android mobile application layers.

---

## 2. Research & Python Architecture Inventory (`idr_pipeline/`, `scripts/`, `models/`)

### 2.1 Dataset Loaders & Data Schemas
- `idr_pipeline/real_iovnbd_loader.py`: Authoritative parser for official IO-VNBD synchronized smartphone (`S-*.csv`) and vehicle OBD-II/CAN (`V-*.csv`) datasets.
- `idr_pipeline/outage_catalog.py`: Systematic catalog generator extracting 158 authentic outages (10s, 30s, 60s, 120s) across all 5 authentic drives.

### 2.2 Feature Generation & Preprocessing
- `idr_pipeline/dataset_loader.py` (`extract_sliding_window_features`): Causal 1.0s sliding window feature extractor generating 45 time- and frequency-domain kinematic features.
- `idr_pipeline/alignment_engine.py`: Batch and online PCA longitudinal axis identification and gravity leveling mounting calibration.
- `idr_pipeline/vibration_filter.py`: 4th-order Butterworth bandpass filter and engine idle frequency detector.

### 2.3 Estimators & Filter Formulations
- `idr_pipeline/eskf_fusion.py`: 15-state Error-State Kalman Filter in ENU navigation frame and FLU body frame. Implements prediction, altitude aiding, NHC, ZUPT, in-flight ZARU, and AI velocity update.
- `idr_pipeline/gnss_ins_fusion_engine.py`: 6-state supervisory transition engine (`SeamlessGNSSDeficitEngine`), arbitrating between `GNSS_AIDED`, `GNSS_DEGRADED`, `TRANSITION_TO_DR`, `DR_ONLY`, and `TRANSITION_TO_GNSS`.
- `idr_pipeline/speed_scale_bias_calibrator.py`: Causal online speed scale and bias observer.
- `idr_pipeline/heading_supervisor.py`: Tri-level heading supervisor fusing GNSS COG, calibrated magnetometer, and gyro integration.

### 2.4 Map Matching
- `idr_pipeline/offline_osm_graph.py`: Multi-layer topological road network graph built from OpenStreetMap data.
- `idr_pipeline/map_matcher_3d.py`: Causal 3D HMM map matcher featuring an elevation barrier ($|\Delta h| > 4.0\text{ m}$) preventing flyover jumping.

### 2.5 Historical Research Scripts & Benchmarks
- `scripts/run_phase10c_adaptive_nhc.py`: Evaluated historical N0 through N7 models (N7 = 35.71% drift under CAN speed).
- `scripts/run_phase14_ablation_benchmarks.py`: Fast A1 architecture (31.65% drift under CAN speed).
- `scripts/run_phase15_final_benchmark.py`: Bounded-slip NHC + ZARU (30.88% drift under CAN speed).
- `scripts/run_phase16_final_benchmark.py`: GNSS Doppler + ESKF Observability Upgrade (27.25% drift under CAN speed).
- `scripts/run_phase17_final_benchmark.py`: Dynamic Kinematic Confidence Modulation (27.64% drift under CAN speed).
- `scripts/run_phase16_17_ablation_study.py`: 11-configuration component ablation study.
- `scripts/run_autonomous_validation_suite.py`: Clean Regime B autonomous validation across 158 outages.

### 2.6 Model Artifacts
- `models/production/gbdt_speed_model_iovnbd_top45.joblib`: Authoritative 45-feature Huber Loss GBDT regressor (120 trees, depth 5, learning rate 0.08, Huber loss $\alpha=0.9$).

---

## 3. C++ Production Core Inventory (`idr-core/`)

Portable C++17 library with zero Android and zero external ML runtime dependencies:

| C++ Module | Header (`include/idr/`) | Implementation (`src/`) | Functional Description |
|---|---|---|---|
| **Mathematical Core** | `math_utils.hpp` | `math_utils.cpp` | SO(3) Lie algebra, Rodrigues formula, DCM-Euler conversions, skew-symmetric matrices. |
| **GBDT Regressor** | `gbdt_model.hpp` | `gbdt_model.cpp` | 120-tree Huber GBDT transpiled to branchless static decision tables. |
| **Feature Extraction** | `feature_extractor.hpp` | `feature_extractor.cpp` | C++ causal 1.0s sliding window feature extraction. |
| **Vibration Filter** | `vibration_filter.hpp` | `vibration_filter.cpp` | IIR Butterworth filter cascade and engine idling detection. |
| **In-Vehicle Alignment** | `alignment_engine.hpp` | `alignment_engine.cpp` | Static gravity leveling and PCA longitudinal alignment. |
| **15-State ESKF** | `eskf.hpp` | `eskf.cpp` | Eigen3-based 15-state ESKF, NHC, ZUPT, ZARU, and altitude update. |
| **Heading Supervisor** | `heading_supervisor.hpp` | `heading_supervisor.cpp` | Multi-source heading arbitrator. |
| **Map Matching** | `osm_graph.hpp`, `map_matcher_3d.hpp` | `osm_graph.cpp`, `map_matcher_3d.cpp` | Topological road graph and 3D HMM Viterbi matcher. |
| **Navigation Coordinator** | `navigation_engine.hpp` | `navigation_engine.cpp` | Top-level supervisory coordinator. |
| **C ABI Layer** | `c_api.h` | `c_api.cpp` | Hardened C ABI exports for JNI, ctypes, and external integration. |
| **Map Binary Reader** | `idrmap_reader.hpp` | `idrmap_reader.cpp` | High-speed binary parser for `.idrmap` network files. |

- **Build Output:** Static library `libidr_core.a` (529 KB) and shared library `libidr_core.dylib` / `libidr_core.so` (422 KB).
- **Test Suite:** 14 C++ CTest test binaries passing in `idr-core/build/`.

---

## 4. Android Mobile Application Inventory (`android/`)

Existing mobile application front-end:

### 4.1 Application Identity & Build Metadata
- **Package Name / Namespace:** `org.isro.idr`
- **Application ID:** `org.isro.idr`
- **Compile SDK:** 35 (Android 15)
- **Target SDK:** 34 (Android 14)
- **Min SDK:** 26 (Android 8.0 Oreo)
- **Version Code:** 1
- **Version Name:** 1.1.0
- **NDK Version:** 25.1.8937393
- **ABIs Supported:** `arm64-v8a`, `x86_64`
- **UI Framework:** Jetpack Compose (BOM 2024.02.00, Kotlin Compiler Extension 1.5.8)

### 4.2 UI & Navigation Inventory
- `MainActivity.kt`: Root activity hosting the navigation container.
- Screens:
  - `MainNavigationScreen.kt`: Primary HUD featuring real-time map, trajectory toggle (Raw GNSS vs. IDR), speed, heading, and road layer status.
  - `SensorMonitorScreen.kt`: Real-time IMU, magnetometer, and barometer waveforms.
  - `DiagnosticsScreen.kt`: Filter covariance trace, innovation gate statistics, and satellite constellation tracking.
  - `GnssNavicScreen.kt`: ISRO NavIC (IRNSS) satellite SNR, azimuth, elevation, and fix status.
  - `SettingsScreen.kt`: Offline map selection, vehicle profile configuration, and replay modes.
  - `TripAnalyticsScreen.kt`: Drift error summary, outage history, and trip logging.
  - `MapExplorerScreen.kt` & `OfflineMapsScreen.kt`: Map cache management and basemap styling.
  - `SplashScreen.kt`: Initialization and sensor availability verification.

### 4.3 Sensor Ingestion & JNI Integration
- `SensorIngestionManager.kt`: Continuous 100 Hz ingestion of hardware accelerometer, gyroscope, magnetometer, and barometer.
- `BoundedSensorQueue.kt`: Thread-safe, non-blocking ring buffer preventing heap exhaustion.
- `NativeNavigationEngine.kt`: Kotlin singleton wrapping `libidr_jni.so`.
- `jni_bridge.cpp`: JNI C++ boundary converting Android sensor events into native C API calls.
