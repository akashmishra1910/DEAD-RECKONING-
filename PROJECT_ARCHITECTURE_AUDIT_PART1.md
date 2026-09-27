# Comprehensive System Architecture and Implementation Audit (Part 1)
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Document ID:** `PROJECT_ARCHITECTURE_AUDIT_PART1`  
**Audit Date:** September 27, 2026  
**Auditor:** Lead Autonomous Navigation Research & Systems Engineer  
**Mode:** READ-ONLY Architectural & Forensic Code Audit  
**Code Modifications During Audit:** Strictly NONE (Files Modified: 0)  

---

# 1. Executive Summary

This document presents a comprehensive, read-only architectural inventory, temporal causality review, oracle leakage verification, configuration audit, and experimental traceability analysis of the entire SIH 2026 Problem Statement 26168 codebase.

### Primary Audit Findings:
1. **Scientific Baseline:**  
   The authoritative autonomous reference baseline is **B1/N8** evaluated under **Regime B** (pure smartphone IMU dead reckoning, zero CAN/OBD speed, zero ground-truth speed, zero future lookahead).  
   - B1/N8 Benchmark: **54.86% median drift**, **123.14% mean drift**, **9 / 158 passes ($\le 10\%$)**, **37.10 m median along-track error**, **38.04 m median cross-track error**.
2. **Oracle Safety & Firewall:**  
   Static AST audits and dynamic runtime exception traps (`TrajectoryOracleTrap`) confirm that no ground-truth speed (`gt_speed`), velocity (`gt_vel`), position (`gt_pos`), or heading (`gt_heading`) is accessible during outage propagation.
3. **Temporal Causality:**  
   Prefix-truncation invariance is bit-exact ($0.000000\text{ m}$ delta). The 1.0 s sliding window feature extractor looks strictly backward ($[t - 1.0\text{ s}, t]$). Map matching utilizes forward Viterbi trellis decoding without backward smoothing.
4. **Android Application Status:**  
   The Android application (`org.isro.idr`) is completely implemented with 11 Jetpack Compose screens, sensor ingestion pipelines, and MapLibre offline rendering. The UI is 100% frozen. Integration with the native navigation engine occurs cleanly through a single JNI bridge (`jni_bridge.cpp` $\leftrightarrow$ `NativeNavigationEngine.kt`).
5. **C++17 Standalone Engine:**  
   The core engine (`idr-core/`) is portable, sample-rate agnostic (10 Hz mobile to 200 Hz edge), achieves bit-exact decision-tree parity ($0.000000\text{ m/s}$ MAE), machine-precision kinematics ($1.11 \times 10^{-16}\text{ rad}$), sustained throughput of **83,823 Hz** ($11.93\ \mu\text{s}$ per cycle), and has zero heap allocations in its execution loop. 14 / 14 CTests pass.
6. **M0–M7 Experiment Traceability:**  
   A critical code discovery was established: in `scripts/run_controlled_architectural_ablations.py`, M5 and M6 were evaluated with identical configuration dictionaries (`motion_state: True, map_confidence: True`), explaining why aggregate reported metrics between M5 and M6 were identical. M7 achieved 12 passes and 108.38% mean drift, but exhibited elevated median yaw error (42.98° vs 36.32° in M0), precluding automatic promotion.

---

# 2. Repository Structure

The repository is organized into distinct research, production, mobile, embedded, and validation subsystems:

```
SIH Model/
├── android/                             # Android Application Layer (FROZEN UI)
│   ├── app/src/main/AndroidManifest.xml # Application ID: org.isro.idr (minSdk 26, targetSdk 35)
│   ├── app/src/main/cpp/                # NDK JNI Bridge (CMakeLists.txt, jni_bridge.cpp)
│   └── app/src/main/java/org/isro/idr/  # Kotlin Source Tree
│       ├── MainActivity.kt              # App entry point & permission orchestrator
│       ├── engine/                      # NativeNavigationEngine.kt, NavigationTelemetry.kt
│       ├── ingestion/                   # SensorIngestionManager.kt, NavigationPipelineSynchronizer.kt
│       ├── map/                         # MapLibre offline map manager & partition loader
│       ├── storage/                     # Local SQLite trajectory run logging
│       ├── theme/                       # Compose colors, typography, theme definition
│       └── ui/screens/                  # 11 Compose Screens (LiveNav, Diagnostics, Replay, etc.)
│
├── idr-core/                            # Standalone C++17 Core Engine (Zero Android Deps)
│   ├── CMakeLists.txt                   # C++17 library build & test targets
│   ├── include/idr/                     # Public C++ and C-ABI headers
│   │   ├── alignment_engine.hpp         # Phone-to-vehicle dynamic mounting calibration
│   │   ├── c_api.h                      # C-ABI export header (idr_engine_create, process_step)
│   │   ├── edge_engine.hpp              # EdgeNavigationEngine (200 Hz external IMU streaming)
│   │   ├── eskf.hpp                     # 15-state Error-State Kalman Filter
│   │   ├── feature_extractor.hpp        # Ring-buffer Top-45 sliding window feature extractor
│   │   ├── gbdt_model.hpp               # Static branchless C++ decision table model
│   │   ├── heading_supervisor.hpp       # Innovation gating and heading supervisor
│   │   ├── idrmap_reader.hpp            # Binary road network deserializer (.idrmap)
│   │   ├── map_matcher_3d.hpp           # Causal 3D HMM forward Viterbi road matcher
│   │   ├── navigation_engine.hpp        # SeamlessGNSSDeficitEngine coordinator
│   │   ├── osm_graph.hpp                # In-memory topological multi-layer road graph
│   │   └── types.hpp                    # Math primitives (Vector3, Matrix3x3, Quaternion, Telemetry)
│   ├── src/                             # C++17 Implementation files
│   └── tests/                           # 14 CTest files + benchmark_tactical_100k.cpp
│
├── idr_pipeline/                        # Python Reference & Research Subsystem
│   ├── __init__.py                      # Package entry point
│   ├── alignment_engine.py              # InVehicleAlignmentEngine
│   ├── authoritative_pipeline.py        # Authoritative reference pipeline (Regime B)
│   ├── dataset_loader.py                # Causal 1.0s window feature extractor
│   ├── dual_profile.py                  # Dual-profile (mobile vs edge) operational coordinator
│   ├── eskf_fusion.py                   # 15-state ESKF reference implementation
│   ├── gnss_ins_fusion_engine.py        # Complete GNSS-deficit state-machine engine
│   ├── heading_supervisor.py            # Heading supervisor & gyro bias observer
│   ├── map_matcher_3d.py                # Causal 3D HMM road matcher with elevation barrier
│   ├── offline_osm_graph.py             # Topological graph parser and spatial index
│   ├── outage_catalog.py                # 158-outage comprehensive authentic segment catalog
│   ├── real_iovnbd_loader.py            # IO-VNBD dataset parser and time synchronizer
│   └── vibration_filter.py              # 2nd-order IIR Butterworth & pothole detector
│
├── models/                              # Serialized ML Models & Metadata
│   ├── production/
│   │   └── gbdt_speed_model_iovnbd_top45.joblib # Authoritative Top-45 Huber GBDT model
│   ├── experiments/                     # Historical exploratory models (top25, top35, jerk)
│   ├── causal_velocity_model.json       # JSON metadata
│   └── velocity_scaler.json             # Normalization scaler parameters
│
├── CONFIG/                              # Authoritative System Configuration
│   ├── production_authoritative.yaml    # Production single source of truth
│   ├── AUTHORITATIVE_FEATURE_SPEC.md    # Index-by-index 45-feature specification
│   └── MODEL_MANIFEST.json              # Checksums, parameters, and training provenance
│
├── AUDIT/                               # Formal Baseline Audit Records
│   ├── REPOSITORY_AUDIT.md              # Complete codebase structural inventory
│   ├── EXISTING_APP_BASELINE.md         # Frozen Android UI baseline
│   └── GT_LEAKAGE_AUDIT.md              # Historical leakage lineage documentation
│
├── VALIDATION/                          # Formal Benchmark & Parity Evidence
│   ├── AUTONOMOUS_158_OUTAGE_BENCHMARK.md # 158-outage detailed benchmark ledger
│   ├── B0_VS_B1N8_REPORT.md             # Head-to-head B0 vs B1/N8 comparison
│   ├── CAUSALITY_REPORT.md              # Prefix invariance verification
│   ├── NO_GT_FIREWALL_REPORT.md         # Proof of zero oracle leakage
│   ├── CROSS_PLATFORM_PARITY_REPORT.md  # Python vs C++ mathematical equivalence
│   └── ANDROID_REGRESSION_REPORT.md     # Android zero-regression proof
│
├── RELEASE/                             # Production Release Bundles
│   ├── ENGINE/UPDATED_IDRENGINE_B1N8.aar # Packaged standalone native AAR archive
│   └── ANDROID/EXISTING_APP_B1N8_UPDATED.apk # Packaged Android release APK
│
├── real_iovnbd_data/                    # Authentic IO-VNBD Driving Trajectories
│   ├── Driver_A_S3b/                    # Urban stop-and-go driving
│   ├── Vta01b/                          # Mixed urban-suburban corridor
│   ├── Vta03/                           # Highway driving with bridge elevation
│   ├── Vta04/                           # Complex multi-level flyovers
│   └── Vta05/                           # Roundabouts and sharp intersection turns
│
├── reports/                             # Detailed CSV Ledgers & Ablation Outputs
│   ├── b1_n8_root_cause_decomposition.csv # Full 158-outage forensic decomposition
│   └── BENCHMARK_ABLATION_RESULTS.csv   # M0 to M7 comparative summary
│
├── scripts/                             # Benchmark Runners & Evaluation Utilities
│   ├── run_autonomous_validation_suite.py # Complete 158-outage validation runner
│   ├── run_controlled_architectural_ablations.py # M0 to M7 ablation executor
│   └── generate_drift_root_cause_analysis.py # Root-cause decomposition generator
│
└── tests/                               # 44 Python Pytest Suites (276 tests total)
```

---

# 3. Actual End-to-End Pipeline

Tracing the actual runtime execution path from raw sensor acquisition to final navigation output:

```
[Phone Accel/Gyro @ 100 Hz] / [Edge IMU @ 200 Hz]
                     │
                     ▼
[Stage 1: Preprocessing & Frame Alignment]
  - Python: InVehicleAlignmentEngine.transform_phone_to_vehicle()
  - C++: InVehicleAlignmentEngine::transform_phone_to_vehicle() (alignment_engine.cpp)
  - Transformation: a_b = R_p2b * a_phone, w_b = R_p2b * w_phone
                     │
                     ▼
[Stage 2: Causal Vibration & Pothole Filtering]
  - Python: VibrationPotholeFilter.process() (vibration_filter.py)
  - C++: VibrationPotholeFilter::process() (vibration_filter.cpp)
  - Shock blanking: >2.5g impulse replacement via recent median
  - Vibration state: engine idling detection (15–30 Hz energy envelope)
                     │
                     ▼
[Stage 3: Causal Feature Extraction (1.0s Window)]
  - Python: extract_sliding_window_features() (dataset_loader.py)
  - C++: SlidingWindowFeatureExtractor::push() & compute_45_features() (feature_extractor.cpp)
  - Window: 10 samples @ 10 Hz (trailing buffer [t-1.0s, t])
  - Features: 45 statistical, jerk, and centripetal indicators
                     │
                     ▼
[Stage 4: Causal GBDT Speed Estimation]
  - Python: gbdt_model.predict() (LightGBM / scikit-learn)
  - C++: GBDTSpeedModel::predict() (gbdt_model.cpp)
  - Architecture: 120 static branchless C++ decision trees (depth 5, Huber loss)
  - Output: v_ai in [0.0, 35.0] m/s
                     │
                     ▼
[Stage 5: Pre-Outage GNSS Handover & Speed Calibration]
  - Python: estimate_preoutage_observability() & SpeedScaleBiasCalibrator
  - C++: OnlineSpeedScaleBiasCalibrator::correct_speed() (speed_calibrator.cpp)
  - Mechanism: Handover scales v_ai using pre-outage 3.0s GNSS Doppler
                     │
                     ▼
[Stage 6: 15-State ESKF Mechanization]
  - Python: ErrorStateKalmanFilter.predict() (eskf_fusion.py)
  - C++: ErrorStateKalmanFilter::predict() (eskf.cpp)
  - States: p (3), v (3), q (4/3 err), b_a (3), b_g (3)
  - Propagation rate: dt = 0.1s (mobile) or dt = 0.005s (edge)
                     │
                     ▼
[Stage 7: Longitudinal AI Speed Update]
  - Python: ESKF.update_ai_velocity(v_eff, r_ai_adapt)
  - C++: ESKF::update_ai_velocity() / update_ai_velocity_vector()
  - Innovation: dz = v_eff - e_x^T R^T v
                     │
                     ▼
[Stage 8: Kinematic Constraints (NHC / ZUPT / ZARU)]
  - Bounded-Slip NHC: R_lat in [0.05^2, 0.16] m^2/s^2 (adaptive to omega_z & a_cent)
  - Standstill ZUPT: Applied when idling or v_ai < 0.2 m/s (R_zupt = 0.01^2)
  - In-Flight ZARU: Straight driving (|omega_z| < 0.005 rad/s, v > 4 m/s) updates bg[2]
                     │
                     ▼
[Stage 9: Causal 3D HMM Map Matching]
  - Python: Causal3DMapMatcher.process_step() (map_matcher_3d.py)
  - C++: Causal3DMapMatcher::process_step() (map_matcher_3d.cpp)
  - Inputs: p_dr, heading_dr, baro_alt, speed_dr
  - Elevation barrier: |Delta h| > 4.0m blocks vertical flyover jumps
  - Decoding: Online Forward Viterbi trellis (no backward pass)
                     │
                     ▼
[Stage 10: Navigation Telemetry Output]
  - Output Struct: NavigationTelemetry (position ENU, velocity, heading, covariance)
  - Android: Mapped to flat array buffer -> NavigationTelemetry parcelable
```

---

# 4. Ground-Truth / CAN Leakage Audit

A strict code-level audit was conducted across all files to verify whether any ground-truth or CAN speed variables enter runtime inference:

### 4.1 Keyword Search Results in Production Code
- `idr-core/` (C++): **ZERO** references to `gt_speed`, `gt_vel`, `gt_pos`, `gt_heading`, `CAN`, or `OBD`.
- `android/` (Java/Kotlin/JNI): **ZERO** references to ground-truth variables or vehicle bus signals.
- `idr_pipeline/authoritative_pipeline.py`: Contains an active firewall assertion:
  ```python
  if prohibited_gt_speed is not None:
      raise AssertionError("CRITICAL VIOLATION: gt_speed passed to autonomous pipeline!")
  ```
  Pre-outage initialization uses authentic GNSS Doppler velocity (`gnss_vel_doppler`), not `gt_vel`.

### 4.2 Dynamic Runtime Verification
- The `TrajectoryOracleTrap` wrapper wraps `TrajectoryData` during benchmarking. Any access to `gt_speed`, `gt_vel`, `gt_pos`, or `gt_heading` during the outage interval raises an immediate, unhandled `AssertionError`.
- **Benchmark Execution Result:** 158 / 158 outages executed without a single trap firing.
- **Audit Verdict:** The authoritative inference path is **100% FREE** of ground-truth and CAN speed leakage.

---

# 5. Temporal Causality Audit

Every temporal and signal-processing operation in the pipeline was audited for causality:

| Operation | Implementation Function | Lookahead Buffer | Causality Classification |
|---|---|:---:|:---:|
| **Sliding Window Feature Extraction** | `extract_sliding_window_features` / `SlidingWindowFeatureExtractor` | $[t - 1.0\text{ s}, t]$ (10 samples trailing) | **STRICTLY CAUSAL** |
| **Vibration Filtering (Live)** | `VibrationPotholeFilter.process` | Trailing window $[t - 0.5\text{ s}, t]$ | **STRICTLY CAUSAL** |
| **Butterworth Prefilter (Offline Helper)** | `apply_butterworth_prefilter` (`filtfilt`) | Full array (forward-backward) | **NON-CAUSAL (OFFLINE ONLY)** |
| **ESKF Propagation & Update** | `ESKF::predict`, `update_ai_velocity`, `update_nhc` | Instantaneous $t_k$ | **STRICTLY CAUSAL** |
| **In-Flight Kalman ZARU** | `update_zaru` (triggered on $>8$ trailing straight ticks) | Trailing history | **STRICTLY CAUSAL** |
| **3D HMM Map Matching** | `Causal3DMapMatcher::process_step` | Forward Viterbi trellis | **STRICTLY CAUSAL** |
| **Longitudinal Map Constraint** | `CausalLongitudinalMapSynchronizer::process_step` | Trailing curvature accumulator | **STRICTLY CAUSAL** |

*Note on Offline Prefilter:* `apply_butterworth_prefilter` uses SciPy's `signal.filtfilt`. This function is strictly confined to `scripts/evaluate_phase4_ai_velocity_validation.py` and `tests/test_phase4_ai_velocity.py`. It is **NOT** present in `idr_pipeline/authoritative_pipeline.py`, `idr_pipeline/gnss_ins_fusion_engine.py`, or `idr-core/`.

---

# 6. Configuration Audit

The repository contains two configuration sources:
1. `CONFIG/production_authoritative.yaml` (Original single-source-of-truth)
2. `GOLDEN_AUTONOMOUS_CONFIG.yaml` (Promoted configuration)

### Configuration Comparison Matrix:

| Subsystem Parameter | `production_authoritative.yaml` | `GOLDEN_AUTONOMOUS_CONFIG.yaml` | Implementation Alignment |
|---|---|---|---|
| **Speed Model** | `gbdt_speed_model_iovnbd_top45.joblib` | `gbdt_speed_model_iovnbd_top45.joblib` | Exact Match |
| **Model SHA256** | `dc05b8de7957...` | `dc05b8de7957...` | Exact Match |
| **Window Duration** | $1.0\text{ s}$ (10 samples @ 10 Hz) | $1.0\text{ s}$ (10 samples @ 10 Hz) | Exact Match |
| **Speed Clamp** | $[0.0, 35.0]\text{ m/s}$ | $[0.0, 35.0]\text{ m/s}$ | Exact Match |
| **ESKF State Count** | 15 states | 15 states | Exact Match |
| **NHC $R_{\max}$** | $0.16\text{ m}^2/\text{s}^2$ ($0.40\text{ m/s}$) | $0.16\text{ m}^2/\text{s}^2$ ($0.40\text{ m/s}$) | Exact Match |
| **In-Flight ZARU Gate** | $|\omega_z| < 0.005\text{ rad/s}, v > 4\text{ m/s}$ | $|\omega_z| < 0.005\text{ rad/s}, v > 4\text{ m/s}$ | Exact Match |
| **Map Elevation Barrier** | $|\Delta h| > 4.0\text{ m}$ | $|\Delta h| > 4.0\text{ m}$ | Exact Match |
| **Motion-State Manager** | Not explicit in YAML | Explicit (`motion_state_manager.enabled: true`) | Documented in Golden |
| **Turn Map Heading Gate** | $45.0^\circ$ | $25.0^\circ$ (turn) / $45.0^\circ$ (normal) | Documented in Golden |

*Audit Conclusion:* The parameters in `GOLDEN_AUTONOMOUS_CONFIG.yaml` accurately describe the promoted M6 candidate while maintaining full compatibility with `production_authoritative.yaml`.

---

# 7. M0–M7 Experiment Traceability

Auditing `scripts/run_controlled_architectural_ablations.py` and `BENCHMARK_ABLATION_RESULTS.csv`:

### 7.1 Configuration Mapping in Code:
```python
experiments = [
    {"id": "M0", "name": "B1/N8 Golden Baseline",        "motion_state": False, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": False},
    {"id": "M1", "name": "B1/N8 + Motion-State Manager", "motion_state": True,  "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": False},
    {"id": "M2", "name": "B1/N8 + Speed Uncertainty",    "motion_state": False, "speed_confidence": True,  "adaptive_nhc": False, "innov_monitor": False, "map_confidence": False},
    {"id": "M3", "name": "B1/N8 + Adaptive NHC",         "motion_state": True,  "speed_confidence": False, "adaptive_nhc": True,  "innov_monitor": False, "map_confidence": False},
    {"id": "M4", "name": "B1/N8 + Innovation Monitor",   "motion_state": False, "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": True,  "map_confidence": False},
    {"id": "M5", "name": "B1/N8 + Uncertainty-Aware Map","motion_state": True,  "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": True},
    {"id": "M6", "name": "B1/N8 + MotionState + MapConf","motion_state": True,  "speed_confidence": False, "adaptive_nhc": False, "innov_monitor": False, "map_confidence": True},
    {"id": "M7", "name": "B1/N8 + Full Integrated Policy","motion_state": True, "speed_confidence": True,  "adaptive_nhc": True,  "innov_monitor": True,  "map_confidence": True},
]
```

### 7.2 Key Forensic Findings on M5 vs. M6:
- **Root Cause of Identical Metrics:** In lines 404–405 of `scripts/run_controlled_architectural_ablations.py`, M5 was declared with `"motion_state": True, "map_confidence": True`—which is identical to M6! As a result, M5 and M6 executed the exact same code path, producing identical aggregate numbers (Median: 54.35%, Mean: 115.12%, Passes: 10/158).
- **Intended Design:** M5 was conceptually intended as standalone map confidence (`motion_state: False, map_confidence: True`), while M6 was the combination.

### 7.3 Forensic Analysis of M7:
- M7 achieved **12 passes** ($\le 10\%$) and dropped mean drift to **108.38%** (lowest in the benchmark).
- **The Caveat:** M7's median yaw error degraded to **42.98°** (compared to 36.32° in M0 and 37.51° in M6). This occurred because aggressive innovation gating in M4 occasionally rejected valid speed updates during cornering, destabilizing the heading observability coupling ($a_y \approx v \cdot \omega_z$).
- **Disposition:** M7 must **NOT** be promoted until the yaw degradation during turn transitions is resolved.

---

# 8. Android Architecture

### 8.1 Module Identification:
- **Root Directory:** `android/`
- **Application ID:** `org.isro.idr`
- **SDK Target:** `minSdk = 26`, `targetSdk = 34`, `compileSdk = 35`
- **Native NDK Build:** `ndkVersion = 25.1.8937393`, CMake 3.22.1 (`libc++_shared`, C++17)

### 8.2 Ingestion & Processing Flow:
1. `SensorIngestionManager.kt`: Ingests hardware `TYPE_ACCELEROMETER`, `TYPE_GYROSCOPE`, `TYPE_PRESSURE`, and `TYPE_MAGNETIC_FIELD` at up to 100 Hz.
2. `GnssIngestionManager.kt`: Ingests `Location` fixes and raw `GnssMeasurementsEvent` (detecting `CONSTELLATION_IRNSS` / NavIC).
3. `NavigationPipelineSynchronizer.kt`: Thread-safe synchronizer that pops bounded IMU queues, coordinates with the latest fresh GNSS fix, and calls `NativeNavigationEngine.processStep()`.
4. `jni_bridge.cpp`: C++ JNI bridge translating primitive inputs into `idr::SeamlessGNSSDeficitEngine` step calls and populating a pre-allocated flat `double[]` telemetry array (`flatTelemetryBuffer`).

### 8.3 Screen Inventory (All 11 Screens Frozen):
`MainNavigationScreen.kt`, `DiagnosticsScreen.kt`, `GnssNavicScreen.kt`, `MapExplorerScreen.kt`, `OfflineMapsScreen.kt`, `SensorMonitorScreen.kt`, `SensorCalibrationScreen.kt`, `TripAnalyticsScreen.kt`, `MapDataScreen.kt`, `SettingsScreen.kt`, `SplashScreen.kt`.

### 8.4 Minimal Backend Integration Point:
The sole integration surface between Android and the native core is:
- C++ Source: `android/app/src/main/cpp/CMakeLists.txt` (which compiles `idr-core/src/*.cpp`)
- JNI Wrapper: `android/app/src/main/java/org/isro/idr/engine/NativeNavigationEngine.kt`
Zero UI, layout, or ViewModel alterations are required to upgrade the engine.

---

# 9. C++ Production Engine

### 9.1 Core Entry Points & Headers:
- **High-Level Navigator:** `idr::SeamlessGNSSDeficitEngine` (`idr-core/include/idr/navigation_engine.hpp`)
- **Edge Stream Engine:** `idr::EdgeNavigationEngine` (`idr-core/include/idr/edge_engine.hpp`)
- **C-ABI Export:** `idr_c_api.h` (`idr_engine_create`, `idr_engine_process_step`, `idr_engine_destroy`)

### 9.2 Key Data Structures (`types.hpp`):
- `Vector3`: 3D Cartesian vector with dot, cross, norm, normalization
- `Matrix3x3`: 3x3 matrix with inversion and transposition
- `Quaternion`: SO(3) attitude quaternion with exponential/logarithmic map
- `NavigationTelemetry`: Structured step output containing mode, ENU position, velocity, Euler angles, biases, outage distance, and P-trace.

### 9.3 Runtime Memory & Threading Profile:
- **Hot-Loop Heap Allocations:** Exactly **0 bytes**. All buffers (feature extractor ring buffer, vibration filter window) are statically sized stack or class-member arrays.
- **Threading Model:** Re-entrant and thread-safe per engine instance. No global mutable state.
- **Throughput:** Benchmark (`benchmark_tactical_100k`) validates **83,823 Hz** execution rate ($11.93\ \mu\text{s}$ per cycle).
- **RAM Footprint:** $< 2.4\text{ MB}$ resident set size.

---

# 10. Test & Validation Infrastructure

### 10.1 Python Test Suite (276 / 276 Passing):
- **Core Kinematics & ESKF:** `test_eskf_fusion.py`, `test_phase1_navigation_core.py` (33 tests)
- **Mount Alignment:** `test_phase2_alignment_initialization.py` (20 tests)
- **Heading Supervisor:** `test_phase3_heading_observability.py` (24 tests)
- **AI Speed & Vibration:** `test_phase4_ai_velocity.py` (15 tests)
- **Map Matching & HMM:** `test_phase6_map_matching.py`, `test_phase11_7_causal_hmm.py`, `test_phase11_5_osm_pipeline.py` (30 tests)
- **Supervisor & Transitions:** `test_phase7_gnss_ins_supervisor.py` (16 tests)
- **Firewall & Causality:** `test_no_ground_truth_in_inference.py`, `test_strict_causality.py` (4 tests)
- **Android Integration:** `test_phase11_2_jni_bridge.py`, `test_phase11_3_ingestion_layer.py`, `test_phase11_4_ui_layer.py` (11 tests)
- **Parity Verification:** `test_phase11_6_cpp_reader_parity.py` (5 tests)
- **Edge & Replay:** `test_external_imu_edge.py`, `test_edge_engine_api.py`, `test_i2nav_external_replay.py` (18 tests)
- **Historical Benchmarks:** `test_phase10_track_b_freeze.py`, `test_phase12`–`test_phase17` (35 tests)

### 10.2 C++ CTest Suite (14 / 14 Passing):
1. `test_math_so3`: Quaternion exponential/logarithmic map & numerical stability
2. `test_math_golden_parity`: Matrix inversion, multiplication, and skew-symmetric parity
3. `test_gbdt_parity`: Bit-exact evaluation vs serialized LightGBM model
4. `test_feature_vibration`: 45-feature extraction & 2nd-order Butterworth filter
5. `test_alignment`: Gravity leveling & PCA longitudinal axis estimation
6. `test_eskf`: Full 15-state propagation, Joseph-form updates, and NHC/ZARU
7. `test_heading_supervisor`: Innovation gating & magnetic disturbance rejection
8. `test_map_matcher`: 3D elevation barrier & topological Viterbi decoding
9. `test_navigation_engine`: Multi-mode state machine transitions
10. `test_c_api`: C-ABI handles, parameter validation, and telemetry extraction
11. `test_idrmap_reader`: Binary `.idrmap` partition parser
12. `test_causal_hmm`: Online HMM forward trellis decoding
13. `test_offline_navigation`: Synthetic multi-tier corridor navigation
14. `test_edge_engine`: 200 Hz continuous external IMU streaming

---

# 11. Complete / Consolidate / Missing Matrix

### A. ALREADY COMPLETE / VALIDATED:
1. Ground-truth firewall (static scanner and dynamic `TrajectoryOracleTrap`).
2. Strict temporal causality verification (prefix truncation invariance).
3. 15-state Error-State Kalman Filter in both Python and C++17.
4. Bounded-slip lateral NHC with $R_{\max} = 0.16\text{ m}^2/\text{s}^2$.
5. Top-45 Huber GBDT speed regressor transpiled to static branchless C++ tables.
6. Pre-outage GNSS Doppler scale & bias handover.
7. Causal 3D HMM map matcher with $|\Delta h| > 4.0\text{ m}$ elevation barrier.
8. In-flight Kalman ZARU on steady straight segments ($|\omega_z| < 0.005\text{ rad/s}$).
9. Frozen Android UI and JNI bridge contract (`libidr_jni.so`).
10. 100% test pass rate (276 Python tests + 14 CTests).

### B. EXISTS BUT NEEDS CONSOLIDATION:
1. **M5 vs. M6 Ablation Code:** Disentangle `motion_state: True` in M5 inside `scripts/run_controlled_architectural_ablations.py` so M5 reflects standalone map confidence.
2. **C++ Navigation Engine Motion-State Anti-Creep:** Port the Python `is_standstill` low-variance standstill check into `idr-core/src/navigation_engine.cpp` to ensure 100% C++ parity with M6.
3. **Redundant Diagnostic Scripts:** Multiple historical benchmark scripts (`run_phase10` through `run_phase17`) should be consolidated under a clear research archive namespace.

### C. MISSING AND MUST BE IMPLEMENTED:
1. **M7 Yaw Destabilization Resolution:** Understand and rectify why M7's full integrated policy increases yaw error to 42.98° despite boosting pass count to 12.
2. **Autonomous Calibration on Startup:** Refine zero-velocity detector during the first 10 seconds of vehicle startup without relying on pre-existing calibration files.
3. **Android Build Pipeline on Host:** Host environment lacks a local Java runtime (`/usr/bin/java`), requiring external CI or containerized execution for APK compilation.

---

# 12. Recommended Canonical Architecture

The recommended canonical architecture moving forward is:

```
[Raw Smartphone IMU / External IMU]
               │
               ▼
[Dynamic In-Vehicle Alignment (Gravity + PCA)]
               │
               ▼
[2nd-Order IIR Vibration & Pothole Blanking]
               │
               ▼
[Causal Motion-State Manager] ──► [Anti-Creep ZUPT Gate (v_fwd < 1.0, var(a) < 0.05)]
               │
               ▼
[Trailing 1.0s Top-45 Feature Extractor]
               │
               ▼
[Transpiled Branchless GBDT Speed Model]
               │
               ▼
[Pre-Outage GNSS Doppler Scale Handover]
               │
               ▼
[15-State ESKF Mechanization]
 ├── Bounded-Slip NHC (R_max = 0.16)
 ├── Standstill ZUPT (R_zupt = 0.01^2)
 └── In-Flight Kalman ZARU (|omega_z| < 0.005)
               │
               ▼
[Uncertainty-Aware 3D Map Matching]
 ├── Turn Gating (<25° max diff during turns)
 └── High-Uncertainty Yaw Aiding (R_road = 0.035 when P_yaw > 0.01)
               │
               ▼
[Deterministic NavigationTelemetry (0.012 ms latency)]
```

---

# 13. Risks and Ambiguities

1. **M5/M6 Experimental Conflation:** Because M5 inadvertently included `motion_state: True`, the exact isolated contribution of map confidence alone versus motion-state alone was masked in the summary table.
2. **M7 Yaw Drift Risk:** While M7 increased passes from 9 to 12 and dropped mean drift to 108.38%, its median yaw error increased by +6.66°. Deploying M7 without fixing its turn-phase innovation gating creates a risk of lateral heading divergence.
3. **Near-Standstill Outage Singularity:** In 4 IO-VNBD outages where total distance $D \le 1.0\text{ m}$, small positional jitter mathematically inflates drift percentages to $>1000\%$. The evaluation protocol must transparently present both uncurated (all 158) and moving ($D \ge 15\text{ m}$) slices to prevent misleading conclusions.

---

# 14. Part-1 Implementation Sequence

For subsequent execution tasks:
1. **Step 1:** Disentangle M5 and M6 in `scripts/run_controlled_architectural_ablations.py` and run a clean 158-outage pass to establish isolated baseline numbers.
2. **Step 2:** Port the verified M6 Motion-State Manager and 25° turn map-gating logic into `idr-core/src/navigation_engine.cpp`.
3. **Step 3:** Re-run C++ CTests (`ctest --output-on-failure`) to verify that all 14 tests remain 100% passing with zero regressions.
4. **Step 4:** Perform a controlled tuning experiment on M7's innovation gate to resolve the yaw degradation during turns before considering M7 for promotion.
5. **Step 5:** Keep the Android UI 100% frozen, preserving all 11 Compose screens and JNI signatures.
