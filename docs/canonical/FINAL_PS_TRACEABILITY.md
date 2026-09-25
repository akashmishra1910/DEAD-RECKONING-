# FINAL PS 26168 TRACEABILITY MATRIX
## Strict Requirements-to-Implementation Mapping & Verification Audit

**Document ID:** `DOC-CANONICAL-PS-TRACE-02`  
**Version:** 2.0.0 (Submission Freeze)  
**Date:** September 26, 2026  
**Status Standard:** Rule Zero — Only [VERIFIED, PARTIALLY VERIFIED, NOT VERIFIED, PENDING]  

---

## 1. Traceability Table

| PS Requirement ID | Mandated Problem Statement Requirement | Architectural Implementation | Primary Source Code | Verification Test / Harness | Quantitative Evidence Artifact | Final Verification Status |
|---|---|---|---|---|---|---|
| **PS-REQ-01** | AI/ML-based forward vehicle speed estimation without OBD-II/CAN-bus access | 120-tree Huber GBDT regressor trained on 1.0s vibration/kinematics windows, transpiled to static branchless C++ decision tables | `idr-core/src/gbdt_speed.cpp`, `idr_pipeline/gbdt_speed.py`, `models/production/gbdt_speed_model_iovnbd_top45.joblib` | `tests/test_gbdt_speed.py`, `tests/test_parity.py` | `reports/phase9_python_cpp_equivalence.csv` (Bit-for-bit parity, $R^2 = 0.884$, RMSE $1.21\text{ m/s}$) | **VERIFIED** |
| **PS-REQ-02** | 15-State Error-State Kalman Filter ($p, v, q, b_a, b_g$) | Continuous-discrete error-state formulation with quaternion attitude kinematics, gyro/accel bias tracking, and covariance propagation | `idr-core/src/eskf.cpp`, `idr_pipeline/eskf.py` | `tests/test_eskf.py`, `tests/test_covariance.py` | `reports/phase10_track_b_freeze_results.json` | **VERIFIED** |
| **PS-REQ-03** | Non-Holonomic Constraints (NHC) | Vehicle body-frame zero lateral and vertical velocity suppression ($v_y^b \approx 0, v_z^b \approx 0$) | `idr-core/src/eskf.cpp` | `tests/test_nhc.py` | `reports/phase10c_adaptive_nhc.csv` | **VERIFIED** |
| **PS-REQ-04** | Standstill Detection & Zero-Velocity Updates (ZUPT) | Acceleration and angular-rate energy variance threshold detector with automatic velocity clamping | `idr-core/src/eskf.cpp` | `tests/test_zupt.py` | `reports/zupt_ablation_experiment.json` (Drift reduced from $4.61\%$ to $2.58\%$ in stop-and-go) | **VERIFIED** |
| **PS-REQ-05** | Portable C++ Core Engine decoupled from Android | Self-contained C++17 library with CMake build, zero external ML runtimes (no PyTorch, no TFLite, no ONNX) | `idr-core/CMakeLists.txt`, `idr-core/include/idr/` | `tests/test_cpp_engine.py`, CTest suite (10/10 passing) | `idr-core/build/` compilation artifacts, Valgrind memory report ($<2.4\text{ MB}$ RSS) | **VERIFIED** |
| **PS-REQ-06** | Edge Engine capable of running with external IMU at ~200 Hz | Standalone CLI binary accepting CSV/binary streams, sample-rate agnostic $\Delta t$ integration | `idr-cli/src/main.cpp`, `idr-core/src/sensor_adapter.cpp` | `tests/test_cli_runner.py` | `reports/idr_cli_output.csv` ($56,026\text{ Hz}$ benchmark throughput, $17.85\ \mu\text{s}$ latency) | **VERIFIED** |
| **PS-REQ-07** | 10 Hz Android Mobile Application | 12-Screen Jetpack Compose app with JNI bindings, thread-safe bounded circular FIFO buffering | `android/app/src/main/java/org/isro/idr/` | Android JUnit tests (`./gradlew testDebugUnitTest`), UI tests | `android/app/build/` test execution logs | **VERIFIED** |
| **PS-REQ-08** | Offline 3D Multi-Layer Map Matching (Flyovers, Bridges, Tunnels) | Causal 3D HMM Viterbi trellis decoding with $|\Delta h| > 4.0\text{ m}$ elevation barrier | `idr-core/src/map_matcher.cpp`, `idr_map/` | `tests/test_map_matcher.py`, `tests/test_elevation_barrier.py` | `reports/map_matching_3d_evaluation.json` ($99.7\%-100.0\%$ layer accuracy) | **VERIFIED** |
| **PS-REQ-09** | GNSS Deficit & Multipath Supervisor | Chi-Square innovation gate ($\gamma \le 11.35$) rejecting multipath and managing GNSS $\leftrightarrow$ DR transitions | `idr-core/src/supervisor.cpp` | `tests/test_supervisor.py` | `reports/phase7_map_closed_loop_diagnostics.csv` | **VERIFIED** |
| **PS-REQ-10** | ISRO NavIC / IRNSS Constellation Monitoring | Android `GnssStatus.Callback` reading `CONSTELLATION_IRNSS` and live polar skyview radar | `android/app/.../GnssIngestionManager.kt`, `android/app/.../GnssNavicScreen.kt` | `android/app/.../GnssIngestionManagerTest.kt` | Physical Android device telemetry logs | **VERIFIED** |
| **PS-REQ-11** | Magnetometer Yaw Aiding with Magnetic Disturbance Rejection | Dynamic magnetic field norm check $|\|\mathbf{m}\| - \|\mathbf{m}_0\|| < \epsilon$ suppressing steel vehicle distortion | `idr-core/src/eskf.cpp`, `idr_pipeline/heading.py` | `tests/test_heading.py` | `reports/phase5_heading_error_budget_replay.csv` | **VERIFIED** |
| **PS-REQ-12** | Mandatory Evaluation on Official IO-VNBD Dataset | Complete 158-outage benchmark evaluated across authentic IO-VNBD driving logs | `idr_pipeline/outage_evaluator.py` | `tests/test_outage_benchmark.py` | `reports/final_track_b_benchmark.csv`, `reports/final_track_b_outage_results.csv` | **VERIFIED** |
| **PS-REQ-13** | Two-Wheeler Lean-Compensated Constraint | Roll-compensated NHC mode dynamically scaling vertical constraint variance during banking | `idr-core/src/eskf.cpp` | `tests/test_nhc.py` | Algorithm unit test passing | **VERIFIED** |
| **PS-REQ-14** | Dataset-Wide PS <10% Drift Bound across all 158 Outages | Outage-stratified evaluation: Short/Medium outages pass ($1.62\%-2.88\%$), but full dataset median is $35.71\%$ in C++ and $27.25\%$ in Python research | `idr_pipeline/outage_evaluator.py` | `tests/test_outage_benchmark.py` | `reports/final_track_b_benchmark.csv` (29/158 passes in N7 C++, 49/158 in P17 research) | **PARTIALLY VERIFIED (GAP DISCLOSED)** |
| **PS-REQ-15** | Physical FOG Hardware Interfacing | Physical fiber-optic gyro serial stream ingestion on embedded Linux target | `idr-cli/src/main.cpp` | Software replay test | Validated on high-rate software streams and I2NAV logs; physical FOG hardware connection pending | **PENDING** |

---

## 2. Summary Status Distribution

- **VERIFIED:** 13 Requirements (86.7%)
- **PARTIALLY VERIFIED (GAP DISCLOSED):** 1 Requirement (6.7% — Dataset-wide $<10\%$ drift across full 158-outage population)
- **PENDING:** 1 Requirement (6.7% — Physical FOG hardware bench validation)
- **NOT VERIFIED:** 0 Requirements (0.0%)
