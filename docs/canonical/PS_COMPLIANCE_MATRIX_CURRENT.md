# PS 26168 COMPLIANCE MATRIX — CURRENT & AUTHORITATIVE
## Evaluation Against Mandated ISRO Problem Statement Criteria

**Document ID:** `DOC-CANONICAL-COMPLIANCE-07`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Compliance Checklist

| Item | Problem Statement Mandate | Implemented Capability | Evidence / Source Code | Compliance Status |
|---|---|---|---|---|
| **PS-01** | AI/ML Based Speed Estimation from IMU without OBD-II/CAN | Transpiled GBDT (120 trees, depth 5, $\eta=0.08$) regressing forward speed from 1.0s vibration/kinematics windows. | `idr-core/src/gbdt_speed.cpp`, `idr_pipeline/gbdt_speed.py`, `models/production/` | **COMPLIANT (100%)** |
| **PS-02** | 15-State ESKF Formulation ($p, v, q, b_a, b_g$) | Continuous-discrete error-state Kalman filter with quaternion kinematics, bias estimation, and covariance propagation. | `idr-core/src/eskf.cpp`, `idr_pipeline/eskf.py` | **COMPLIANT (100%)** |
| **PS-03** | Non-Holonomic Constraints (NHC) | Lateral ($v_y^b \approx 0$) and vertical ($v_z^b \approx 0$) velocity updates in vehicle body frame. | `idr-core/src/eskf.cpp` | **COMPLIANT (100%)** |
| **PS-04** | Standstill Detection & Zero-Velocity Updates (ZUPT) | Dual accelerometer & gyroscope energy variance detector triggering velocity reset during traffic stops. | `idr-core/src/eskf.cpp` | **COMPLIANT (100%)** |
| **PS-05** | Portable C++ Core decoupled from Android | `idr-core` is pure C++17 with CMake, zero external ML dependencies (no PyTorch, no TFLite, no ONNX). | `idr-core/CMakeLists.txt`, `idr-core/include/idr/` | **COMPLIANT (100%)** |
| **PS-06** | Edge Engine running at ~200 Hz with external IMU data | `idr-cli` runs at $>250\times$ real-time rate on external IMU streams, supporting arbitrary sampling rates ($\Delta t$ passed per sample). | `idr-cli/src/main.cpp`, `idr-core/src/sensor_adapter.cpp` | **COMPLIANT (100%)** |
| **PS-07** | 10 Hz Android Mobile Application | 12-Screen Jetpack Compose application with JNI bridge to `idr-core`, bounded circular FIFO buffering, and smooth tracking. | `android/app/src/main/java/org/isro/idr/` | **COMPLIANT (100%)** |
| **PS-08** | Offline Multi-Layer Map Matching (Flyovers, Tunnels, Ground) | Causal 3D HMM with elevation barrier $|\Delta h| > 4.0\text{ m}$ blocking impossible flyover hops. | `idr-core/src/map_matcher.cpp`, `idr_map/` | **COMPLIANT (100%)** |
| **PS-09** | GNSS Multipath & Deficit Supervision | Chi-Square $\chi^2 \le 11.35$ innovation gating rejecting tunnel-exit multipath spikes. | `idr-core/src/supervisor.cpp` | **COMPLIANT (100%)** |
| **PS-10** | ISRO NavIC / IRNSS Constellation Support | Android `GnssStatus.Callback` reading `CONSTELLATION_IRNSS`, tracking NavIC PRNs, and displaying live radar skyview. | `android/app/.../GnssIngestionManager.kt`, `android/app/.../GnssNavicScreen.kt` | **COMPLIANT (100%)** |
| **PS-11** | Magnetometer Yaw Aiding with Disturbance Check | Dynamic magnetic disturbance detection using field magnitude residual $|\|\mathbf{m}\| - \|\mathbf{m}_0\|| < \epsilon$. | `idr-core/src/eskf.cpp`, `idr_pipeline/heading.py` | **COMPLIANT (100%)** |
| **PS-12** | Mandatory IO-VNBD Dataset Training & Evaluation | Evaluated across all 158 outages in real vehicle IO-VNBD recordings. | `docs/canonical/AUTHORITATIVE_BENCHMARK_REGISTER.md`, `tests/` | **COMPLIANT (100%)** |
| **PS-13** | Two-Wheeler Lean-Compensated Constraint | Roll-compensated NHC mode adjusting vertical constraint variance during banking. | `idr-core/src/eskf.cpp` | **COMPLIANT (100%)** |
| **PS-14** | Dataset-Wide <10% Bound Across All 158 Outages | 1.62%–2.88% on short/medium/highway runs; overall dataset median is 35.71% in C++ and 27.25% in Python research. | `docs/canonical/AUTHORITATIVE_BENCHMARK_REGISTER.md` | **PARTIAL / GAP DOCUMENTED** |
