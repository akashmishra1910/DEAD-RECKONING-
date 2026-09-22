# Requirements Traceability Matrix (RTM)
## ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Source of Truth Hierarchy:**
1. Official SIH / PS 26168 Statement (Documented in repo root and `Problem Statement 26168.txt`).
2. Existing Project Requirements / Architecture Specifications (`MASTER_RESEARCH_SYSTEM_SPECIFICATION.md`, `AGENTS.md`).
3. Validated Repository Implementation & Benchmark Reports (`ESKF_STEP4C_IOVNBD_BENCHMARK_REPORT.md`).

**Permitted Status Values:**
`NOT STARTED` | `IN PROGRESS` | `IMPLEMENTED` | `UNIT TESTED` | `INTEGRATION TESTED` | `BENCHMARKED` | `VERIFIED` | `BLOCKED` | `NOT APPLICABLE`

---

## 1. Complete Traceability Table

| ID | Source | Requirement Description | Priority | Implementation Component | Verification Test | Benchmark Metric | Status | Remaining Work |
|---|---|---|:---:|---|---|---|:---:|---|
| **REQ-01** | REQUIRED BY PS | Standalone Smartphone Inertial Dead Reckoning (No OBD-II / CAN) | P0 | `idr_pipeline/eskf_fusion.py`, `ai_velocity_model.py` | `test_suite.py::Test 1,5`, `tests/test_phase3_heading_observability.py` | Position Drift % during GNSS outage | **BENCHMARKED** | Integrated multi-source heading supervisor achieving 5.68% drift on authentic drive `Vta01b`. |
| **REQ-02** | REQUIRED BY PS | < 10% Positional Drift over Total Distance during GNSS Blackout | P0 | `idr_pipeline/eskf_fusion.py`, `heading_supervisor.py` | `tests/test_phase3_heading_observability.py::test_23_authentic_iovnbd_heading_aiding_outage` | Drift % on 10s, 30s, 60s, 120s outages | **VERIFIED** | Verified 5.68% drift over 473.9m outage on authentic IO-VNBD drive `Vta01b` (<10% PS benchmark met). |
| **REQ-03** | REQUIRED BY PS | Short Outage: < 5 m drift over 50 m in < 1 min | P0 | `idr_pipeline/eskf_fusion.py` | `scripts/evaluate_isro_benchmarks.py` | Horizontal distance error (m) | **BENCHMARKED** | Baseline 22.25m on `Vta01b`; heading observability reduces initial segment drift. |
| **REQ-04** | REQUIRED BY PS | Extended Highway Outage: < 100 m drift over 1 km @ 60 km/h | P0 | `idr_pipeline/eskf_fusion.py` | `scripts/evaluate_isro_benchmarks.py` | Horizontal distance error (m) | **BENCHMARKED** | Measured 334.36 m on 700m corridor; requires road snapping & heading fusion. |
| **REQ-05** | REQUIRED BY PS | Instant Seamless Transition between GNSS & DR (< 5 ms) | P0 | `idr_pipeline/gnss_ins_fusion_engine.py` | `test_suite.py::Test 3` | Handover latency (ms) & position discontinuity | **INTEGRATION TESTED** | Benchmark transition latency and covariance continuity on real logs. |
| **REQ-06** | REQUIRED BY PS | Chi-Square Innovation Gate for Multipath Rejection ($\gamma \le 11.35$) | P0 | `idr_pipeline/gnss_ins_fusion_engine.py` | `test_suite.py::Test 3` | False-positive multipath rejection rate | **INTEGRATION TESTED** | Test with real tunnel exit multipath step responses. |
| **REQ-07** | REQUIRED BY PS | Mobile Application 10 Hz Real-Time On-Device Update Rate | P0 | `idr-android/`, `web_app/` | `test_suite.py::Test 5` | Cycle latency ($\le 100\text{ ms}$) | **IN PROGRESS** | Port `idr-core` to Kotlin + NDK JNI Android app. |
| **REQ-08** | REQUIRED BY PS | Edge-Deployable Software Engine (~200 Hz with FOG IMUs) | P0 | `idr-core/`, `idr_pipeline/edge_engine.py` | `test_suite.py::Test 5` | Achieved throughput (Hz) & cycle time ($\mu\text{s}$) | **BENCHMARKED** | Measured 1817.4 Hz throughput in Python; build native C++17 library. |
| **REQ-09** | REQUIRED BY PS | Automatic In-Vehicle Alignment (Pitch, Roll, Yaw relative to body) | P0 | `idr_pipeline/alignment_engine.py` | `test_suite.py::Test 1`, `tests/test_phase2_alignment_initialization.py` | Alignment error ($<2^\circ$), det($R$) = +1.0 | **VERIFIED** | Multi-mode alignment handling initial stillness and dynamic motion verified. |
| **REQ-10** | REQUIRED BY PS | Cradle Slip Detection & Rapid Recalibration (< 4 s) | P1 | `idr_pipeline/alignment_engine.py` | `test_suite.py::Test 1`, `tests/test_phase2_alignment_initialization.py` | Recalibration latency (s) | **VERIFIED** | Verified $<0.1\text{ s}$ bump detection and rapid recalibration. |
| **REQ-11** | REQUIRED BY PS | AI Speed & Vibration/Pothole Filter (Direct IMU speed inference) | P0 | `idr_pipeline/vibration_filter.py`, `models/gbdt_speed_model_iovnbd_top45.joblib` | `test_suite.py::Test 2, 2b` | Speed RMSE (m/s), shock rejection $>2.5g$ | **BENCHMARKED** | Validated on Top-45 GBDT (RMSE: 5.00 m/s overall, 4.89 m/s 0–60 km/h). |
| **REQ-12** | REQUIRED BY PS | AI-Based GNSS+INS Fusion Model | P0 | `idr_pipeline/eskf_fusion.py` | `tests/test_eskf_fusion.py` | Filter estimation error reduction | **IN PROGRESS** | Implement learned adaptive measurement noise $R_v(t)$ (Phase 5). |
| **REQ-13** | REQUIRED BY PS | Advanced Map-Matching with Kinematic Constraints (NHC) | P0 | `idr_pipeline/map_matcher_3d.py`, `eskf_fusion.py` | `tests/test_phase1_navigation_core.py::test_14`, `test_suite.py::Test 4` | NHC velocity bounds ($v_y \approx 0, v_z \approx 0$) | **VERIFIED** | Extract authentic OSM road graph for IO-VNBD real drive routes. |
| **REQ-14** | REQUIRED BY PS | 3D Layer Disambiguation (Flyover vs Ground Road, $\|Δh\| > 4\text{ m}$) | P1 | `idr_pipeline/map_matcher_3d.py` | `test_suite.py::Test 4` | Flyover layer accuracy ($>99\%$) | **INTEGRATION TESTED** | Validate with real barometric elevation logs on multi-level bridges. |
| **REQ-15** | REQUIRED BY PS | Mandated IO-VNBD Dataset Training & Verification | P0 | `idr_pipeline/real_iovnbd_loader.py` | `scripts/run_step4c_iovnbd_benchmark.py` | 72 naturalistic drives, 5 held-out test drives | **BENCHMARKED** | Executed 935 evaluation records across 4 outage durations on real logs. |
| **REQ-16** | REQUIRED BY PS | Live Magnetometer / Compass Ingestion with Disturbance Rejection | P1 | `idr_pipeline/heading_supervisor.py`, `eskf_fusion.py` | `tests/test_phase3_heading_observability.py` | Heading convergence & anomaly gating | **VERIFIED** | 3D tilt-compensated leveling, multi-tier anomaly gating ($|\Delta B| > 12\ \mu\text{T}$), $\chi^2$ gating, and 2-epoch re-acquisition verified. |
| **REQ-17** | REQUIRED BY PS | Two-Wheeler Vehicle Mode (Motorcycles/Scooters Lean-Compensated) | P2 | `idr_pipeline/eskf_fusion.py` | *Pending Phase 8* | Roll-compensated lateral/vertical NHC | **NOT STARTED** | Implement lean angle dynamics model. |
| **REQ-18** | REQUIRED BY PS | NavIC (IRNSS) Constellation Detection & Tracking | P1 | `idr-android/` | *Pending Phase 12* | Satellite count and fix display | **NOT STARTED** | Android `GnssMeasurement` (`CONSTELLATION_IRNSS`) integration. |
| **REQ-19** | REQUIRED BY PS | Fully Offline Operation (Zero Network Calls at Runtime) | P0 | System-wide architecture | *Pending Phase 13* | Airplane mode execution | **IN PROGRESS** | Ensure all models and map assets are local. |
| **REQ-20** | REQUIRED BY DESIGN | Zero External ML Runtime (Transpiled C++ Decision Trees) | P0 | `idr-core/` | `test_suite.py::Test 5` | RAM $<2.5\text{ MB}$, zero PyTorch/TFLite | **IN PROGRESS** | Transpile Top45 GBDT to static C++ tables in Phase 10. |
| **REQ-21** | REQUIRED BY DESIGN | Unified FLU Body & ENU Navigation Frames ($\det(R) = +1.0$) | P0 | `idr_pipeline/eskf_fusion.py`, `alignment_engine.py` | `tests/test_phase1_navigation_core.py::test_01-05` | Orthogonality error $<10^{-14}$, $\det(R)=1.0$ | **VERIFIED** | Enforced in Step 4B-C; strictly verified across all unit tests. |
| **REQ-22** | REQUIRED BY PS | Real-Time Navigation Interface with Replay Mode | P1 | `web_app/`, `idr-android/` | *Pending Phase 14* | Smooth vehicle tracking, GPS vs IDR toggle | **IN PROGRESS** | WebSocket cockpit working; build native Android UI & replay module. |
