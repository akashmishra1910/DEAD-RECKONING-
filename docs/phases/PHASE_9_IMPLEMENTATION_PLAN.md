# Phase 9 Implementation Plan: Portable C++17 Edge Core & Production Engine (`idr-core`)
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Target Deliverable:** Production-grade, zero-dependency, portable C++17 navigation core library (`idr-core`) and standalone edge CLI (`idr-cli`).  
**Baseline Verification Status:** Phase 8 Completed and Locked (155/155 Regression Tests PASSING).  
**Two-Wheeler Support:** OUT OF SCOPE (As explicitly directed by governing authorization).  
**Reference Implementation:** Python pipeline in `idr_pipeline/`.  

---

## 1. Executive Summary & Forensics Audit

### 1.1 Repository Forensics Summary
A complete audit of the repository, reports, decisions, and codebase establishes:
1. **Mathematical Invariants (Phase 1):** Strict right-handed FLU body frame (+X Forward, +Y Left, +Z Up) and ENU navigation frame (+X East, +Y North, +Z Up), with $\mathbf{R}_b^n \in \text{SO}(3), \det(\mathbf{R}) = +1.0$, Rodrigues rotation vector integration, and SVD/Gram-Schmidt re-orthogonalization.
2. **Mounting & Alignment (Phase 2):** Gravity leveling for roll/pitch, stationary gyro bias initialization, dual-threshold surge PCA ($\kappa = \lambda_{\text{long}}/\lambda_{\text{trans}} \ge 2.0, a_{\text{surge}} \ge 0.35\text{ m/s}^2$), and cradle slip watchdog with rapid recalibration.
3. **Vibration Pre-Filtering (Phase 1 & 2):** 4th-order zero-phase Butterworth bandpass filter (0.1–15 Hz), impulse shock blanking ($>2.5g$) with rolling median replacement, and engine idle harmonic detection ($15–30\text{ Hz}, \sigma^2 \in [0.02, 0.25]$).
4. **Frozen AI Velocity Model (Phase 4):** Frozen Top-45 Huber GBDT model (`models/gbdt_speed_model_iovnbd_top45.joblib`) with 120 trees, max depth 5, learning rate $\eta=0.08$, initial prediction constant $+11.0894\text{ m/s}$, trained with standard scaler over 45 selected features from sliding windows $[t-1.0\text{ s}, t]$.
5. **15-State ESKF (Phase 1, 5, 6, 7):** States $\delta\mathbf{x} = [\delta\mathbf{p}^n, \delta\mathbf{v}^n, \delta\boldsymbol{\theta}^n, \delta\mathbf{b}_a^b, \delta\mathbf{b}_g^b]^T \in \mathbb{R}^{15}$. Joseph-form covariance update $(\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$, higher-order error-state reset $\mathbf{G}_{\text{reset}} = \mathbf{I} - 0.5[\delta\boldsymbol{\theta}^n]_\times$, adaptive AI velocity noise $R_v(t) = 0.35(1.0 + 0.5\cdot\text{Vib} + 1.2\cdot|\omega_z|)$, Non-Holonomic Constraints (NHC), Zero Velocity Updates (ZUPT), barometric altitude, GNSS position/velocity, heading updates, and soft road constraints.
6. **Heading Supervisor (Phase 3):** Dynamic velocity-weighted GNSS COG ($\sigma_\psi = \text{clip}(\sigma_v/v_h, 1^\circ, 15^\circ)$ for $v_h \ge 2.5\text{ m/s}$), 3D tilt-compensated magnetometer leveling in FLU body frame, multi-tier disturbance rejection ($|\Delta B| > 12\ \mu\text{T}$, gradient $> 35\ \mu\text{T/s}$), $\chi^2$ innovation gating ($\gamma \le 6.635$), and 2-epoch reacquisition confirmation.
7. **Offline 3D OSM Graph & Map Matching (Phase 6):** Sub-millimeter Bowring WGS-84 $\leftrightarrow$ ENU conversions, topological directed graph, 2D spatial grid partitioning, $4\text{ m}$ elevation barrier ($\log P_{\text{alt}} \le -50.0$), causal online HMM with Viterbi transitions, soft road heading and cross-track normal updates.
8. **Phase 8 Hardening Components:**
   - `OnlineSpeedScaleBiasCalibrator`: RLS estimator ($v_{\text{corr}} = \alpha \cdot v_{\text{ai}} + \beta$) trained during `GNSS_AIDED` motion, frozen upon outage onset (`mode != GNSS_AIDED`).
   - `CausalLongitudinalMapSynchronizer`: Turn curvature synchronization at road vertices when $|\omega_z| > 0.05\text{ rad/s}$, gated with $\chi^2 \le 6.635$.
9. **Full 6-State FSM Supervisor (Phase 7):** `INITIALIZING`, `GNSS_AIDED`, `GNSS_DEGRADED`, `TRANSITION_TO_DR`, `DR_ONLY`, `TRANSITION_TO_GNSS`, `GNSS_REACQUIRED`, seamless handover ($\Delta p < 0.05\text{ m}$), $\chi^2$ gates for position ($\le 11.35$) and velocity ($\le 9.0$).

---

## 2. Target C++17 Core Architecture (`idr-core`)

```
idr-core/
├── CMakeLists.txt
├── include/
│   └── idr/
│       ├── types.hpp                  # Vector3, Matrix3x3, Matrix15x15, Telemetry, Mode enums
│       ├── math_utils.hpp             # SO(3), Rodrigues, skew-symmetric, Euler, wrapping, SVD
│       ├── alignment_engine.hpp       # In-vehicle leveling, PCA surge alignment, cradle watchdog
│       ├── vibration_filter.hpp       # Butterworth prefilter, shock blanking, engine idle
│       ├── gbdt_model.hpp             # Transpiled static branchless Top-45 decision trees
│       ├── feature_extractor.hpp      # Causal sliding-window feature extractor (45 features)
│       ├── eskf.hpp                   # 15-state ESKF, Joseph-form, reset, all Jacobians
│       ├── heading_supervisor.hpp     # GNSS COG, tilt-compensated mag, anomaly gate
│       ├── osm_graph.hpp              # WGS-84/ENU, nodes, edges, polylines, spatial grid
│       ├── map_matcher_3d.hpp         # 3D HMM, elevation barrier, soft road constraints
│       ├── speed_calibrator.hpp       # Online RLS scale/bias calibrator with outage freeze
│       ├── longitudinal_sync.hpp      # Curvature/turn along-track synchronizer
│       ├── navigation_engine.hpp      # 6-state FSM supervisor & end-to-end pipeline
│       └── c_api.h                    # Clean C API for JNI / Android and external FFI
├── src/
│   ├── math_utils.cpp
│   ├── alignment_engine.cpp
│   ├── vibration_filter.cpp
│   ├── gbdt_model.cpp
│   ├── feature_extractor.cpp
│   ├── eskf.cpp
│   ├── heading_supervisor.cpp
│   ├── osm_graph.cpp
│   ├── map_matcher_3d.cpp
│   ├── speed_calibrator.cpp
│   ├── longitudinal_sync.cpp
│   ├── navigation_engine.cpp
│   └── c_api.cpp
├── tests/
│   ├── CMakeLists.txt
│   ├── test_math_so3.cpp
│   ├── test_alignment.cpp
│   ├── test_vibration.cpp
│   ├── test_gbdt_parity.cpp
│   ├── test_eskf.cpp
│   ├── test_heading_supervisor.cpp
│   ├── test_osm_map_matcher.cpp
│   ├── test_speed_calibrator.cpp
│   └── test_full_engine.cpp
├── examples/
│   ├── run_synthetic_corridor.cpp
│   └── run_iovnbd_stream.cpp
└── idr-cli/
    ├── CMakeLists.txt
    └── main.cpp                       # Standalone edge CLI executable
```

### 2.1 Architectural Constraints
- **Language:** Standard C++17.
- **Dependencies:** Self-contained standard library only (`<cmath>`, `<array>`, `<vector>`, `<algorithm>`, `<string>`, `<chrono>`). Zero third-party dependencies required for core build. Linear algebra uses small fixed-size static structures (Stack-allocated 3x3, 15x15 arrays) to avoid dynamic heap allocation in the high-rate processing loop.
- **Portability:** Builds seamlessly with Clang (macOS), GCC (Linux), and Android NDK (Clang toolchain).
- **Sample Rate Agnostic:** Explicit `dt` per step, supporting 10 Hz mobile to $\ge 200\text{ Hz}$ tactical FOG streams.

---

## 3. Detailed Component Porting Specifications

### 3.1 Mathematics & Geometry (`idr/math_utils.hpp`)
- `Vector3`, `Matrix3x3`, `Matrix15x15` fixed-size stack structs.
- `skew_symmetric(v)`: $[v]_\times$.
- `rodrigues(omega)`: $\mathbf{R} = \mathbf{I} + \frac{\sin\theta}{\theta}[\mathbf{u}]_\times + \frac{1-\cos\theta}{\theta^2}[\mathbf{u}]_\times^2$.
- `euler_to_rotation_matrix(roll, pitch, yaw)`: Clockwise yaw from North in ENU.
- `rotation_matrix_to_euler(R)`: Exact match with Python reference.
- `renormalize_so3(R)`: Iterative polar decomposition / SVD projection to $\text{SO}(3)$ guaranteeing $\det(\mathbf{R}) = +1.0$ and orthonormality $<10^{-14}$.
- `wrap_pi(angle)`: Radians wrapped to $[-\pi, +\pi]$.

### 3.2 Transpiled GBDT Speed Model (`idr/gbdt_model.hpp`, `gbdt_model.cpp`)
- Transpile the 120 decision trees of `models/gbdt_speed_model_iovnbd_top45.joblib` directly to static C++ structures.
- Scaler parameters: `mean[45]` and `scale[45]` embedded as `constexpr double`.
- Tree evaluation: Flattened array representation with branchless/unrolled traversal or inline tree evaluations.
- Initial prediction: `+11.089444444444444`.
- Output clamping: `std::clamp(v, 0.0, 35.0)`.

### 3.3 Sliding Window Feature Extractor (`idr/feature_extractor.hpp`)
- Circular buffer of size 10 (1.0 s at 10 Hz).
- Computes exact Top-45 features: moments (mean, std, rms, ptp), dynamic gravity leveling, mount-invariant yaw/tilt rates, roughness, dynamic acceleration norm, normalized vibration energy, jerk/rate stats.

### 3.4 15-State ESKF (`idr/eskf.hpp`)
- State vector: $\mathbf{p}, \mathbf{v}, \mathbf{R}, \mathbf{b}_a, \mathbf{b}_g$.
- Covariance: $15 \times 15$ symmetric matrix $\mathbf{P}$.
- Complete update suite:
  - `update_ai_velocity(v_ai, r_ai)`
  - `update_nhc(r_lat, r_vert)`
  - `update_zupt(r_zupt)`
  - `update_altitude(baro_alt, r_alt)`
  - `update_gnss(gnss_pos, r_gnss)`
  - `update_gnss_velocity(gnss_vel, r_vel)`
  - `update_heading(heading_rad, r_yaw, chi2_gate)`
  - `update_road_heading(road_azimuth, r_road_yaw, chi2_gate)`
  - `update_road_position(road_snapped_pos, road_tangent_2d, r_cross, chi2_gate)`
- Joseph-form update: $\mathbf{P} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$.
- Higher-order error-state attitude reset: $\mathbf{G}_{\text{reset}} = \mathbf{I} - 0.5[\delta\boldsymbol{\theta}^n]_\times$.

### 3.5 In-Vehicle Alignment (`idr/alignment_engine.hpp`)
- Multi-mode state machine: `INITIALIZING`, `STATIONARY_LEVELED`, `QUASI_STATIC`, `DYNAMIC_ALIGNED`, `RECALIBRATING`.
- Gravity leveling from stationary acceleration buffer.
- PCA forward surge detection ($\kappa \ge 2.0, a_{\text{surge}} \ge 0.35\text{ m/s}^2$).
- Cradle slip detector monitoring low-pass gravity deviation and gyro rate impulses.

### 3.6 Vibration & Shock Filter (`idr/vibration_filter.hpp`)
- Online causal Butterworth bandpass (0.1–15 Hz) IIR filter (Direct Form II Transposed).
- Pothole detection ($>2.2g$ deviation above $1g$ or $>15.0\text{ m/s}^2$ horizontal) with 5-epoch cooldown and rolling median blanking.
- Engine idle detection: harmonic variance $\sigma^2 \in [0.02, 0.25]\text{ (m/s}^2)^2$.

### 3.7 Heading Supervisor (`idr/heading_supervisor.hpp`)
- GNSS course deriver: $\psi_{\text{cog}} = \text{atan2}(v_E, v_N)$ for $v_h \ge 2.5\text{ m/s}$, with $\sigma_\psi = \text{clip}(\sigma_v/v_h, 1^\circ, 15^\circ)$.
- 3D tilt-compensated magnetometer leveling in FLU frame.
- Anomaly detector ($|\Delta B| > 12\ \mu\text{T}$, gradient $> 35\ \mu\text{T/s}$).
- $\chi^2$ innovation gate ($\le 6.635$) and multi-epoch reacquisition dwell.

### 3.8 3D OSM Graph & Map Matching (`idr/osm_graph.hpp`, `map_matcher_3d.hpp`)
- WGS-84 $\leftrightarrow$ ENU projections (Bowring algorithm).
- Polyline segment projection (closest point, tangent, cross-track offset, deck elevation).
- 3D HMM with elevation barrier ($|\Delta h| > 4\text{ m} \implies \log P \le -50.0$).
- Fast 2D spatial hash grid for candidate edge retrieval in $<10\ \mu\text{s}$.

### 3.9 Phase 8 Hardening Components (`idr/speed_calibrator.hpp`, `longitudinal_sync.hpp`)
- Online RLS calibrator estimating scale $\alpha$ and bias $\beta$ with forgetting factor $\lambda=0.995$; frozen during outages.
- Causal longitudinal turn synchronizer applying along-track corrections at geometry vertices.

### 3.10 Navigation Engine (`idr/navigation_engine.hpp`)
- Integrates all stages into single `process_step(...)` cycle.
- 6-state FSM supervisor with causal outage detection and anti-snapback reacquisition.

---

## 4. Cross-Language Validation & Parity Testing Protocol

### 4.1 Test Vector Generation
A Python script (`scripts/generate_cross_language_vectors.py`) will dump exact input streams and step-by-step intermediate/output states:
1. Pure rotational maneuvers (Euler $\to$ Matrix $\to$ Rodrigues $\to$ Euler).
2. 500 feature vectors fed to Python GBDT vs C++ GBDT.
3. Strapdown mechanization: 1,000 steps of synthetic trajectory.
4. Filter updates: AI velocity, NHC, ZUPT, Baro, GNSS, Heading, Map constraint.
5. End-to-end full drive replay: authentic IO-VNBD drive `Vta01b` and `Driver_A_S3b`.

### 4.2 Numerical Equivalence Tolerances
| Quantity | Tolerance | Metric |
|---|---|---|
| GBDT Speed Prediction | $< 10^{-5}\text{ m/s}$ | Max Absolute Error |
| Rotation Matrix $\mathbf{R}$ | $< 10^{-12}$ | Frobenius Norm |
| Euler Angles (Roll, Pitch, Yaw) | $< 10^{-6}\text{ rad}$ | Absolute Error |
| Position $\mathbf{p}$ | $< 10^{-3}\text{ m}$ ($1\text{ mm}$) | Norm Error |
| Velocity $\mathbf{v}$ | $< 10^{-4}\text{ m/s}$ | Norm Error |
| Covariance $\mathbf{P}$ | $< 10^{-5}$ | Max Element Absolute Error |
| Map Matching Edge & Snapped Pos | Identical Edge ID, $< 10^{-3}\text{ m}$ | Categorical + Metric |

---

## 5. Standalone Edge CLI (`idr-cli`)

`idr-cli` will be a portable standalone command-line application:
- `--input <file.csv>`: Ingests authentic IO-VNBD or FOG CSV stream.
- `--rate <hz>`: Configurable sample rate (10, 50, 100, 200 Hz).
- `--benchmark`: Runs performance benchmark measuring per-cycle latency, throughput, and P95/P99 execution time.
- `--outage <start_s> <duration_s>`: Injects GNSS blackout window.
- `--output <out.csv>`: Emits high-rate navigation telemetry CSV.

---

## 6. Execution Phases & Verification Gates

```
Phase 9.1: Build Infrastructure & C++ Mathematics (SO(3), Rodrigues, Matrix)
   └── Unit tests for 3D math, DCM orthonormality, SVD projection.

Phase 9.2: GBDT Model Transpilation & Parity Verification
   └── Transpile Top-45 GBDT to C++ decision tables.
   └── Verify 10,000 test vectors against Python scikit-learn model (<1e-5 m/s MAE).

Phase 9.3: Preprocessing & Alignment Engine Port
   └── Butterworth IIR, shock blanking, engine idle detection.
   └── Alignment leveling, PCA surge extraction, cradle slip watchdog.

Phase 9.4: 15-State ESKF Core Port
   └── Strapdown propagation, Joseph-form update, reset, all measurement models.
   └── Numerical equivalence against Python ESKF.

Phase 9.5: Heading Supervisor, 3D OSM Graph & Map Matcher Port
   └── Bowring geodetics, spatial grid index, 3D HMM, soft road constraints.
   └── Speed calibrator & longitudinal synchronizer.

Phase 9.6: Full Seamless Navigation Engine Integration
   └── 6-state supervisor FSM, causal outage detection, seamless handover.

Phase 9.7: Standalone CLI (`idr-cli`) & 200 Hz Edge Benchmark
   └── Build `idr-cli`, measure 10/50/100/200 Hz throughput, latency percentiles, RAM.

Phase 9.8: Authentic IO-VNBD Outage Benchmark Replay (158 Outages)
   └── Run C++ engine on authentic IO-VNBD outages (10s, 30s, 60s, 120s).
   └── Cross-validate against Phase 8 results.

Phase 9.9: Full Regression Safety & Documentation
   └── Verify 155/155 Python tests PASS + all new C++ tests PASS.
   └── Update master tracker, matrices, and produce PHASE_9_CPP_EDGE_ENGINE_REPORT.md.
```
