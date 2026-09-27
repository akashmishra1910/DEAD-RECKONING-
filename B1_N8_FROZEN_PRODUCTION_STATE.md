# B1/N8 FROZEN PRODUCTION STATE SPECIFICATION

**Document ID:** `SPEC-B1-N8-FROZEN-01`  
**Date:** September 27, 2026  
**Git Commit Hash:** `af1b68fb451b0587500aa1ba15ad433221cf8239`  
**Branch:** `main` (synchronized with `origin/main`)  
**Status:** FROZEN PRODUCTION ARCHITECTURAL STATE  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Architectural Definition of B1 / N8

The **B1 / N8 Production Architecture** is the definitive, strictly causal, external-runtime-free Dead Reckoning system developed for SIH 2026 / ISRO-SAC PS 26168. It unites the validated 15-state Error-State Kalman Filter with causal pre-outage Doppler observability, bounded-slip lateral kinematics, and in-flight zero angular rate updates (ZARU).

```
+-----------------------------------------------------------------------------------+
| PIPELINE STAGES IN B1 / N8:                                                       |
|                                                                                   |
| 1. In-Vehicle Mounting Alignment:                                                 |
|    Gravity leveling (pitch/roll) + longitudinal acceleration PCA (yaw)            |
| 2. Vibration & Engine Idle Filter:                                                |
|    4th-order Butterworth bandpass (0.5-20 Hz) + 20-30 Hz idle vibration detector |
| 3. Pre-Outage GNSS Doppler Velocity & ESKF Handover (3.0s window):                |
|    Fuses valid pre-outage GNSS Doppler velocity vectors into ESKF observer        |
|    Observes accelerometer bias b_a, gyro bias b_g, and converges covariance P     |
| 4. Causal AI Speed Regressor:                                                     |
|    Huber Loss GBDT (120 trees, depth 5), 45 causal IMU sliding window features    |
|    Inference clamped to [0.0, 35.0] m/s; NO OBD-II, NO CAN, NO gt_speed           |
| 5. 15-State Error-State Kalman Filter (ESKF):                                     |
|    States: [p(3), v(3), q(4), b_a(3), b_g(3)] in ENU navigation / FLU body frame  |
| 6. Bounded-Slip Adaptive Lateral NHC:                                             |
|    Target variance: R_target = R0 * (1 + 25*w_z^2 + 12*a_cent^2 + 5*a_y^2)         |
|    Strictly bounded: R_lat in [R0, R_max], where R_max = (0.40 m/s)^2 = 0.16 m^2/s^2 |
| 7. Causal In-Flight Gyroscope Kalman ZARU:                                        |
|    Fires on confirmed straight cruising (|w_z| < 0.005, |a_y| < 0.15, v > 4.0 m/s) |
|    Applies Kalman error-state update on b_g[2] with R = 0.005^2 rad^2/s^2         |
| 8. 3D HMM Topological Map Matcher:                                                |
|    Viterbi decoding with elevation barrier |Delta_h| > 4.0 m                      |
|    Soft road heading fusion with chi^2 innovation gating (gate = 25.0)            |
+-----------------------------------------------------------------------------------+
```

---

## 2. Inventory of Source Files & Checkpoints

### 2.1 Core Python Pipeline (`idr_pipeline/`)
- `idr_pipeline/eskf_fusion.py`: 15-state ESKF, quaternion kinematics, NHC, ZUPT, ZARU, altitude, and AI velocity update routines.
- `idr_pipeline/gnss_ins_fusion_engine.py`: 6-state supervisory transition engine (`SeamlessGNSSDeficitEngine`), pre-outage handover manager, and multi-sensor arbitrator.
- `idr_pipeline/alignment_engine.py`: Causal static gravity leveling and batch/online PCA mounting calibration (`InVehicleAlignmentEngine`).
- `idr_pipeline/vibration_filter.py`: IIR filter cascade and engine idling detector (`VibrationPotholeFilter`).
- `idr_pipeline/heading_supervisor.py`: Tri-level heading supervisor (GNSS COG, calibrated magnetometer, gyro integration).
- `idr_pipeline/offline_osm_graph.py`: Topological road network graph with multi-layer elevation support (`OfflineOSMGraph`).
- `idr_pipeline/map_matcher_3d.py`: Causal 3D HMM map matcher with vertical flyover disambiguation (`Causal3DMapMatcher`).
- `idr_pipeline/dataset_loader.py`: Generic IO-VNBD dataset loader and causal 45-feature extractor.

### 2.2 Model Checkpoints (`models/`)
- `models/production/gbdt_speed_model_iovnbd_top45.joblib`: Production 45-feature Huber GBDT regressor (570 KB).
  - Model type: `sklearn.ensemble.GradientBoostingRegressor`
  - Hyperparameters: `n_estimators=120`, `max_depth=5`, `learning_rate=0.08`, `loss='huber'`, `alpha=0.9`
  - Scaler: `sklearn.preprocessing.StandardScaler`
  - Input dimension: 45 causal time-domain and frequency-domain statistics over 10 samples (1.0s)

### 2.3 C++ Edge Engine (`idr-core/`)
- `idr-core/include/idr/navigation_engine.hpp` & `idr-core/src/navigation_engine.cpp`: Unified C++17 navigation coordinator.
- `idr-core/include/idr/eskf.hpp` & `idr-core/src/eskf.cpp`: Full 15-state ESKF implementation in Eigen3.
- `idr-core/include/idr/gbdt.hpp` & `idr-core/src/gbdt.cpp`: Transpiled static branchless C++ decision tables (zero runtime dependencies).
- `idr-core/include/idr/map_matcher.hpp` & `idr-core/src/map_matcher.cpp`: C++ topological map matcher.

### 2.4 Android Front-End (`android/`)
- `android/app/src/main/cpp/idr_jni_bridge.cpp`: JNI interface bridging Kotlin sensor threads to C++ `idr-core`.
- `android/app/src/main/java/org/isro/idr/navigation/DeadReckoningEngine.kt`: Kotlin real-time sensor ingestion, circular buffer management, and UI dispatching.

---

## 3. Exact Quantitative Parameter Registry

| Parameter Subsystem | Symbol / Name | Value | Physical Unit | Operational Rationale |
|---|---|:---:|:---:|---|
| **Filter Mechanization** | $\Delta t$ | 0.10 | seconds | 10 Hz mobile baseline |
| | $\sigma_a$ (accel noise) | 0.15 | $\text{m/s}^2$ | Smartphone IMU standard deviation |
| | $\sigma_g$ (gyro noise) | 0.008 | $\text{rad/s}$ | Gyroscope angle random walk |
| | $R_{\text{alt}}$ | 0.80 | $\text{m}^2$ | Barometric altitude measurement noise |
| **Pre-Outage Handover** | $T_{\text{pre}}$ | 3.0 | seconds | Immediate 30-sample pre-outage window |
| | $R_{\text{vel}}$ | $0.35^2$ | $\text{m}^2/\text{s}^2$ | GNSS Doppler velocity measurement covariance |
| | $R_{\text{yaw,pre}}$ | 0.08 | $\text{rad}^2$ | GNSS course angle variance |
| **Bounded-Slip NHC** | $R_0$ | $0.05^2 = 0.0025$ | $\text{m}^2/\text{s}^2$ | Zero-slip baseline covariance |
| | $R_{\text{max}}$ | $0.40^2 = 0.16$ | $\text{m}^2/\text{s}^2$ | **Strict bound preventing lateral runaway** |
| | $\alpha_\omega$ | 25.0 | dimensionless | Yaw rate covariance inflation factor |
| | $\alpha_c$ | 12.0 | dimensionless | Centripetal acceleration inflation factor |
| | $\alpha_a$ | 5.0 | dimensionless | Lateral acceleration inflation factor |
| | $\beta$ | 0.70 | dimensionless | 1st-order IIR smoothing coefficient |
| **In-Flight ZARU** | $\omega_{z,\text{thresh}}$ | 0.005 | $\text{rad/s}$ ($0.29^\circ/\text{s}$) | Straight driving threshold |
| | $a_{y,\text{thresh}}$ | 0.15 | $\text{m/s}^2$ | Lateral acceleration threshold |
| | $v_{\text{thresh}}$ | 4.0 | $\text{m/s}$ (14.4 km/h) | Minimum cruising speed |
| | $N_{\text{ticks}}$ | 8 | samples (0.8 s) | Temporal confirmation persistence |
| | $R_{\text{zaru}}$ | $0.005^2$ | $\text{rad}^2/\text{s}^2$ | Gyro bias observation covariance |
| **Map Matching** | $h_{\text{barrier}}$ | 4.0 | meters | Flyover / underpass layer separation barrier |
| | $\chi^2_{\text{gate}}$ | 25.0 | dimensionless | Road heading innovation gate ($\alpha = 0.01$) |
| | $r_{\text{yaw,map}}$ | 0.050 – 0.087 | radians | Multi-hypothesis road heading covariance |
