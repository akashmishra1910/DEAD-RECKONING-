# AUDIT REPORT — UNFREEZE DEVELOPMENT & PRODUCTION CORRECTION
## SIH 2026 PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Audit Date:** September 27, 2026  
**Active Git Branch:** `unfreeze-live-navigation-correction`  
**Base Commit:** `6658e83`  
**Status:** Deep System Forensic & Architecture Mapping Complete

---

## 1. System Inventory: Production vs. Research

### A. Active Production Components
1. **Core C++ Engine (`idr-core/`)**:
   - `idr/navigation_engine.hpp`, `src/navigation_engine.cpp`: `SeamlessGNSSDeficitEngine` managing 15-state ESKF, alignment engine, vibration filter, heading supervisor, and supervisor state machine.
   - `idr/eskf.hpp`, `src/eskf.cpp`: 15-state Error-State Kalman Filter $(\mathbf{p}, \mathbf{v}, \mathbf{R}, \mathbf{b}_a, \mathbf{b}_g)$ with Joseph-form covariance updates and error-state reset Jacobians.
   - `idr/alignment_engine.hpp`, `src/alignment_engine.cpp`: In-vehicle gravity leveling (roll/pitch) and acceleration transient PCA (yaw/forward axis) computing phone-to-vehicle matrix $\mathbf{R}_p^b$, plus cradle watchdog slip detection.
   - `idr/gbdt_model.hpp`, `src/gbdt_model.cpp`: 120-tree, depth-5 GBDT forward speed regressor transpiled into static branchless C++ decision tables (zero external ML runtime, $<10\,\mu\text{s}$ execution).
   - `idr/feature_extractor.hpp`, `src/feature_extractor.cpp`: Causal 1.0s sliding window extracting 45 top kinematic features.
   - `idr/vibration_filter.hpp`, `src/vibration_filter.cpp`: Engine idle standstill detector and road pothole transient suppression filter.
   - `idr/heading_supervisor.hpp`, `src/heading_supervisor.cpp`: Multi-source heading supervisor with innovation gating and zero-angular-rate updates.
   - `idr/map_matcher_3d.hpp`, `src/map_matcher_3d.cpp`: Causal 3D HMM map matcher evaluating road candidate emission probabilities and topological Viterbi path decoding with vertical flyover barrier ($|\Delta h| > 4\,\text{m}$).
   - `idr/c_api.h`, `src/c_api.cpp`: Portable C-ABI exposing handles, step execution, and diagnostics for JNI and CLI.

2. **Android Ingestion & Presentation Layer (`android/`)**:
   - `android/app/src/main/cpp/jni_bridge.cpp`: JNI bridge marshalling data between JVM and native C++ engine with zero heap allocations during high-rate step execution.
   - `org.isro.idr.ingestion.SensorIngestionManager`: Android HAL sensor listener (accel, gyro, mag, pressure, rotation vector) with deterministic phase-accumulator 100 Hz downsampling and standstill detection.
   - `org.isro.idr.ingestion.GnssIngestionManager`: LocationListener & `GnssStatus.Callback` for geodetic-to-ENU projection, NavIC/IRNSS satellite parsing, and fix age tracking.
   - `org.isro.idr.ingestion.NavigationPipelineSynchronizer`: Coordinates 100 Hz IMU samples with latest valid GNSS fix ($<1.5\,\text{s}$ window) and executes native C++ steps.
   - `org.isro.idr.ui.viewmodel.NavigationViewModel`: Decouples 100 Hz native telemetry into 10 Hz StateFlow updates for Jetpack Compose UI.
   - `org.isro.idr.ui.map.RealGeographicMapCanvas`: Worldwide Web Mercator geographic map canvas with vector tile caching, live route waypoints, multi-track display, and cursor rendering.

### B. Research-Only Components (`idr_pipeline/`)
- `idr_pipeline/dual_profile.py`: Python experimental dual-profile architecture.
- `idr_pipeline/eskf_fusion.py`: Python research implementation of ESKF and Unified 3D Velocity-Vector model.
- `idr_pipeline/speed_scale_bias_calibrator.py`: Python online GNSS speed calibration model.
- `idr_pipeline/longitudinal_map_constraint.py`: Experimental longitudinal map synchronizer.
- Research benchmark scripts: `tests/test_phase14_*.py` through `tests/test_phase17_*.py`.

---

## 2. Research Causal Deployability Analysis (Phases 16 & 17)

| Research Component | Origin Phase | Causal / Deployable? | Status & Action |
|---|---|:---:|---|
| **Unified 3D AI Velocity Vector Update** | Phase 17 / Final Track B | **YES (Fully Causal)** | $\mathbf{v}_{\text{ai}}^b = [v_{\text{gbdt}}, l_x \omega_z, 0]^T$ with dynamic covariance $\mathbf{R}_v$. Eliminates competition between scalar speed and NHC. Port to C++ `eskf.cpp`. |
| **Dynamic Lateral Noise Modulation ($c_{\text{lat}}$)** | Phase 17 | **YES (Fully Causal)** | Modulates $\sigma_{vy}$ based on turn rate, speed, and lateral accel ($0.025\,\text{m/s}$ straight cruise to $0.40\,\text{m/s}$ sharp turns). Port to C++ `navigation_engine.cpp`. |
| **Pre-Outage GNSS Doppler Velocity Alignment** | Phase 16 | **YES (Fully Causal)** | Uses valid GNSS velocity vector fixes during GNSS-available periods to constrain accelerometer and gyro biases before outage. Port to C++ `navigation_engine.cpp`. |
| **Kinematic Consistency ZARU Clamping** | Phase 14 / 17 | **YES (Fully Causal)** | Updates gyro Z bias during straight cruising when lateral acceleration matches $v \cdot \omega_z \approx 0$. Port to C++ `navigation_engine.cpp`. |
| **Standstill Engine Idle ZUPT Injection** | Phase 15 / 17 | **YES (Fully Causal)** | Applies 3D velocity zero update ($\mathbf{v} \approx \mathbf{0}$) when vibration filter detects idling or accelerometer variance $< 0.08\,\text{m}^2/\text{s}^4$. Port to C++ `eskf.cpp`. |
| **Ground-Truth Map/Velocity Access** | Experimental | **NO (Oracle)** | **STRICTLY REJECTED**. Ground truth is forbidden during navigation. |
| **Non-Causal Trajectory Smoothing (RTS Smoother)** | Offline Analysis | **NO (Non-Causal)** | **STRICTLY REJECTED**. Smoothing uses future epochs; live navigation must remain 100% causal and forward-only. |

---

## 3. Detailed Dataflow and Execution Tracing

### E. Where Android Sensor Data Enters
1. Physical hardware interrupts fire `SensorEventListener.onSensorChanged(event)` in `SensorIngestionManager.kt` on a background `HandlerThread` (`"SensorIngestionThread"`).
2. Accelerometer samples pass through `TimestampPolicy` and a deterministic phase accumulator emitting at exactly 100.0 Hz.
3. Gyroscope, magnetometer, barometer, and rotation vector readings update volatile state vectors.
4. Each 100 Hz sample forms an `ImuSample` passed to `NavigationPipelineSynchronizer.processSingleImuSample(imu)`.

### F. Where GNSS and NavIC Data Enters
1. Android `LocationManager.requestLocationUpdates(GPS_PROVIDER)` triggers `GnssIngestionManager.onLocationChanged(location)`.
2. `LocationManager.registerGnssStatusCallback` triggers `GnssStatus.Callback.onSatelliteStatusChanged(status)`.
3. Geodetic fixes are converted to local tangent ENU coordinates via `GeoCoordinates.geodeticToEnu(...)`.
4. Fixes younger than 1.5s are paired with 100 Hz IMU samples in `NavigationPipelineSynchronizer`.

### G. Where Sensor Frames Are Transformed
1. `SensorIngestionManager`: Converts raw sensor units to SI ($\text{m/s}^2$, $\text{rad/s}$, $\mu\text{T}$, $\text{m}$).
2. `InVehicleAlignmentEngine`: Applies mounting rotation $\mathbf{R}_p^b$ to transform phone frame to vehicle body frame: $\mathbf{a}^b = \mathbf{R}_p^b \mathbf{a}^p$, $\boldsymbol{\omega}^b = \mathbf{R}_p^b \boldsymbol{\omega}^p$.
3. `ErrorStateKalmanFilter`: Multiplies body specific force by navigation attitude matrix: $\mathbf{f}^n = \mathbf{R} \mathbf{f}^b$, adds gravity $\mathbf{g}^n = [0, 0, -9.80665]^T$, and integrates velocity and position in ENU frame.

### H. Where Velocity and Speed Are Calculated
1. **ESKF Velocity**: $\mathbf{v}^n = [v_E, v_N, v_U]^T$ integrated at 100 Hz and updated via GNSS velocity, AI velocity vector, and ZUPT.
2. **Horizontal Ground Speed**: Computed as $\text{speed} = \sqrt{v_E^2 + v_N^2}$ in $\text{m/s}$, converted to $\text{km/h}$ via $\times 3.6$.
3. **AI GBDT Speed**: Inferred by 120-tree GBDT regressor from 45 sliding window features.

### I. Where Standstill is Detected
1. **Physical IMU Variance**: `SensorIngestionManager` tracks sliding 25-sample accelerometer norm variance ($< 0.08\,\text{m}^2/\text{s}^4$) and gyro rate ($< 0.15\,\text{rad/s}$).
2. **Engine Idle Vibration**: `VibrationPotholeFilter` detects characteristic engine idle frequencies.
3. **Standstill Enforcement**: Triggers ESKF 3D ZUPT ($\mathbf{v}^n \to \mathbf{0}$) and halts position propagation.

### J. Where Map Matching Affects/Displays Position
1. `Causal3DMapMatcher`: Evaluates HMM emission & transition probabilities against `.idrmap` road graph.
2. Soft road constraint updates ESKF cross-track position and heading when confidence is HIGH/MEDIUM.
3. `NavigationTelemetry` exports both `raw_dr_pos_enu` and `map_matched_pos_enu`.

### K. Where Live Cursor Position is Generated
1. `RealGeographicMapCanvas.kt`: Converts vehicle geodetic coordinates to Web Mercator pixel coordinates ($256 \times 2^z$).
2. Dynamically draws the high-visibility vehicle chevron oriented with true physical device heading or navigation azimuth.

---

## 4. Root Causes of Identified Issues

1. **GNSS/NavIC Status Showing 0**:
   - `NavigationViewModel.onGnssSample` reconstructed `GnssStatusState` on every 1 Hz location fix, dropping `satellites` list and resetting `navicSatellitesUsed` to default 0.
   - `onLocationChanged` satellite counts were out of sync with `GnssStatus.Callback` invocations.
2. **Speed & Velocity Inaccuracies & Wild DR Drift**:
   - Android `NavigationPipelineSynchronizer` passed `aiSpeedMps = -1.0` assuming C++ engine would automatically run GBDT regression internally, but `SeamlessGNSSDeficitEngine` lacked an internal `SlidingWindowFeatureExtractor`. As a result, in DR mode, **NO AI SPEED UPDATES WERE APPLIED AT ALL**, causing open-loop accelerometer drift!
3. **Standstill Drift**:
   - Missing tight coupling between Android standstill detection and C++ ZUPT execution during DR outages when GBDT speed was unset.
4. **Live Cursor Route Snapping vs Multi-Layer Visibility**:
   - The map canvas lacked simultaneous multi-track debug rendering overlays for GNSS, Raw DR, and Map-Matched positions.

---

## 5. Planned Engineering Actions
1. Embed Causal GBDT Feature Extractor inside `SeamlessGNSSDeficitEngine::process_step` so that AI speed is always autonomously computed from IMU when not explicitly provided.
2. Integrate Unified 3D Velocity-Vector update ($\mathbf{v}_{\text{ai}}^b = [v_{\text{gbdt}}, l_x \omega_z, 0]^T$ with dynamic covariance $\mathbf{R}_v$) into `idr-core/src/eskf.cpp`.
3. Fix GNSS/NavIC StateFlow update pipeline in `GnssIngestionManager.kt` and `NavigationViewModel.kt` to preserve all satellite metadata.
4. Enhance standstill detector in C++ and Android to apply rigorous ZUPT when vehicle is at rest.
5. Add multi-layer debug trajectory visualization toggle in `RealGeographicMapCanvas.kt`.
6. Add unit tests and parity tests, run full regression test suite, and rerun authentic 158-outage benchmark.
