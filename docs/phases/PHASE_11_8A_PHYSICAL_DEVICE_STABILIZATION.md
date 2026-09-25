# Phase 11.8A — Physical Device Stabilization & Geographic Map Integration

**Project**: ISRO / SIH PS 26168 — AI/ML Based Intelligent Dead Reckoning System  
**Phase**: 11.8A (Physical Device Hardware Hardening & UI Geographic Map Integration)  
**Date**: September 2026  
**Status**: COMPLETE & VERIFIED  
**Track B Baselines**: Strictly FROZEN ($C_0 = 76.34\%$, $N_0 = 34.43\%$, $N_7 = 35.71\%$)  

---

## 1. Executive Summary

Phase 11.8A addresses physical-device integration anomalies discovered during live physical-hardware deployment on Android devices:
1. **Sensor Rate Mismatch & Queue Saturation**: Raw Android sensor interrupts at `SENSOR_DELAY_FASTEST` fired at ~468.8 Hz, while the navigation engine is budgeted for ~100 Hz. The bounded queue saturated (500/500 items) with ~32,428 overflow drop events.
2. **Tactical Grid to Geographic Map Transition**: Upgraded the Android UI map canvas from a synthetic metric ENU grid to high-fidelity vector rendering of real OpenStreetMap road networks extracted from `.idrmap` binary files.
3. **GNSS Fix & Anchor Seeding**: Fixed cold-start GNSS standby issues by seeding the initial anchor from cached `getLastKnownLocation` and ensuring main thread looper dispatch.

---

## 2. Root Cause Analysis & Sensor Rate Decimation

### 2.1 The 468.8 Hz Interrupt Overload
On modern hardware (e.g. Google Pixel), `SensorManager.SENSOR_DELAY_FASTEST` produces hardware interrupts as fast as 468–500 Hz for IMU components (accelerometer and gyroscope). 

**Previous Behavior**:
- Every sensor event was pushed into `BoundedSensorQueue(capacity = 500)`.
- Without a background polling worker actively dequeuing, the queue reached 500 elements rapidly, triggering overflow drops.
- Furthermore, feeding 468 Hz data directly into the 100 Hz ESKF would violate the discrete noise formulation and dt assumptions.

### 2.2 Causal Rate Decimation Design
Implemented deterministic time-gating in `SensorIngestionManager.kt`:
- **Target Ingestion Frequency**: 100 Hz ($dt_{\text{target}} = 10.0\text{ ms}$).
- **Interval Gate**: $9.5\text{ ms}$ minimum epoch spacing to allow for sensor jitter.
- **Separate Rate Instrumentation**:
  - `callbackHz`: Tracks raw hardware interrupt rate from the Android HAL (~468.8 Hz).
  - `consumedHz`: Tracks the decimated stream admitted into the navigation pipeline (~100.0 Hz).
  - P95 / Min / Max $dt$ tracking for real-time jitter detection.
- **Active Queue Drainage**: `NavigationPipelineSynchronizer.kt` continuously drains the sensor queues during step execution, keeping queue depth near 0–2 samples.

---

## 3. GNSS Physical-Device Stabilization

1. **Looper Binding**: Explicitly passed `Looper.getMainLooper()` to `requestLocationUpdates` to ensure callbacks fire reliably on all Android OS versions without thread starvation.
2. **Anchor Seeding**: On start, `GnssIngestionManager` retrieves `getLastKnownLocation(GPS_PROVIDER)` and `getLastKnownLocation(FUSED_PROVIDER)` to seed the geodetic reference anchor $(\phi_0, \lambda_0, h_0)$ immediately, avoiding cold-start coordinate singularity.
3. **NavIC / IRNSS Constellation Monitoring**: Registered `GnssStatus.Callback` to count visible and used-in-fix NavIC satellites (`CONSTELLATION_IRNSS`), providing real-time telemetry on the UI status banner.

---

## 4. Geographic Offline Map Integration

### 4.1 Bundled `.idrmap` Binary Assets
The following pre-compiled `.idrmap` v1.0 binary map packages are bundled directly into `android/app/src/main/assets/maps/`:
- `sample_prayagraj.idrmap` (Prayagraj corridor validation track)
- `dense_metro.idrmap` (Delhi Metro multi-layer flyover/tunnel network)
- `regional_network.idrmap` (Regional transit corridor)

### 4.2 Zero-Allocation Kotlin Binary Reader
Implemented `IdrMapBinaryReader.kt` to decode the `.idrmap` v1.0 binary layout:
- 128-byte header parsing (magic `IDRM`, bounding box, node/edge/geometry counts).
- 40-byte edge record decoding (start/end nodes, layer index, bridge/tunnel flags, bidirectional flags).
- 16-byte geometry point stream extraction.
- String table resolution for human-readable street names.

### 4.3 Coordinate Transformation & Canvas Rendering
- **`IdrMapData.kt`**: Projects WGS-84 polyline vertices into metric East-North-Up (ENU) coordinates relative to the active anchor.
- **`TacticalGridMapCanvas.kt`**:
  - Vector road network rendering with stroke width scaling.
  - Multi-layer visualization: Elevation shadow rendering for flyovers/bridges (`layer > 0`), dashed line styling for subterranean tunnels (`layer < 0`).
  - Active matched road segment highlighted with a high-contrast saffron glow.
  - Heading-oriented vehicle icon with uncertainty ellipse rendering.
  - Ground Truth vs. IDR trajectory toggle overlays.

---

## 5. Verification & Test Evidence

### 5.1 Python & C++ Regression Suite
- **Full Pytest Suite**: 189 passed, 0 failed.
- **Phase 11.8 Offline Nav Suite**: 9/9 passed (including native C++ test execution).
- **Frozen Track B Baseline Invariance**:
  - $C_0$ (Raw Kinematic DR): $76.34\%$ median drift (12/158 pass)
  - $N_0$ (Classical EKF + GBDT): $34.43\%$ median drift (28/158 pass)
  - $N_7$ (Full Production Estimator): $35.71\%$ median drift (36/158 pass)

### 5.2 Android APK Artifact Details
- **File**: `android/app/build/outputs/apk/debug/app-debug.apk`
- **Size**: 10,763,490 bytes (10.26 MB)
- **SHA-256**: `F59EFA019C5E277F2C27CAB63BD363768B0208B7D3088DB614F3B4A9F2E85E54`
- **Architectures**: `arm64-v8a`, `x86_64`
- **Native JNI Library**: `libidr_jni.so` + `libc++_shared.so`
- **Bundled Maps**: `sample_prayagraj.idrmap`, `dense_metro.idrmap`, `regional_network.idrmap`
