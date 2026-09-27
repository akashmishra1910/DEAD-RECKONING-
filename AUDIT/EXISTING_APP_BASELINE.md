# EXISTING ANDROID APPLICATION BASELINE SPECIFICATION

**Document ID:** `BASELINE-ANDROID-APP-01`  
**Date:** September 27, 2026  
**Status:** FROZEN ANDROID REGRESSION BASELINE  
**Application ID:** `org.isro.idr`  
**Package Name:** `org.isro.idr`  
**Authority:** Mobile Systems & Application Lead

---

## 1. Executive Summary

This document establishes the frozen baseline for the existing Android application. **Under Rule 19 of the master directive, the user interface, screens, Compose layouts, colors, branding, user flows, application ID, and permissions are 100% frozen**. The B1 / N8 update operates strictly underneath this existing application as an engine upgrade.

---

## 2. Application Identity & Architecture

```
+-----------------------------------------------------------------------------------+
| ANDROID APPLICATION METADATA:                                                     |
| - Application ID:          org.isro.idr                                           |
| - Namespace:               org.isro.idr                                           |
| - Version Code:            1                                                      |
| - Version Name:            1.1.0                                                  |
| - Minimum SDK:             26 (Android 8.0 Oreo)                                  |
| - Target SDK:              34 (Android 14)                                        |
| - Compile SDK:             35 (Android 15)                                        |
| - NDK Version:             25.1.8937393                                           |
| - Build Tooling:           Gradle Kotlin DSL (build.gradle.kts)                   |
| - UI Toolkit:              Jetpack Compose (BOM 2024.02.00)                       |
+-----------------------------------------------------------------------------------+
```

---

## 3. UI & Screens Inventory (FROZEN)

The application possesses 10 core screens and 12 reusable tactical components that must remain visually and functionally identical:

### 3.1 Primary Screens
1. **`MainNavigationScreen`** (`org.isro.idr.ui.screens.MainNavigationScreen`):
   - Central interactive navigation display with MapLibre vector map rendering.
   - Dual-trajectory visualization with live toggle (`TrajectoryToggleControls.kt`): Raw GNSS track vs. IDR Dead Reckoning track.
   - Tactical HUD banner displaying current navigation mode (`GNSS_OK`, `GNSS_DEGRADED`, `DEAD_RECKONING`), speed in km/h, and compass heading.
   - Next-turn directional guidance card (`NextTurnCard.kt`) and road layer indicator (Ground / Flyover / Underpass / Tunnel).
2. **`SensorMonitorScreen`** (`org.isro.idr.ui.screens.SensorMonitorScreen`):
   - Live oscilloscope waveform rendering of tri-axial accelerometer, gyroscope, magnetometer, and barometric pressure.
3. **`DiagnosticsScreen`** (`org.isro.idr.ui.screens.DiagnosticsScreen`):
   - ESKF covariance trace monitor ($\text{Tr}(P)$), $\chi^2$ innovation gate status, and multi-hypothesis map likelihood metrics.
4. **`GnssNavicScreen`** (`org.isro.idr.ui.screens.GnssNavicScreen`):
   - Dedicated ISRO NavIC / IRNSS constellation monitor (`CONSTELLATION_IRNSS`), showing live satellite polar plot, carrier-to-noise density ($C/N_0$), and active fix count.
5. **`TripAnalyticsScreen`** (`org.isro.idr.ui.screens.TripAnalyticsScreen`):
   - Cumulative drift percentage ledger, outage duration breakdown, and trip trajectory log.
6. **`SettingsScreen`** (`org.isro.idr.ui.screens.SettingsScreen`):
   - Offline map layer selection, vehicle dynamics profile selector (Passenger Car vs. Two-Wheeler lean compensation), and replay mode configuration.
7. **`MapExplorerScreen` & `OfflineMapsScreen`**:
   - Local `.idrmap` binary management and cached offline basemap tile control.
8. **`SensorCalibrationScreen`**:
   - Guided phone mounting calibration workflow (gravity leveling and PCA alignment).
9. **`SplashScreen`**:
   - Sensor capability hardware check and background map cache loading.

---

## 4. Sensor-to-Engine Data Flow

```
[Hardware Sensors]
   ├── Accelerometer (TYPE_ACCELEROMETER) @ 100 Hz
   ├── Gyroscope (TYPE_GYROSCOPE) @ 100 Hz
   ├── Magnetometer (TYPE_MAGNETIC_FIELD) @ 50 Hz
   ├── Barometer (TYPE_PRESSURE) @ 10 Hz
   └── GNSS (GnssStatus + GnssMeasurement) @ 1 Hz
           │
           ▼
[SensorIngestionManager.kt]
   ├── Thread-safe Circular Ring Buffer (`BoundedSensorQueue.kt`)
   └── Hardware Timestamp Synchronization (`TimestampPolicy.kt`)
           │
           ▼ (10 Hz Navigation Epoch)
[NativeNavigationEngine.kt]
   └── JNI Call: nativeProcessStepExt(...)
           │
           ▼
[jni_bridge.cpp]
   └── C ABI Call: idr_engine_step_ext(...)
           │
           ▼
[idr-core: NavigationEngine.cpp]
   ├── Alignment Engine (Mounting Matrix R_p^b)
   ├── Vibration Filter (Butterworth + Idle Detector)
   ├── Transpiled Huber GBDT Model (45 Features)
   ├── 15-State ESKF Mechanization (Eigen3)
   ├── Bounded-Slip NHC (R_max = 0.16)
   ├── In-Flight Gyro ZARU
   └── 3D HMM Map Matcher (OSM Graph)
           │
           ▼ (25-element flat double array)
[NavigationTelemetry.kt]
   └── Updates Jetpack Compose UI State (`NavigationUiState.kt`)
```

---

## 5. Current Native Engine & Model Specification

- **Native Shared Library:** `libidr_jni.so` (links static `libidr_core.a`).
- **GBDT Model:** Transpiled 120-tree Huber GBDT model compiled into static C++ decision tables in `gbdt_model.cpp` (zero runtime dependencies, zero external ML library).
- **JNI C-ABI Contract:** Defined in `idr-core/include/idr/c_api.h` and implemented in `android/app/src/main/cpp/jni_bridge.cpp`.
- **API Version:** Major 1, Minor 1, Patch 0 (`0x010100`).

---

## 6. Build Baseline & Host Toolchain Status

- **C++ Engine:** Builds cleanly via CMake (`libidr_core.a` and `libidr_core.dylib` verified).
- **CTest Suite:** 14 / 14 C++ tests passing in `idr-core/build`.
- **Python-to-C++ Golden Parity:** Verified across 5,101 vectors with zero mismatch ($0.000000\text{ m/s}$ MAE).
- **Android Gradle Build:** Android project structure, Manifest, Compose layouts, and CMakeLists.txt are fully defined and intact.
