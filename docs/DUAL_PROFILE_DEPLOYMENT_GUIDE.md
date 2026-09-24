# Dual-Profile Deployment Guide: Mobile & Edge Platforms

## 1. Overview

This guide details deployment workflows for the two operational profiles required by **PS 26168**:
1. **Mobile Profile** (Android / Smartphone MEMS IMU)
2. **Edge / FOG Profile** (Industrial Edge Computer / Tactical IMU)

---

## 2. Profile 1: Mobile Deployment (Android)

### 2.1 Prerequisites
- Android Studio Ladybug / Iguana or later
- Android NDK 25+ (r25c recommended)
- Minimum SDK: 26 (Android 8.0 Oreo), Target SDK: 34 (Android 14)
- Hardware: Mid-range or high-end smartphone with calibrated IMU and barometer (e.g. Google Pixel 6+, Samsung Galaxy S21+).

### 2.2 Integration Architecture
The mobile application operates via a layered architecture:
1. **Sensor Acquisition Layer (`com.idr.app.sensors`):**
   - High-rate accelerometer and gyroscope streams gathered at 100 Hz using `SensorManager.SENSOR_DELAY_FASTEST`.
   - Barometer sampled at 10 Hz.
   - Raw `GnssStatus` and `GnssMeasurements` collected at 1 Hz for NavIC constellation detection.
2. **JNI Bridge (`idr_jni.cpp`):**
   - Maps normalized sensor packets directly into the C++17 core engine via zero-copy direct byte buffers.
3. **UI / Navigation Loop:**
   - Decoupled from sensor acquisition; runs at 10 Hz to drive the MapLibre GL map view, vehicle orientation, and mode transition banners without frame drops.

### 2.3 Build Commands
```bash
cd android
./gradlew assembleRelease
# APK generated at app/build/outputs/apk/release/app-release.apk
```

---

## 3. Profile 2: Edge / FOG Deployment (Native C++17)

### 3.1 Prerequisites
- C++17 compliant compiler (`g++-10+`, `clang-12+`, or MSVC 2019+)
- CMake 3.16+
- Linux (Ubuntu 20.04/22.04 LTS, Debian 11/12, or Yocto/Embedded Linux)
- High-rate serial (RS-422/RS-232, 921600 baud) or UDP socket interface for external IMU ingestion.

### 3.2 Compilation
```bash
# Build native static and shared libraries
cmake -B idr-core/build -S idr-core -DCMAKE_BUILD_TYPE=Release
cmake --build idr-core/build --config Release

# Run automated C++ unit and parity tests
ctest --test-dir idr-core/build --output-on-failure
```

### 3.3 Standalone C++ Edge Daemon Example
```cpp
#include "idr/edge_engine.hpp"
#include <iostream>
#include <thread>

int main() {
    idr::EdgeNavigationEngine engine;

    idr::EngineConfig config;
    config.profile = idr::NavigationProfile::EDGE_FOG;
    config.imu_sample_rate_hz = 200.0;
    config.target_update_rate_hz = 200.0;
    
    // Explicit FOG calibration (measured via static multi-position test)
    config.calibration.accel_scale = {1.0002, 0.9998, 1.0001};
    config.calibration.gyro_bias = {1.2e-5, -0.8e-5, 0.5e-5}; // rad/s
    config.hardware_validated = false; // Mark true once calibrated on physical rig

    if (!engine.initialize(config)) {
        std::cerr << "Failed to initialize Edge Engine." << std::endl;
        return 1;
    }

    // Ingestion loop running at 200 Hz
    while (true) {
        idr::ImuSample imu = read_hardware_imu(); // Read from RS-422 / UDP
        engine.pushImu(imu);

        if (has_new_gnss()) {
            engine.pushGnss(read_gnss_fix());
        }

        engine.update(); // Executes in <300 microseconds
        auto nav = engine.getNavigationState();

        // Broadcast nav telemetry over DDS / ROS2 / ZeroMQ
        broadcast_nav(nav);
    }

    engine.shutdown();
    return 0;
}
```

---

## 4. Edge Sensor Calibration Procedure

To achieve optimal performance on physical FOG or industrial IMU hardware, follow this three-step calibration procedure prior to field deployment:

### 4.1 Mounting Misalignment Matrix ($R_{s2b}$)
1. Rigidly bolt the IMU chassis to the vehicle's structural frame.
2. Record physical lever-arm offsets from the vehicle's rear axle center to the IMU reference point: $[l_x, l_y, l_z]^\top$.
3. Compute the Euler rotation angles $[\phi, \theta, \psi]$ between the IMU reference axes and vehicle body axes (Forward-Left-Up).
4. Populate $R_{s2b} = R_z(\psi) R_y(\theta) R_x(\phi)$.

### 4.2 Static Gyroscope Bias Estimation
1. Keep the vehicle stationary in a temperature-controlled environment for 30 minutes.
2. Average angular rate measurements along each axis to estimate constant turn-on bias $b_{g,0}$.
3. Input $b_{g,0}$ into `config.calibration.gyro_bias`.

### 4.3 Accelerometer Scale Factor & Leveling
1. Use a 6-position static tumble test or gravity alignment to compute per-axis scale factors:
   $$s_a = \frac{\|a_{\text{measured}}\|}{g_0}$$
2. Populate `config.calibration.accel_scale`.

---

## 5. Validation Status Matrix

> [!IMPORTANT]
> The software timing path for the Edge Engine is fully verified, sustaining **>3500 Hz** (>17x the 200 Hz requirement). However, physical hardware testing on an actual FOG IMU is pending availability of physical sensor hardware.

| Milestone | Mobile Profile | Edge / FOG Profile |
|---|---|---|
| **Mathematical Formulation** | Complete & Frozen | Complete & Frozen |
| **Common SensorFrame Adapter** | Fully Implemented | Fully Implemented |
| **Unit & Parity Tests** | 100% Passing (205 tests) | 100% Passing (14 C++ tests) |
| **High-Rate Software Timing** | Validated (10-100 Hz) | Validated (~3500 Hz sustained) |
| **Physical Hardware Trials** | Complete (Pixel 7 / Prayagraj trial) | **PENDING PHYSICAL FOG HARDWARE** |
| **Learned Model Adaptation** | Trained on IO-VNBD schema | **PENDING FOG-SPECIFIC DATASET** |
