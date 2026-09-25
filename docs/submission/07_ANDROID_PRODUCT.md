# 07 — ANDROID MOBILE APPLICATION
## 12-Screen Jetpack Compose Global Geographic Product

---

### 1. Application Screen Architecture

The Android product (`org.isro.idr`) is built using Kotlin and Jetpack Compose across 12 functional screens:

1. **Splash Screen:** Animated ISRO IDR identity welcome sequence.
2. **Sensor Calibration Screen:** 3D gimbal ring alignment visualizer with physical variance checks.
3. **Main Navigation Screen:** Primary HUD with active maneuver banner, turn card, speed/heading display, and GPS vs IDR track toggle.
4. **Map Explorer Screen:** Interactive map with layer toggles (3D buildings, terrain, landmarks, satellite).
5. **Sensor Monitor Screen:** Real-time 100 Hz waveform oscilloscopes (Accel, Gyro, Mag, Baro) and ingestion jitter meters.
6. **GNSS / NavIC Skyview Screen:** Live polar radar plot rendering active constellation satellites (GPS, NavIC, GLONASS, Galileo) and C/N0 telemetry.
7. **Offline Maps Manager Screen:** Regional map pack download manager with install/status tracking.
8. **Trip Analytics Screen:** Journey metrics, speed profiles, and drift error vs the 10% bound.
9. **Diagnostics Screen:** JNI latency benchmarks, memory metrics, and live simulated outage injection.
10. **Settings Screen:** Speed units, coordinate formats, 3D building rendering, and heading-up modes.
11. **Top Navigation Banner:** Seamless status indicator (`GNSS_OK` $\rightarrow$ `DEGRADED` $\rightarrow$ `DEAD_RECKONING_ACTIVE`).
12. **IdrCoverBanner:** Stylish branded identity component.

### 2. Sensor Synchronization & Rate Throttling
- **Bounded Circular FIFO Queue (`BoundedSensorQueue`):** Buffers incoming Android hardware sensor interrupts (~100 Hz Accel/Gyro, 20-50 Hz Mag, 10 Hz Baro).
- **Decoupled UI Flow:** Throttles UI recomposition to a steady 10 Hz StateFlow, preventing main-thread stutter while feeding the native C++ filter at full sensor rate.

### 3. JNI Bridge Architecture
- Communicates directly with `libidr_core.so` compiled with C++17 `-O3` flags via the Android NDK, ensuring identical mathematical outputs across mobile and desktop.
