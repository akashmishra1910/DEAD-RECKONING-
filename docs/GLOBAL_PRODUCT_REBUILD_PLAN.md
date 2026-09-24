# SIH26168 Global Offline Geographic Navigation Product Rebuild Plan

## System Architecture & Objectives

Rebuilding the mobile UI into a **Global-First Offline Geographic Navigation Product** adhering to the 12-screen visual specification:
- **Map Architecture**: MapLibre Native Android + offline PMTiles/vector basemap + local byte-range storage.
- **IDR Navigation Inference**: `.idrmap` v1.0 binary graph + C++17 ESKF/GBDT/3D HMM (Track-B strictly frozen).
- **Design System**: Dark aerospace UI shell (`#06111F`, `#0B1B2B`, `#10263A`, `#17324A`), ISRO saffron (`#FF8A00`), GNSS blue (`#1688FF`), DR orange (`#FF7A00`), recovery cyan (`#00B8FF`), success green (`#18D47B`).

---

## 12-Screen System Specification

| # | Screen | Key Features & Design |
|---|---|---|
| 1 | **Splash / Welcome** | ISRO branding, mission title, rocket/globe imagery, "Get Started" CTA, auto/manual launch flow. |
| 2 | **Sensor Calibration** | 3D level gimbal ring, real-time sensor readiness checklist (Accel, Gyro, Mag, Baro), progress indicator, calibration completion. |
| 3 | **Navigation (GNSS Mode)** | Full-screen vector map, top green banner (`GNSS NAVIGATION`, sats, NavIC count, HDOP, accuracy), next-turn card, floating map controls, speed/heading/accuracy HUD. |
| 4 | **Navigation (DR Mode)** | Top amber banner (`DEAD RECKONING ACTIVE`, outage timer, maintaining IDR), saffron DR trajectory, estimated drift error. |
| 5 | **Navigation (Recovery)** | Top cyan banner (`GNSS REACQUIRED`, syncing indicator), dual GNSS/DR track convergence, accuracy recovery. |
| 6 | **Map Explorer** | Full global-to-street search & browse, layer selector modal (Standard, Satellite, Dark, Terrain, 3D Buildings, POIs, Water). |
| 7 | **Sensor Monitor** | 100 Hz live telemetry cards (Accel, Gyro, Mag, Baro), dynamic multi-trace real-time waveforms. |
| 8 | **GNSS / NavIC Status** | Polar Sky View constellation diagram (GPS, NavIC, GLONASS, Galileo, Beidou), SNR, NavIC in-view / in-fix counters. |
| 9 | **Trip Analytics** | Journey statistics (duration, distance, avg/max speed), drift comparison chart (GNSS vs Raw DR vs Map Matched), event timeline. |
| 10 | **Offline Maps** | Storage manager, region pack downloader (India, Europe, Americas, Custom), actual byte-size computation. |
| 11 | **Diagnostics** | System health table (IMU, GNSS, ESKF, HMM, JNI, RAM), performance metrics (latency, jitter, queue, dropped samples), event log. |
| 12 | **Settings** | Navigation preferences, map appearance, units & display, offline storage configurations. |

---

## 5 Execution Stages

1. **Stage 1**: Scaffold theme, color tokens, typography, bottom navigation shell (5 core tabs + 7 dedicated screens), navigation viewmodels, and screen layouts.
2. **Stage 2**: Implement PMTiles/MapLibre offline tile manager (copying from assets to app private file storage for byte-range support), world vector basemap, 3D building extrusions, and multi-scale continuous zoom engine.
3. **Stage 3**: Connect the 100 Hz phase accumulator sensor ingestion manager, JNI estimator bridge, and state transitions (GNSS $\to$ DR $\to$ Recovery $\to$ Restored).
4. **Stage 4**: Build real-time sensor waveforms, polar GNSS sky view, trip recording & drift analytics charts, and offline region manager.
5. **Stage 5**: Build debug APK (`arm64-v8a` + `x86_64`), inspect APK package, run 245 regression tests, and verify physical-device compatibility.
