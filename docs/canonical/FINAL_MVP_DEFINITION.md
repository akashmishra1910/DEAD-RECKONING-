# FINAL MVP DEFINITION — SIH26168
## ISRO IDR: Intelligent Dead Reckoning for Seamless Navigation

**Document ID:** `DOC-CANONICAL-MVP-DEF-01`  
**Version:** 2.0.0 (Submission Freeze)  
**Date:** September 26, 2026  
**Repository State:** Frozen at HEAD `eb7da70`  
**Evaluation Standard:** Rule Zero — Scientific and Product Honesty  

---

## 1. Executive Summary & Problem-Solution Definition

### Product Identity
- **Product Name:** ISRO IDR — Intelligent Dead Reckoning for Seamless Navigation
- **Sponsor / Mentors:** Indian Space Research Organisation (ISRO) / Space Applications Centre (SAC) — Problem Statement 26168

### The Problem
Conventional vehicle navigation systems rely entirely on space-based GNSS positioning (GPS, NavIC/IRNSS, Galileo, GLONASS). When a vehicle enters satellite-denied environments—such as urban canyons, tunnels, multi-level flyovers, underpasses, and multi-deck parking structures—satellite signals suffer severe attenuation, destructive multipath reflections, or total loss-of-lock. Under GNSS loss, standard double-integration of smartphone-grade IMU sensors experiences exponential drift ($\mathcal{O}(t^2 \text{ to } t^3)$), leading to catastrophic track divergence (>100% error), while traditional 2D map matchers erroneously snap elevated vehicles onto ground-level service roads.

### The Solution
A decoupled, dual-target (Mobile & Edge) inertial navigation system combining:
1. **Automated In-Vehicle Alignment:** Dynamic gravity-leveling and PCA forward heading estimation with a cradle watchdog.
2. **AI/ML Speed & Vibration Filtering:** Transpiled 120-tree GBDT regressing forward speed directly from 1.0s vibration/kinematics windows without OBD-II/CAN-bus.
3. **15-State Error-State Kalman Filter (ESKF):** Continuous-discrete error-state formulation estimating position, velocity, attitude quaternion, accelerometer bias, and gyroscope bias, reinforced by Non-Holonomic Constraints (NHC) and standstill Zero-Velocity Updates (ZUPT).
4. **Causal 3D Multi-Layer Map Matching:** 3D HMM with an elevation barrier ($|\Delta h| > 4.0\text{ m}$) preventing false snaps across elevated flyovers, bridges, and underpasses.
5. **GNSS Deficit Supervisor:** Chi-Square innovation gate ($\gamma \le 11.35$) safeguarding the filter against tunnel-exit multipath spikes and managing seamless transitions between GNSS, Dead Reckoning, and Map-Aided states.

---

## 2. Product Architecture & Layer Classification

```
========================================================================================
                              ISRO IDR MVP ARCHITECTURE
========================================================================================

                 [ PRODUCTION MVP — FROZEN AT HEAD eb7da70 ]
                                    │
    ┌───────────────────────────────┴───────────────────────────────┐
    │                                                               │
┌───▼───────────────────────────┐               ┌───────────────────▼───────────────────┐
│ idr-core (Portable C++17)     │               │ idr-android (Android Application)     │
│ - 15-State ESKF Engine        │ ──JNI Bridge─►│ - 12-Screen Jetpack Compose Product   │
│ - Transpiled GBDT Speed Model │               │ - Live NavIC Satellite Skyview Canvas │
│ - Non-Holonomic Constraints   │               │ - OSM / Valhalla-aligned Vector Map   │
│ - Standstill ZUPT Detector    │               │ - Bounded Circular Sensor FIFO Queue  │
│ - 3D HMM Elevation Barrier    │               │ - Real-Time Diagnostics & Trip HUD    │
│ - Chi-Square Deficit Gating   │               │ - 100% Offline Regional Map Matcher   │
└───────────────────────────────┘               └───────────────────────────────────────┘
    │
    └───► idr-cli (Standalone Edge Engine)
          - High-throughput (>50 kHz) streaming runner for external IMU logs/sockets

────────────────────────────────────────────────────────────────────────────────────────

                 [ RESEARCH EXTENSIONS — idr_pipeline (Python) ]
                                    │
    ┌───────────────────────────────┴───────────────────────────────┐
    │                                                               │
┌───▼───────────────────────────┐               ┌───────────────────▼───────────────────┐
│ Algorithmic Exploration       │               │ Validated Research Results (IO-VNBD)  │
│ - Phase 13 Spectral Bands     │               │ - Phase 16 Adaptive Noise (27.25% med)│
│ - Phase 14 Feature Selection  │               │ - Phase 17 Unified Fusion (27.64% med)│
│ - Phase 15 Jerk Derivatives   │               │ - 158-Outage Benchmark Harness        │
└───────────────────────────────┘               └───────────────────────────────────────┘
========================================================================================
```

---

## 3. Detailed Scope Breakdown

### Category A: Production MVP (Fully Implemented & Verified in C++ / Android)
- **`idr-core/`**: Self-contained C++17 library with zero external ML runtimes (no PyTorch, no TFLite, no ONNX). Runs sample-rate agnostically.
- **Transpiled Decision Tables**: 120-tree Huber GBDT speed regressor compiled into static C++ arrays ($R^2 = 0.884$, latency $< 0.02\text{ ms}$).
- **15-State ESKF**: Full error-state covariance propagation, quaternion kinematics, NHC body-velocity damping ($v_y^b \approx 0, v_z^b \approx 0$), and ZUPT standstill clamping.
- **3D HMM Map Matcher**: Fast topological graph projection with discrete elevation layers and $|\Delta h| > 4.0\text{ m}$ elevation barrier.
- **Android Sensor & GNSS Layer**: 100 Hz IMU capture, 1 Hz `GnssStatus.Callback` monitoring real NavIC (`CONSTELLATION_IRNSS`) satellites, and circular bounded queue synchronization.
- **Android Presentation UI**: 12-Screen Compose navigation system with HUD, live trajectory toggles, satellite polar radar, trip analytics, and offline `.idrmap` loading.
- **`idr-cli` Edge Runner**: Command-line binary benchmarked at $>50,000\text{ Hz}$ throughput ($>250\times$ real-time).

### Category B: Research Extensions (Verified in Python `idr_pipeline/`)
- **Phase 13**: High-frequency vibration power spectral density features ($30.15\%$ median drift).
- **Phase 14**: Top-45 feature ranking and selection pruning ($29.42\%$ median drift).
- **Phase 15**: Time-derivative kinematic jerk feature extraction ($28.80\%$ median drift).
- **Phase 16**: Pre-outage GNSS Doppler velocity handover and learned adaptive measurement noise $R_{\text{ai}}$ ($27.25\%$ median drift, 45/158 passes).
- **Phase 17**: Causal kinematic lateral consistency and standstill drift containment ($27.64\%$ median drift, 49/158 passes).

### Category C: Pending & Future Extensions
- **Phase 16/17 C++ Port**: Porting the adaptive $R_{\text{ai}}$ covariance function into `idr-core` with formal bitwise-parity verification.
- **Physical FOG Hardware Integration**: Live streaming from physical fiber-optic gyroscopes (currently validated on high-rate software replays and I2NAV datasets).
- **Dataset-Wide Global Vector Road Coverage**: Bundling pre-sliced binary `.idrmap` graphs for all worldwide territories (currently bundled for showcase regional corridors).
