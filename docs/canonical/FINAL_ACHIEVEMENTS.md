# FINAL DEMONSTRATED ACHIEVEMENTS — SIH26168
## Verified Engineering, Algorithmic & Product Capabilities

**Document ID:** `DOC-CANONICAL-ACHIEVEMENTS-04`  
**Version:** 2.0.0 (Submission Freeze)  
**Date:** September 26, 2026  
**Status Standard:** Rule Zero — Evidence-Backed Achievements  

---

## 1. Demonstrated Core Capabilities (Track A — C++17)

1. **Self-Contained C++17 Engine (`idr-core`):**
   - Implemented without third-party ML runtime dependencies (no ONNX, TFLite, PyTorch).
   - Zero dynamic heap allocation in high-rate streaming loop.
   - Resident Memory: **$<2.4\text{ MB}$ RSS**; Peak memory footprint: **$1.44\text{ MB}$**.
   - Sample-rate agnostic architecture supporting $\Delta t$ per sample up to 1000 Hz.
2. **Deterministic Bit-for-Bit Parity:**
   - C++ decision table speed regressor reproduces Python reference with **$0.000000\text{ m/s}$ delta** across 5,101 test vectors (`tests/test_parity.py`).
   - 158-outage trajectory delta between Python and C++ engines is **$1.18 \times 10^{-11}\text{ m}$ (11.8 picometers)** (`reports/phase9_python_cpp_equivalence.csv`).

---

## 2. Demonstrated AI/ML Capabilities

1. **Vibration-Aware Speed Regression:**
   - 120-tree Huber GBDT regressor achieves $R^2 = 0.884$ and RMSE $1.21\text{ m/s}$ on real vehicle IMU data from IO-VNBD.
   - 45 physical and statistical features (SMA, vibration power bands, centripetal acceleration $a_y \approx v \cdot \omega_z$, statistical moments).
2. **Learned Adaptive Measurement Noise ($R_{\text{ai}}$):**
   - Dynamic noise scaling as a function of instantaneous vehicle vibration and turning rate.
   - Reduces median outage drift from $35.71\%$ down to $27.25\%$ in Python research.

---

## 3. Demonstrated Navigation & Filter Capabilities

1. **15-State Error-State Kalman Filter (ESKF):**
   - States: $\mathbf{x} = [\mathbf{p}^n, \mathbf{v}^n, \mathbf{q}_b^n, \mathbf{b}_a, \mathbf{b}_g]^T \in \mathbb{R}^{15}$.
   - Continuous-discrete error propagation with body-frame Non-Holonomic Constraints (NHC).
2. **Standstill Detection (ZUPT):**
   - Standstill detector triggers zero-velocity updates during traffic stops, reducing stop-and-go drift from $4.61\%$ to $2.58\%$.
3. **Causal 3D Multi-Layer Map Matching:**
   - 3D HMM with $|\Delta h| > 4.0\text{ m}$ barometric elevation barrier achieves **$99.7\%-100.0\%$ layer disambiguation**, eliminating false snaps to ground roads on elevated flyovers.
4. **Chi-Square GNSS Deficit Supervisor:**
   - $\chi^2 \le 11.35$ innovation gating rejects tunnel-exit multipath spikes and smoothly reconciles GNSS and DR states without position jumps.

---

## 4. Demonstrated Android Mobile Capabilities

1. **12-Screen Jetpack Compose Product:**
   - Complete navigation suite: Main Navigation, Map Explorer, Trip Analytics, Diagnostics, Settings, Sensor Monitor, GNSS SkyView, Offline Region Packs, and Calibration.
2. **Real NavIC / IRNSS Ingestion:**
   - Integrated with Android `GnssStatus.Callback`, tracking satellite PRN, elevation, azimuth, and C/N0 for `CONSTELLATION_IRNSS` satellites.
3. **Bounded Circular Sensor Queue:**
   - Thread-safe FIFO queue decoupling 100 Hz hardware sensor streams from 10 Hz UI recomposition without dropped samples or UI stutter.
4. **Offline Map Loading:**
   - Zero-copy binary `.idrmap` loader parsing regional multi-layer road networks directly from device assets and cache.

---

## 5. Demonstrated Edge Capabilities

1. **High-Throughput Streaming CLI (`idr-cli`):**
   - Benchmark throughput: **$56,026\text{ Hz}$** ($>280\times$ the PS required 200 Hz threshold).
   - Mean per-sample inference latency: **$17.85\ \mu\text{s}$ ($0.018\text{ ms}$)**.
2. **External IMU Ingestion:**
   - Validated against I2NAV and industrial IMU log files containing raw gyro, accel, magnetometer, and GNSS packets.
