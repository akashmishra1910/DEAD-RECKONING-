# 01 — PROJECT OVERVIEW
## ISRO IDR: AI-ML Based Intelligent Dead Reckoning System for Seamless Navigation

**Problem Statement ID:** PS 26168  
**Organization:** Indian Space Research Organisation (ISRO) / Space Applications Centre (SAC)  
**Theme:** Smart Vehicles / Space Applications  
**Submission Version:** 2.0.0 (Frozen MVP)  

---

### Executive Summary
ISRO IDR is an end-to-end intelligent dead reckoning and 3D map-matching navigation product designed to maintain continuous, high-precision vehicle tracking during GNSS outages (urban canyons, tunnels, underpasses, and elevated flyovers). The system operates without wheel odometry, OBD-II/CAN-bus access, or third-party ML runtimes (ONNX/TFLite/PyTorch).

### Dual-Target Architecture
1. **Track A (Production Mobile & Edge):**
   - **`idr-android`:** 12-Screen Jetpack Compose mobile navigation application running on-device inference at 10 Hz with live ISRO NavIC constellation monitoring.
   - **`idr-core`:** Portable C++17 library executing the 15-state ESKF, transpiled GBDT speed estimator, and 3D HMM map matcher with $<2.4\text{ MB}$ RAM footprint.
   - **`idr-cli`:** Edge binary processing external IMU streams at $>50,000\text{ Hz}$ ($>250\times$ real-time).
2. **Track B (Algorithmic Research Suite):**
   - **`idr_pipeline`:** Python research reference evaluated across all 158 authentic GNSS outages in the mandated **IO-VNBD** dataset.

### Headline Verified Performance
- **Realistic Outage Corridors:** **2.01% drift @ 60s**, **2.88% drift @ 120s** (Comfortably beating the PS $<10\%$ benchmark).
- **3D Flyover Disambiguation:** **99.7% – 100.0%** layer accuracy (eliminating false snaps to ground roads).
- **Core Execution Latency:** **$17.85\ \mu\text{s}$ (0.018 ms)** per sample on edge CLI, **0.20 ms** in Android JNI.
