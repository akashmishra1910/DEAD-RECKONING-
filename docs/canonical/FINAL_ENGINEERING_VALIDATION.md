# Final Engineering Validation & Release Report
## SIH 2026 PS 26168 — ISRO IDR

### 1. Final Baseline & Architecture
- **Git Branch**: `final-production-hardening`
- **Compiler**: Clang++ 14.0.6 (Android NDK 25.1.8937393) / C++17
- **Android Target**: compileSdk 35, targetSdk 34, minSdk 26, Gradle 8.14.5
- **Architecture**: Portable `idr-core` C++17 library (zero Android dependencies) + Android Kotlin Compose Front-End.

### 2. Integrated vs Rejected Research Ledger
- **Integrated**:
  1. Unified 3D AI Velocity Update $\mathbf{v}_{\text{ai}}^b = [v_{\text{eff}}, 0, 0]^T$ with dynamic covariance $\mathbf{R}_v$.
  2. Autonomous 45-feature extraction & GBDT evaluation inside edge C++ engine.
  3. Dynamic lateral constraint relaxation during turning ($\sigma_{vy} = \sigma_{\text{base}}(1 + c_{\text{lat}} |\omega_{bz}|)$).
  4. Pre-outage Doppler velocity handover into ESKF state.
  5. Multi-tier standstill detector with full 3D ZUPT covariance clamping ($\sigma_{\text{zupt}} = 0.01\,\text{m/s}$).
- **Rejected (Non-Causal / Prohibited)**:
  1. Rauch-Tung-Striebel (RTS) bidirectional smoother (violates real-time causality).
  2. Oracle ground-truth velocity / position injection.
  3. Artificial map snapping.

### 3. Authentic Benchmark Truth (IO-VNBD 158 Real Outages)
- **Median Drift Rate**: **27.94%** (76.81 m mean error across full dataset)
- **C0 Baseline**: 76.34%
- **Previous N7**: 35.71%
- **Pass Count (<10% Drift)**: 34 / 158 authentic outages
- **P95 Drift**: 68.20%

### 4. Edge Runtime Performance
- **Throughput**: **>50,000 Hz** (>50 kHz)
- **Per-Cycle Latency**: **<20 μs** (<0.020 ms)
- **RAM Footprint**: **<2.4 MB RSS**
- **Dynamic Allocations in Tactical Loop**: **0 bytes** (100% deterministic pre-allocated state)

### 5. Test Suite Verification
- **Python Pytest Suite**: **266 Passed, 0 Failed, 6 Skipped**
- **Android Gradle Unit Tests**: **7 Passed, 0 Failed**
- **C++ idr-core Tests**: **All 62 targets compiled and passed**
- **Total Tests**: **335 Tests**

### 6. Release Artifacts
- **Android APK**: `android/app/build/outputs/apk/debug/app-debug.apk`
- **APK SHA-256**: `5A180A499AFB2967B03AC4969715E943E32210D75CC6423EB938A06AE0E44D6F`
- **Core Engine**: `idr-core/build/libidr_core.a` & `libidr_core_shared.so`
