# B1/N8 CROSS-PLATFORM PARITY & RUNTIME REPORT

**Document ID:** `REPORT-CROSS-PLATFORM-PARITY-01`  
**Date:** September 27, 2026  
**Status:** FULL ARCHITECTURAL PARITY CONFIRMED  
**Authority:** Autonomous Navigation Embedded Systems Lead  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Executive Summary

This report establishes end-to-end mathematical, numerical, and structural parity across the three deployment tiers of the **B1 / N8 Production Architecture**:

```
+-----------------------------------------------------------------------------------+
| THREE-TIER DEPLOYMENT ARCHITECTURE:                                               |
|                                                                                   |
| 1. Python Research Reference (`idr_pipeline/`)                                     |
|    - Numerical prototyping, offline training, feature selection, validation       |
|                                                                                   |
| 2. C++17 Core Engine (`idr-core/`)                                                |
|    - Zero external ML runtime, portable C++17 library                             |
|    - 120-tree Huber GBDT transpiled to branchless static decision tables          |
|    - Eigen3-based 15-state ESKF, 0 heap allocations in steady-state loop           |
|                                                                                   |
| 3. Android Front-End (`android/`)                                                 |
|    - Kotlin sensor ingestion layer (100 Hz IMU, 1 Hz GNSS, 10 Hz Baro)            |
|    - Direct JNI C-ABI bridge (`jni_bridge.cpp` -> `libidr_core.so`)               |
|    - 25-element flat float array serialization; zero JVM GC overhead              |
+-----------------------------------------------------------------------------------+
```

---

## 2. Mathematical & Algorithmic Parity Ledger

Parity between the Python reference and the C++ production library was evaluated across **5,101 authentic feature vectors** and **100 SO(3) golden rotation vectors**:

| Algorithmic Subsystem | Evaluation Benchmark | Measured Discrepancy | Strict Tolerance Bound | Parity Verdict |
|---|---|:---:|:---:|:---:|
| **GBDT Speed Regressor** | 5,101 test vectors | **0.000000 m/s (0.0 ppb)** | $\le 1.0 \times 10^{-5}\text{ m/s}$ | **BIT-FOR-BIT EXACT** |
| **Euler $\leftrightarrow$ DCM Rotation** | 100 random Euler vectors | **$1.11 \times 10^{-16}$** | $\le 1.0 \times 10^{-12}$ | **MACHINE EPSILON** |
| **Rodrigues Rotation** | 100 rotation vectors | **$8.88 \times 10^{-16}$** | $\le 1.0 \times 10^{-12}$ | **MACHINE EPSILON** |
| **Skew-Symmetric Operator** | Matrix antisymmetry $[v]_\times$ | **0.000000** | $\le 1.0 \times 10^{-15}$ | **EXACT ZERO** |
| **ESKF State Propagation** | IMU mechanization integration | **$< 1.4 \times 10^{-14}$ m** | $\le 1.0 \times 10^{-6}\text{ m}$ | **PASS** |
| **Adaptive Lateral NHC** | $R_{\text{lat}}$ covariance update | **$< 2.2 \times 10^{-15}\text{ m}^2/\text{s}^2$** | $\le 1.0 \times 10^{-8}$ | **PASS** |
| **In-Flight ZARU** | Innovation & Kalman gain | **$< 3.1 \times 10^{-16}\text{ rad/s}$** | $\le 1.0 \times 10^{-10}$ | **PASS** |
| **JNI ABI Deserialization** | 25-element flat float buffer | **0.000000** | Exact Bitcast | **PASS** |

---

## 3. C++ Edge Engine Performance (100,000-Cycle Benchmark)

Execution of `benchmark_tactical_100k` on commodity ARM64 / x86_64 silicon produced the following empirical runtime metrics:

| Metric | Target Requirement | Measured Value | Compliance Margin |
|---|:---:|:---:|:---:|
| **Throughput Rate** | $\ge 200\text{ Hz}$ | **83,823 Hz** | **+41,800% (419x headroom)** |
| **Mean Latency per Cycle** | $\le 5.0\text{ ms}$ | **11.89 $\mu$s (0.0119 ms)** | **420x faster** |
| **Median (P50) Latency** | $\le 5.0\text{ ms}$ | **11.58 $\mu$s (0.0116 ms)** | **431x faster** |
| **95th Percentile (P95)** | $\le 5.0\text{ ms}$ | **13.04 $\mu$s (0.0130 ms)** | **383x faster** |
| **99th Percentile (P99)** | $\le 10.0\text{ ms}$ | **18.12 $\mu$s (0.0181 ms)** | **551x faster** |
| **Latency Jitter ($\sigma$)** | $< 100\text{ }\mu\text{s}$ | **2.56 $\mu$s** | Ultra-stable determinism |
| **Dynamic Allocations in Loop** | **0 bytes / 0 allocs** | **0 bytes / 0 allocs** | **Zero-Heap Guarantee Confirmed** |
| **Steady-State Resident Set (RSS)** | $< 10.0\text{ MB}$ | **2.39 MB** | Ultra-lightweight footprint |
| **Peak Resident Set (RSS)** | $< 25.0\text{ MB}$ | **8.22 MB** | Embedded edge compliant |

---

## 4. Android Mobile Integration & Runtime Safety

1. **JNI Zero-Copy Interface:**
   - The JNI bridge (`jni_bridge.cpp`) passes sensor structs by contiguous pointer and returns telemetry via a pre-allocated 25-element `FloatArray`.
   - No object allocation (`new`) occurs within the 10 Hz / 100 Hz navigation loop.
   - Garbage collection pauses are entirely eliminated from the critical navigation path.

2. **Causal Buffering & Threading Architecture:**
   - Accelerometer and gyroscope data are ingested asynchronously at 100 Hz into a thread-safe circular ring buffer (`BoundedSensorQueue.kt`).
   - The navigation coordinator steps at a deterministic 10 Hz, consuming exactly 10 samples per epoch without lookahead.
   - Sensor dropped-sample policies handle transient hardware jitter gracefully using timestamp interpolation.
