# Automated Cross-Platform Parity Validation Report
## Python 3.14 Authoritative Pipeline vs. Standalone C++17 Core Engine

**Document ID:** `REPORT-PARITY-VALIDATION-01`  
**Date:** September 27, 2026  
**Commit:** `af1b68fb` + C++ Engine ZARU Integration  
**Execution Environment:** macOS Darwin (Apple Silicon clang-1500 C++17 / Python 3.14.3)  
**Target:** Floating-Point Parity Across All 8 Mathematical Navigation Subsystems  

---

## 1. Summary Parity Scorecard

Every core mathematical function in the dead reckoning pipeline was cross-validated between `idr_pipeline/` and `idr-core/` using identical golden test vectors:

| Subsystem Component | Python Reference | C++17 Core Implementation | Observed Delta ($\Delta$) | Pass Criteria | Status |
|---|---|---|---|---|:---:|
| **Top-45 Feature Extractor** | `dataset_loader.py` | `SlidingWindowFeatureExtractor` | $3.55 \times 10^{-15}$ | $< 10^{-6}$ | **PASSED** |
| **GBDT Speed Model** | `gbdt_model.predict` | `GBDTSpeedModel::predict` | $0.000000\text{ m/s}$ ($< 10^{-9}$) | $< 10^{-6}\text{ m/s}$ | **PASSED** |
| **Vibration Bandpass & Pothole** | `VibrationPotholeFilter` | `VibrationFilter::process` | $1.78 \times 10^{-15}\text{ m/s}^2$ | $< 10^{-6}$ | **PASSED** |
| **SO(3) Manifold Exp/Log** | `so3_exp` / `so3_log` | `SO3::exp` / `SO3::log` | $1.11 \times 10^{-16}\text{ rad}$ | $< 10^{-10}$ | **PASSED** |
| **ESKF State Propagation** | `ESKF.predict` | `ErrorStateKalmanFilter::predict`| $1.42 \times 10^{-14}\text{ m}$ | $< 10^{-6}\text{ m}$ | **PASSED** |
| **Bounded-Slip NHC Update** | `update_nhc` | `ErrorStateKalmanFilter::update_nhc`| $4.44 \times 10^{-16}\text{ m/s}$ | $< 10^{-6}\text{ m/s}$ | **PASSED** |
| **In-Flight Kalman ZARU** | `_apply_kalman_update` | `ErrorStateKalmanFilter::update_zaru`| $2.22 \times 10^{-16}\text{ rad/s}$ | $< 10^{-6}\text{ rad/s}$ | **PASSED** |
| **3D HMM Road Matcher** | `Causal3DMapMatcher` | `Causal3DMapMatcher::process_step` | 100% Edge Index Match | Bit-exact index | **PASSED** |

---

## 2. Regression Parity Suite Results

### 2.1 CTest Engine Suite (14 / 14 Passed)
```
      Start  1: test_math_so3 ....................   Passed    0.00 sec
      Start  2: test_math_golden_parity ..........   Passed    0.00 sec
      Start  3: test_gbdt_parity .................   Passed    0.03 sec
      Start  4: test_feature_vibration ...........   Passed    0.00 sec
      Start  5: test_alignment ...................   Passed    0.00 sec
      Start  6: test_eskf ........................   Passed    0.00 sec
      Start  7: test_heading_supervisor ..........   Passed    0.00 sec
      Start  8: test_map_matcher .................   Passed    0.00 sec
      Start  9: test_navigation_engine ...........   Passed    0.00 sec
      Start 10: test_c_api .......................   Passed    0.00 sec
      Start 11: test_idrmap_reader ...............   Passed    0.15 sec
      Start 12: test_causal_hmm ..................   Passed    0.01 sec
      Start 13: test_offline_navigation ..........   Passed    0.01 sec
      Start 14: test_edge_engine .................   Passed    0.01 sec
```

### 2.2 Python Parity Test (`test_phase11_6_cpp_reader_parity.py`)
```
============================== 5 passed in 0.42s ===============================
```

---

## 3. High-Rate Edge Engine Profiling (`idr-cli` / `benchmark_tactical_100k`)

- **Dataset:** 100,000 synthetic high-rate IMU samples at $200\text{ Hz}$.
- **Total Execution Time:** $1.193\text{ seconds}$.
- **Sustained Frequency:** **$83,823\text{ Hz}$** ($419\times$ real-time headroom over $200\text{ Hz}$).
- **Mean Step Latency:** **$11.93\ \mu\text{s}$**.
- **Max Step Latency:** **$42.10\ \mu\text{s}$**.
- **Heap Allocations in Hot Loop:** **0 bytes** (100% stack and static allocation).
