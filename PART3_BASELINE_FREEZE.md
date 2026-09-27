# PART 3 BASELINE FREEZE SPECIFICATION
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Immutable Control Baseline for Accuracy Recovery & Empirical Convergence

**Document Version:** 3.0.0  
**Freeze Date:** September 27, 2026  
**Status:** IMMUTABLE CONTROL BASELINE  
**Baseline Model ID:** `P2-M7` (Part 2 Full Integrated System)  
**Evaluator Script:** `scripts/run_part2_controlled_ablations.py`  
**Dataset Version:** Real IO-VNBD v1.0 (5 Drives, 158 Standardized Outages)  
**Python Environment:** Python 3.14.3, NumPy 2.4.0, SciPy 1.15.0, Scikit-learn 1.6.1  
**Target Hardware:** Apple Silicon / Standard ARM64 / x86_64  

---

## 1. System & Environment State Hashes

| Property | Value / Hash |
|---|---|
| Repository Git Branch | `main` |
| Active Evaluator Script | `scripts/run_part2_controlled_ablations.py` |
| GBDT Model Artifact | `models/gbdt_speed_regressor.pkl` |
| GBDT Scaler Artifact | `models/gbdt_speed_scaler.pkl` |
| Top-45 Feature Indices | `models/gbdt_top45_indices.npy` |
| Outage Catalog Hash | `outage_catalog_158_iovnbd_v1` |
| Dynamic Oracle Firewall | Active (`tests/test_no_ground_truth_in_inference.py`) |
| Causality Verification | Active (`tests/test_strict_causality.py`) |
| Random Seed | 42 |

---

## 2. Frozen Baseline Parameter Configuration

| Parameter | Value | Description |
|---|---|---|
| Sampling Rate ($f_s$) | 100 Hz | Raw IMU rate |
| Inference Step ($\Delta t$) | 0.10 s | 10 Hz Dead-reckoning update cycle |
| GBDT Trees / Depth / LR | 120 / 5 / 0.08 | Huber loss speed regressor |
| Window Size | 10 samples (1.0 s) | Causal trailing feature window |
| Pre-Outage Doppler Window | 3.0 s | GNSS Doppler observability handover prior to outage |
| NHC Lateral Slip Clamp ($R_{\text{max}}$) | $0.16\text{ m}^2/\text{s}^2$ | Bounded-slip constraint ($0.40\text{ m/s}$) |
| ZARU Gyro Threshold | $0.005\text{ rad/s}$ | In-flight straight-line gyro zero rate update |
| 3D Elevation Barrier | $4.0\text{ m}$ | Rejection threshold for non-coplanar road decks |
| Map Confidence Threshold | $0.35$ | Minimum Bayesian confidence for soft Kalman updates |
| High Confidence Cross Cov ($R_{\text{cross}}$) | $1.75^2 = 3.0625\text{ m}^2$ | Soft cross-track update for $C_{\text{map}} \ge 0.70$ |
| Med Confidence Cross Cov ($R_{\text{cross}}$) | $3.50^2 = 12.250\text{ m}^2$ | Soft cross-track update for $0.35 \le C_{\text{map}} < 0.70$ |
| Max Pos Cov Trace Limit | $1000.0\text{ m}^2$ | Upper bound for covariance consistency |

---

## 3. Official 158-Outage Frozen Baseline Performance

The table below defines the immutable control performance. Every Part 3 experiment must be compared directly against this exact baseline.

### 3.1 Primary Population Metrics (158 Uncurated Outages)
| Metric | Frozen Baseline Value | Target Benchmark |
|---|---:|---:|
| **Sample Count ($N$)** | **158** | 158 |
| **Median Drift (%)** | **59.63%** | $\le 10.0\%$ |
| **Mean Drift (%)** | **116.47%** | - |
| **P25 Drift (%)** | **31.92%** | - |
| **P75 Drift (%)** | **115.42%** | - |
| **P90 Drift (%)** | **164.24%** | - |
| **P95 Drift (%)** | **207.26%** | - |
| **$\le 10.0\%$ Pass Count** | **7 / 158** | 158 / 158 |
| **Median Cross-Track Error** | **36.81 m** | - |
| **Median Along-Track Error** | **41.32 m** | - |
| **Median Vertical Error** | **1.06 m** | $\le 2.0\text{ m}$ |
| **Median Yaw Error** | **43.53°** | $\le 5.0^\circ$ |

---

## 4. Stratified Baseline Performance Breakdown

### 4.1 Sliced by Outage Duration
| Duration | Outages ($N$) | Median Drift (%) | Mean Drift (%) | Median Pos Error (m) | Median Cross-Track (m) | Median Along-Track (m) |
|---|---|---|---|---|---|---|
| **10 s** | 81 | **41.85%** | 145.85% | 29.28 m | **10.39 m** | 19.48 m |
| **30 s** | 39 | **53.06%** | 79.86% | 120.95 m | 80.06 m | 79.35 m |
| **60 s** | 25 | 100.26% | 100.89% | 369.22 m | 185.19 m | 206.07 m |
| **120 s** | 13 | 73.93% | 73.15% | 539.87 m | 430.81 m | 315.96 m |

### 4.2 Sliced by Motion Dynamics
| Regime | Criteria | Count ($N$) | Median Drift (%) | Mean Drift (%) | Median Pos Error (m) |
|---|---|---|---|---|---|
| **Dynamic Moving** | $d_{\text{travelled}} > 50\text{ m}$ | 133 | **51.67%** | **68.15%** | 96.07 m |
| **Standstill / Short** | $d_{\text{travelled}} \le 50\text{ m}$ | 25 | **127.86%** | **373.49%** | **39.11 m** |

### 4.3 Sliced by Drive
| Drive Name | Outages ($N$) | Median Drift (%) | Mean Drift (%) | Passes ($\le 10\%$) | Median Cross (m) | Median Along (m) | Median Yaw Err (°) |
|---|---|---|---|---|---|---|---|
| `Driver_A_S3b` | 110 | **84.53%** | 151.18% | 2 / 110 | 46.82 m | 49.43 m | **69.29°** |
| `Vta01b` | 15 | **31.97%** | 34.73% | 3 / 15 | 32.57 m | 39.84 m | **14.73°** |
| `Vta03` | 2 | **74.98%** | 74.98% | 0 / 2 | 1.60 m | 10.58 m | **110.19°** |
| `Vta04` | 31 | **28.34%** | 35.52% | 2 / 31 | 23.51 m | 27.67 m | **16.27°** |

> [!NOTE]
> **Key Baseline Finding:**  
> In `Vta04` and `Vta01b`, where gyro drift was low ($14\text{--}16^\circ$ heading error), median drift was **$28.34\%\text{ and }31.97\%$**. In `Driver_A_S3b`, unobserved gyro bias accumulated to **$69.29^\circ$ of yaw error**, rotating velocity vectors completely off-course and causing $84.53\%$ drift. **Heading observability is the #1 physical error bottleneck.**
