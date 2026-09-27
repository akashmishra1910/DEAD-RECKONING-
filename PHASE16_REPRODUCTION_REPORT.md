# PHASE 16 REPRODUCTION & CAUSALITY AUDIT REPORT

**Document ID:** `REPORT-PHASE16-REPRO-01`  
**Date:** September 27, 2026  
**Status:** FULLY REPRODUCED WITH ORACLE LEAKAGE AUDIT  
**Author:** Autonomous Navigation Engineering Team  
**Script Path:** [`scripts/run_phase16_final_benchmark.py`](file:///Users/akashmishra/SIH%20MOdel/scripts/run_phase16_final_benchmark.py)  
**Evaluation Standard:** 158 Authentic IO-VNBD Naturalistic Outages  
**Problem Statement:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Executive Summary

Phase 16 introduced the **GNSS Doppler + ESKF Observability Upgrade**, comprising:
1. **Pre-Outage GNSS Doppler Velocity & ESKF Bias Handover (3.0s window):**
   Fuses authentic smartphone GNSS Doppler velocity (`GPS SPEED` and `GPS ORIENTATION`) preceding outage onset ($t \in [t_0 - 3.0\text{s}, t_0]$) to causally observe accelerometer bias $b_a$, gyroscope bias $b_g$, and converge covariance $P[6:15, 6:15]$.
2. **Bounded-Slip Adaptive Lateral NHC:**
   Clamps the maximum lateral measurement covariance to $R_{\text{max}} = (0.40\text{ m/s})^2 = 0.16\text{ m}^2/\text{s}^2$ (compared to $R_{\text{max}} = 2.25\text{ m}^2/\text{s}^2$ in N7).
3. **Causal In-Flight ESKF Kalman ZARU:**
   Applies discrete Kalman error-state zero angular rate updates on yaw bias $b_g[2]$ when stable rectilinear motion is detected ($\lvert \omega_z \rvert < 0.005\text{ rad/s}$, $\lvert a_y \rvert < 0.15\text{ m/s}^2$, $v_{\text{fwd}} > 4.0\text{ m/s}$).
4. **Adaptive Multi-Hypothesis Soft Road Heading Fusion:**
   Tightens heading measurement noise to $r_{\text{yaw}} = 0.050\text{ rad}$ ($2.86^\circ$) when map candidate confidence exceeds $80\%$.

---

## 2. Benchmark Reproduction Results

Execution of `scripts/run_phase16_final_benchmark.py` across all 158 authentic outages yielded exact numerical reproduction:

| Metric | B0 Production Baseline | Phase 16 Reproduced | Delta ($\Delta$) | Impact Direction |
|---|:---:|:---:|:---:|:---:|
| **Overall Median Drift** | **35.71%** | **27.25%** | **-8.46%** | Significant Improvement |
| **Overall Mean Drift** | **45.86%** | **42.91%** | **-2.95%** | Improvement |
| **Pass Count ($\le 10\%$)** | **36 / 158** | **45 / 158** | **+9 passes** | +25.0% relative passes |
| **Pass Rate ($\le 10\%$)** | **22.78%** | **28.48%** | **+5.70%** | Improvement |
| **Median Along-Track Error** | 16.60 m | **15.88 m** | -0.72 m | Slight Reduction |
| **Median Cross-Track Error** | 28.77 m | **20.47 m** | **-8.30 m** | Major 28.8% Reduction |

---

## 3. Duration Stratification Breakdown

| Outage Duration | Sample Count ($N$) | Phase 16 Median Drift | B0 Baseline Median | Passes ($\le 10\%$) | Along Error (m) | Cross Error (m) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **10 seconds** | 81 | **13.29%** | 20.10% | 34 / 81 (42.0%) | 4.23 m | 4.40 m |
| **30 seconds** | 39 | **34.79%** | 36.24% | 9 / 39 (23.1%) | 36.02 m | 37.28 m |
| **60 seconds** | 25 | **56.03%** | 59.35% | 2 / 25 (8.0%) | 83.78 m | 113.11 m |
| **120 seconds** | 13 | **61.90%** | 51.24% | 0 / 13 (0.0%) | 319.64 m | 153.64 m |

---

## 4. Road Geometry Breakdown

| Dynamics Slice | Outage Count | Phase 16 Median Drift | B0 Median Drift | Passes ($\le 10\%$) |
|---|:---:|:---:|:---:|:---:|
| **Straight Corridors** | 12 | **29.90%** | 29.81% | 4 / 12 (33.3%) |
| **Curved Corridors** | 146 | **27.25%** | 35.71% | 41 / 146 (28.1%) |

---

## 5. Ten Worst Outages Under Phase 16

| Rank | Outage Identifier | Duration | Distance | Phase 16 Drift (%) | Along Error (m) | Cross Error (m) |
|:---:|---|:---:|:---:|:---:|:---:|:---:|
| 1 | `OUT-060s-132_Driver_A_S3b` | 60 s | 295.2 m | **164.43%** | 485.2 m | 11.7 m |
| 2 | `OUT-060s-126_Driver_A_S3b` | 60 s | 190.4 m | **155.83%** | 274.3 m | 113.1 m |
| 3 | `OUT-060s-135_Driver_A_S3b` | 60 s | 311.3 m | **154.50%** | 175.0 m | 448.0 m |
| 4 | `OUT-010s-017_Driver_A_S3b` | 10 s | 31.2 m | **143.67%** | 43.0 m | 12.8 m |
| 5 | `OUT-030s-102_Driver_A_S3b` | 30 s | 184.6 m | **140.93%** | 238.4 m | 104.3 m |
| 6 | `OUT-030s-089_Driver_A_S3b` | 30 s | 41.2 m | **139.58%** | 45.1 m | 35.7 m |
| 7 | `OUT-060s-129_Driver_A_S3b` | 60 s | 374.1 m | **137.83%** | 159.4 m | 490.4 m |
| 8 | `OUT-030s-092_Driver_A_S3b` | 30 s | 204.5 m | **133.61%** | 269.2 m | 46.9 m |
| 9 | `OUT-030s-087_Driver_A_S3b` | 30 s | 155.6 m | **131.39%** | 204.1 m | 11.2 m |
| 10 | `OUT-060s-136_Driver_A_S3b` | 60 s | 287.7 m | **127.17%** | 304.9 m | 202.2 m |

---

## 6. Causality & Oracle Leakage Audit

### 6.1 Critical Finding: Longitudinal Speed Oracle Leakage
An audit of `scripts/run_phase16_final_benchmark.py` reveals the following code in the GNSS-denied propagation loop (Line 214):
```python
# GNSS-Denied Outage Navigation (strictly causal from i0 to i1)
for k in range(i0, i1):
  ...
  ai_spd = float(traj.gt_speed[k])  # <--- CRITICAL ORACLE LEAKAGE
```
**Finding:** Ground truth CAN bus / OBD-II vehicle speed (`traj.gt_speed`) was used as the speed input during the outage loop. While this practice originated in earlier research benchmarks (Phase 10c / N0–N7), it constitutes an oracle velocity leakage for an autonomous Dead Reckoning system that is constrained to use IMU data only.

### 6.2 Causality of Pre-Outage Doppler Observability Handover
- The function `estimate_preoutage_observability(traj, gnss_v, i0, pre_window_s=3.0)` operates strictly on $k \in [i_0 - 30, i_0]$.
- During this window, the vehicle has valid, uncorrupted GNSS reception.
- Fusing GNSS Doppler velocity (`GPS SPEED` and `GPS ORIENTATION`) to observe sensor biases prior to an outage is **100% causal and physically deployable in real vehicles**.

### 6.3 True Autonomous Performance (Without Oracle Speed)
When evaluated without `gt_speed` using the production Top-45 Huber GBDT model:
- `Auton_B0_N7`: Median drift = **65.81%**, Mean = **129.47%**
- `Auton_Phase16`: Median drift = **57.00%**, Mean = **124.97%**
- **Conclusion:** The Phase 16 algorithmic improvements (pre-outage Doppler handover, bounded-slip NHC, and in-flight ZARU) are genuine filter advancements that reduce autonomous drift by **8.81%** in absolute terms (and reduce lateral error from 47.12 m to 40.12 m). However, neither configuration meets the ISRO PS <10% benchmark autonomously without external velocity aiding.
