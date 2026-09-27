# Empirical Drift Root-Cause Decomposition Report
## ISRO-SAC PS 26168 — 158 Authentic IO-VNBD Outages (B1/N8 Baseline)

**Document ID:** `REPORT-DRIFT-ROOT-CAUSE-01`  
**Date:** September 27, 2026  
**Evaluation Standard:** 158 Authentic IO-VNBD Outages  
**Regime:** Regime B (Pure Autonomous IMU Dead Reckoning, Zero Ground-Truth Speed)  
**Data Source:** `reports/b1_n8_root_cause_decomposition.csv`  

---

## 1. Executive Summary & Macro Error Budget

An exhaustive step-by-step root cause analysis of all 158 authentic outages under the validated B1/N8 baseline isolates the physical and mathematical mechanisms driving dead reckoning drift:

```
========================================================================================
ERROR DIMENSION          MEDIAN MAGNITUDE       PRIMARY PHYSICAL MECHANISM
========================================================================================
Along-Track Error        37.10 m (dominant 58%) Forward velocity integration bias (3.32 m/s)
Cross-Track Error        38.04 m (dominant 42%) Heading/yaw divergence (36.32 deg median)
Vertical Error           124.83 m               Barometer scale/drift in multi-layer roads
Yaw Divergence           36.32 deg (10s: 10.0°) Gyro bias drift rate (0.83 deg/s unobserved)
Forward Velocity Error   3.32 m/s               GBDT scale mismatch under vehicle dynamics
Lateral Velocity Error   0.14 m/s               Well-constrained by bounded NHC (R_max=0.16)
Near-Standstill Outages  1164.34% drift (D<15m) Divisor singularity when denominator D <= 5m
========================================================================================
```

---

## 2. Temporal Evolution & Error Growth with Outage Duration

The trajectory error decomposes distinctly between short, medium, and long outages:

| Outage Duration | Count | Median Drift (%) | Pass Count ($\le 10\%$) | Median Along (m) | Median Cross (m) | Median Yaw Err (deg) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **10 seconds** | 81 | **40.92%** | **7 / 81 (8.6%)** | 19.13 m | 6.21 m | 10.00° |
| **30 seconds** | 39 | **66.25%** | **1 / 39 (2.6%)** | 76.47 m | 87.25 m | 53.88° |
| **60 seconds** | 25 | **88.51%** | **1 / 25 (4.0%)** | 169.33 m | 133.68 m | 71.58° |
| **120 seconds** | 13 | **81.89%** | **0 / 13 (0.0%)** | 424.59 m | 248.97 m | 76.76° |

### Key Diagnostic Discoveries:
1. **Yaw Drift is the Primary Bottleneck in Outages $\ge 30\text{ s}$:**  
   At 10 seconds, yaw error is tightly bounded ($10.0^\circ$), keeping cross-track error to only $6.21\text{ m}$. By 30 seconds, yaw error surges to $53.88^\circ$, which rotates the forward velocity vector into the lateral plane, causing cross-track error to balloon from $6.21\text{ m}$ to $87.25\text{ m}$.
2. **Along-Track Error Accumulates Linearly with Speed Model Bias:**  
   The median forward velocity error of $3.32\text{ m/s}$ integrates linearly with time:
   - At 10s: $3.32\text{ m/s} \times 10\text{ s} \approx 33\text{ m}$ (measured median along: $19.13\text{ m}$)
   - At 60s: $3.32\text{ m/s} \times 60\text{ s} \approx 199\text{ m}$ (measured median along: $169.33\text{ m}$)
   - At 120s: $3.32\text{ m/s} \times 120\text{ s} \approx 398\text{ m}$ (measured median along: $424.59\text{ m}$)
3. **Lateral Velocity is Successfully Controlled:**  
   The bounded-slip NHC successfully suppresses body $v_y$ to a median error of $0.14\text{ m/s}$, proving that lateral cross-track drift is driven almost entirely by **heading orientation error**, not lateral vehicle slip.

---

## 3. Road Dynamics & Motion Slices

| Motion Regime | Count | Median Drift (%) | Dominant Failure Mode |
|---|:---:|:---:|---|
| **Cruising (15–30 m/s)** | 64 | 45.53% | GBDT speed scale bias |
| **Turning / U-Turns** | 81 | 33.11% | Yaw integration under high angular rate |
| **Stop-and-Go Crawl** | 18 | 103.72% | GBDT residual non-zero speed during creep |
| **Short Distance ($D < 15\text{ m}$)** | 4 | 1164.34% | Mathematical singularity in drift % calculation |
| **High Vibration / Rough Road** | 29 | 88.35% | Acceleration noise corrupting attitude leveling |

---

## 4. Priority Ranked Architectural Target Opportunities

Based on this evidence, the most technically justified improvements to test are:

1. **Motion-State Manager & Adaptive ZUPT Gate (Targeting Along-Track & Stop-and-Go):**
   - In stop-and-go driving and low-speed crawl, GBDT outputs residual speeds of $0.3 - 0.8\text{ m/s}$ when the vehicle is stationary or creeping. A causal motion state detector using acceleration variance and gyro magnitude can aggressively force ZUPT, eliminating along-track drift accumulation during stops.
2. **Adaptive Speed Confidence Scaling (Targeting Along-Track Error):**
   - Scale $R_v$ (speed measurement noise) based on vehicle acceleration dynamics and turn rate. When the vehicle is accelerating or turning, GBDT error variance rises; softening $R_v$ prevents false speed updates.
3. **Uncertainty-Aware Soft Road Heading Matcher (Targeting Yaw Error $>30\text{ s}$):**
   - The primary driver of cross-track error beyond 30s is yaw divergence ($53.88^\circ - 76.76^\circ$). Increasing the authority of high-confidence road heading constraints when ESKF yaw uncertainty grows can prevent unconstrained yaw wandering on mapped corridors.
4. **Adaptive NHC Covariance (Targeting Straight vs. Turn Dynamics):**
   - Strengthen lateral NHC during straight cruising while adaptively loosening it during sharp turns.
