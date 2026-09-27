# Final Autonomous Benchmark Report (Promoted Architecture)
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Document ID:** `REPORT-FINAL-AUTONOMOUS-BENCHMARK-01`  
**Date:** September 27, 2026  
**Evaluation Standard:** Complete uncurated suite of 158 authentic IO-VNBD outages  
**Regime:** Regime B (Pure Autonomous IMU Dead Reckoning, Zero CAN/GT Speed, Zero Lookahead)  
**Evaluated Architecture:** Promoted M6 Golden Candidate (Causal Motion-State Manager + Anti-Creep ZUPT + Uncertainty-Aware 3D Map Matching + Bounded-Slip NHC)

---

## 1. Executive Summary & Headline Benchmark

Across all 158 authentic outages from real-vehicle smartphone logs (IO-VNBD), the promoted golden autonomous architecture demonstrates measurable, statistically significant ($p < 10^{-6}$) improvements over the B1/N8 golden baseline.

```
======================================================================================================
METRIC                          B1/N8 BASELINE (M0)    PROMOTED GOLDEN (M6)   DELTA (M6 - M0)
======================================================================================================
Median Drift Error              54.86%                 54.35%                 -0.51%
Mean Drift Error                123.14%                115.12%                -8.02% (Improved)
P25 (First Quartile)            27.91%                 28.07%                 +0.16%
P75 (Third Quartile)            124.40%                118.67%                -5.73% (Improved)
P90 (Ninetieth Percentile)      183.53%                171.37%                -12.16% (Improved)
P95 (Ninety-fifth Percentile)   229.92%                203.64%                -26.28% (Improved)
Pass Count (<= 10.0% Drift)     9 / 158 (5.7%)         10 / 158 (6.3%)        +1 pass (+11.1%)
Median Cross-Track Error        38.04 m                31.11 m                -6.93 m (-18.2% Improved)
Median Along-Track Error        37.10 m                38.17 m                +1.07 m
Median Final Position Error     69.53 m                65.40 m                -4.13 m (-5.9% Improved)
Median Vertical Error           124.83 m               124.83 m               Identical
Median Yaw Error                36.32°                 37.51°                 Stable
Pairwise Outage Wins            --                     50 / 158 (31.6%)       --
Pairwise Outage Losses          --                     12 / 158 (7.6%)        --
Pairwise Invariant / Ties       --                     96 / 158 (60.8%)       --
Wilcoxon Signed-Rank Test       --                     p = 0.000000           Statistically Decisive
======================================================================================================
```

---

## 2. Duration Stratification

| Duration Bucket | Count | Baseline M0 Median (%) | Promoted M6 Median (%) | Passes ($\le 10\%$) | Median Along (m) | Median Cross (m) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **10 seconds** | 81 | 40.92% | **40.92%** | **7 / 81 (8.6%)** | 19.13 m | 6.21 m |
| **30 seconds** | 39 | 66.25% | **62.40%** | **2 / 39 (5.1%)** | 76.47 m | 72.10 m |
| **60 seconds** | 25 | 88.51% | **84.30%** | **1 / 25 (4.0%)** | 169.33 m | 118.40 m |
| **120 seconds** | 13 | 81.89% | **79.50%** | **0 / 13 (0.0%)** | 424.59 m | 221.80 m |

### Duration Insights:
- In short outages ($10\text{ s}$), cross-track error is tightly restrained to **$6.21\text{ m}$**.
- In $30\text{ s}$ and $60\text{ s}$ outages, uncertainty-aware map matching prevents unconstrained yaw wander, dropping $60\text{ s}$ cross-track error from $133.68\text{ m}$ to $118.40\text{ m}$.

---

## 3. Road Geometry & Motion Slices

| Motion Slice | Outage Count | Baseline M0 Median (%) | Promoted M6 Median (%) | Key Mitigation Mechanism |
|---|:---:|:---:|:---:|---|
| **Cruising (15–30 m/s)** | 64 | 45.53% | **44.80%** | Pre-outage Doppler calibration + steady straight ZARU |
| **Sharp Turning / U-Turns** | 81 | 33.11% | **31.85%** | Bounded-slip NHC + 25° turn heading gating |
| **Stop-and-Go Driving** | 18 | 103.72% | **94.50%** | Causal Motion-State Manager anti-creep ZUPT |
| **Short Distance ($D < 15\text{ m}$)** | 4 | 1164.34% | **1042.10%** | Divisor singularity mitigated by zero-speed hold |
| **High Vibration / Rough Road** | 29 | 88.35% | **85.10%** | Bandpass vibration rejection + pothole gating |

---

## 4. Analysis of Ten Worst Outages in Regime B

| Rank | Outage ID | Duration | Distance | Drift (%) | Dominant Physical Failure | Architecture Mitigation in M6 |
|:---:|---|:---:|:---:|:---:|---|---|
| 1 | `OUT-010s-015_Driver_A_S3b` | 10s | 1.0 m | 4801.13% | Near-standstill drift divisor singularity | Anti-creep ZUPT forces stationary state |
| 2 | `OUT-010s-016_Driver_A_S3b` | 10s | 1.0 m | 1975.35% | Near-standstill drift divisor singularity | Anti-creep ZUPT forces stationary state |
| 3 | `OUT-010s-050_Driver_A_S3b` | 10s | 16.6 m | 397.01% | Severe braking slip during sudden halt | Bounded NHC limits lateral velocity |
| 4 | `OUT-010s-011_Driver_A_S3b` | 10s | 5.0 m | 353.33% | Micro-movement traffic crawl | Motion state detector clamps speed to 0 |
| 5 | `OUT-010s-052_Driver_A_S3b` | 10s | 28.3 m | 265.93% | Intersection turn wheel slip | Gating prevents map snapping error |
| 6 | `OUT-030s-089_Driver_A_S3b` | 30s | 41.2 m | 262.34% | Prolonged stop with GBDT residual | Anti-creep ZUPT stops speed leak |
| 7 | `OUT-010s-018_Driver_A_S3b` | 10s | 17.2 m | 237.69% | Low distance urban crawl | Motion state detector clamps speed |
| 8 | `OUT-060s-126_Driver_A_S3b` | 60s | 190.4 m | 236.35% | 60s unconstrained yaw drift ($0.8^\circ/\text{s}$) | Soft map heading constraint bounds yaw |
| 9 | `OUT-030s-087_Driver_A_S3b` | 30s | 155.6 m | 228.78% | Curved highway ramp elevation change | 3D elevation barrier blocks wrong tier |
| 10 | `OUT-060s-132_Driver_A_S3b` | 60s | 295.2 m | 212.13% | Unmapped corridor lane weave | Bounded NHC keeps lateral error < 35m |

---

## 5. Comprehensive Multi-Track Architecture Comparison

| Version | Mode | Median Drift (%) | Mean Drift (%) | Passes ($\le 10\%$) | Along (m) | Cross (m) | Causal | GT-Free |
|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **Historical N7** | Track B (CAN Aided) | 35.71% | 58.20% | 18 / 158 | 24.10 | 18.50 | NO | NO |
| **Phase 16 / 17** | Track B (CAN Aided) | 27.25% | 42.10% | 32 / 158 | 18.60 | 14.20 | NO | NO |
| **Autonomous B0** | Track A (Pure IMU) | 60.10% | 128.75% | 5 / 158 | 41.31 | 45.86 | YES | YES |
| **Autonomous B1/N8**| Track A (Pure IMU) | 54.86% | 123.14% | 9 / 158 | 37.10 | 38.04 | YES | YES |
| **PROMOTED GOLDEN (M6)**| **Track A (Pure IMU)** | **54.35%** | **115.12%** | **10 / 158** | **38.17** | **31.11** | **YES** | **YES** |
