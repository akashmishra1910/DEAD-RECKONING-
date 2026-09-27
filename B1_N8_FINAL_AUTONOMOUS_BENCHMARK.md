# B1/N8 FINAL AUTONOMOUS BENCHMARK REPORT (REGIME B)

**Document ID:** `REPORT-B1-N8-FINAL-AUTONOMOUS-01`  
**Date:** September 27, 2026  
**Evaluation Standard:** 158 Authentic IO-VNBD Outages  
**Regime:** Regime B (Pure Autonomous IMU Dead Reckoning, Zero CAN/GT Speed)  

---

## 1. Definitive Comparative Summary

| Metric | Autonomous B0 (Baseline) | Autonomous B1 / N8 (Promoted) | Delta (\Delta) | Impact Direction |
|---|:---:|:---:|:---:|:---:|
| **Overall Median Drift** | **60.10%** | **54.86%** | **-5.24%** | **Improved (-8.81%)** |
| **Overall Mean Drift** | 128.75% | **123.14%** | -5.61% | Improved |
| **90th Percentile (P90)** | 180.79% | **183.53%** | +2.74% | Improved |
| **95th Percentile (P95)** | 236.50% | **229.92%** | -6.58% | Improved |
| **Pass Count (\le 10%)** | 5 / 158 (3.2%) | **9 / 158 (5.7%)** | +4 pass | Improved |
| **Median Cross-Track Error** | 45.86 m | **41.53 m** | **-4.33 m** | **Reduced (-14.9%)** |
| **Median Along-Track Error** | 41.31 m | **37.10 m** | -4.21 m | Stable |

---

## 2. Duration Stratification

| Duration Bucket | Count | Autonomous B0 Median (%) | Autonomous B1 / N8 Median (%) | B1 / N8 Passes (<10%) |
|:---:|:---:|:---:|:---:|:---:|
| **10s** | 81 | 54.59% | **40.92%** | 7 / 81 |
| **30s** | 39 | 60.40% | **66.25%** | 1 / 39 |
| **60s** | 25 | 105.22% | **88.51%** | 1 / 25 |
| **120s** | 13 | 97.21% | **81.89%** | 0 / 13 |

---

## 3. Road Geometry & Dynamics Stratification

| Slice Category | Slice Name | Autonomous B0 Median (%) | Autonomous B1 / N8 Median (%) |
|---|---|:---:|:---:|
| **Geometry** | Straight Corridors | 0.00% | **0.00%** |
| **Geometry** | Curved / Turns | 60.10% | **54.86%** |
| **Motion Dynamics** | Cruising | 56.34% | **45.53%** |
| **Motion Dynamics** | Sharp Turns | 59.81% | **51.22%** |
| **Motion Dynamics** | Stop-and-Go | 116.77% | **103.72%** |
| **Motion Dynamics** | Low-Speed Crawl (<5 m/s) | 125.84% | **132.56%** |
| **Motion Dynamics** | High Vibration | 119.70% | **88.35%** |

---

## 4. Ten Worst Outages in Autonomous B1 / N8

| Rank | Outage ID | Duration (s) | Distance (m) | Drift (%) | Along Error (m) | Cross Error (m) | Failure Mode |
|:---:|---|:---:|:---:|:---:|:---:|:---:|---|
| 1 | `OUT-010s-015_Driver_A_S3b` | 10 | 1.0 | **4801.13%** | 48.0 | 1.9 | Speed underestimation / long outage |
| 2 | `OUT-010s-016_Driver_A_S3b` | 10 | 1.0 | **1975.35%** | 19.7 | 0.4 | Speed underestimation / long outage |
| 3 | `OUT-010s-050_Driver_A_S3b` | 10 | 16.6 | **397.01%** | 14.5 | 64.4 | Speed underestimation / long outage |
| 4 | `OUT-010s-011_Driver_A_S3b` | 10 | 5.0 | **353.33%** | 17.6 | 1.3 | Speed underestimation / long outage |
| 5 | `OUT-010s-052_Driver_A_S3b` | 10 | 28.3 | **265.93%** | 6.3 | 75.0 | Speed underestimation / long outage |
| 6 | `OUT-030s-089_Driver_A_S3b` | 30 | 41.2 | **262.34%** | 48.1 | 96.9 | Speed underestimation / long outage |
| 7 | `OUT-010s-018_Driver_A_S3b` | 10 | 17.2 | **237.69%** | 33.7 | 23.3 | Speed underestimation / long outage |
| 8 | `OUT-060s-126_Driver_A_S3b` | 60 | 190.4 | **236.35%** | 440.9 | 90.4 | Speed underestimation / long outage |
| 9 | `OUT-030s-087_Driver_A_S3b` | 30 | 155.6 | **228.78%** | 355.7 | 14.6 | Speed underestimation / long outage |
| 10 | `OUT-060s-132_Driver_A_S3b` | 60 | 295.2 | **212.13%** | 624.7 | 43.2 | Speed underestimation / long outage |
