# PHASE 16/17 PRODUCTION MIGRATION BASELINE (N7 / B0)

**Document ID:** `BASELINE-N7-B0-AUTH-01`  
**Date:** September 27, 2026  
**Status:** AUTHORITATIVE FROZEN BASELINE  
**Author:** Autonomous Navigation Engineering Team  
**Evaluation Standard:** 158 Authentic Naturalistic IO-VNBD Outages  
**Problem Statement:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Executive Summary & Baseline Identity

This document defines the authoritative, frozen **N7 / B0 Production Baseline** against which all Phase 16, Phase 17, and subsequent research advancements must be evaluated.

```
+-----------------------------------------------------------------------------------+
| BASELINE IDENTITY: B0 (N7 Production Architecture)                                |
| - Filter: 15-State Error-State Kalman Filter (ESKF)                               |
| - Alignment: In-Vehicle Gravity Leveling + PCA Yaw Alignment                      |
| - Vibration: 4th-Order Butterworth + Engine Idle Frequency Notch Filter           |
| - NHC Formulation: Dynamic Adaptive NHC with 1st-Order Low-Pass Filter            |
|   R_lat(t) = 0.70 * R_lat(t-1) + 0.30 * clip(R_target, R0, R_max), R_max = 2.25  |
| - Heading Aiding: Soft Multi-Hypothesis OSM Graph Road Heading Fusion (r = 0.087) |
| - ZUPT: Speed threshold (<0.2 m/s) + engine idle vibration gating                 |
+-----------------------------------------------------------------------------------+
```

---

## 2. Frozen Baseline Quantitative Metrics

The evaluation encompasses all **158 authentic outages** across all 5 naturalistic IO-VNBD drives (`Driver_A_S3b`, `Vta01b`, `Vta03`, `Vta04`, `Vta05`), with durations ranging from 10s to 120s and distances up to 860 meters.

| Metric | B0 Frozen Baseline Value | Unit / Standard |
|---|:---:|---|
| **Total Authentic Outages** | **158** | Complete naturalistic dataset |
| **Overall Median Drift** | **35.71%** | 50th percentile across all 158 outages |
| **Overall Mean Drift** | **45.86%** | Arithmetic mean across all 158 outages |
| **90th Percentile Drift (P90)** | **105.58%** | 90th percentile drift |
| **95th Percentile Drift (P95)** | **133.71%** | 95th percentile drift |
| **Maximum (Worst) Drift** | **173.20%** | Maximum observed outage drift |
| **Pass Count ($\le 10.0\%$ Drift)** | **36 / 158** | Exact ISRO PS threshold |
| **Pass Rate ($\le 10.0\%$ Drift)** | **22.78%** | Percentage meeting <10% bound |
| **Median Along-Track Error** | **16.60 m** | Longitudinal position error |
| **Median Cross-Track Error** | **28.77 m** | Lateral position error |

---

## 3. Duration-Stratified Breakdown

| Outage Duration | Sample Count ($N$) | Median Drift (%) | Mean Drift (%) | Passes ($\le 10\%$) | Pass Rate (%) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **10 seconds** | 81 | **20.10%** | 32.77% | 27 / 81 | 33.33% |
| **30 seconds** | 39 | **36.24%** | 53.36% | 6 / 39 | 15.38% |
| **60 seconds** | 25 | **59.35%** | 68.27% | 2 / 25 | 8.00% |
| **120 seconds** | 13 | **51.24%** | 61.86% | 1 / 13 | 7.69% |

---

## 4. Road Geometry & Dynamics Stratification

| Geometry Slice | Sample Count ($N$) | Median Drift (%) | Mean Drift (%) | Passes ($\le 10\%$) | Pass Rate (%) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Straight Corridors** | 12 | **29.81%** | 43.25% | 4 / 12 | 33.33% |
| **Curved / Dynamic Turns** | 146 | **35.71%** | 46.08% | 32 / 146 | 21.92% |

---

## 5. Ten Worst Outages in Baseline B0

These 10 catastrophic outages represent the primary failure modes of N7 (lateral unobservability in sharp bends and prolonged dead reckoning without heading bounds):

| Rank | Outage Identifier | Drive Name | Duration (s) | Distance (m) | B0 Drift (%) | Primary Failure Mechanism |
|:---:|---|---|:---:|:---:|:---:|---|
| 1 | `OUT-030s-102_Driver_A_S3b` | Driver_A_S3b | 30.0 | 184.62 | **173.20%** | Multi-lane turn lateral breakaway |
| 2 | `OUT-060s-126_Driver_A_S3b` | Driver_A_S3b | 60.0 | 190.43 | **156.73%** | Extended curve heading divergence |
| 3 | `OUT-060s-135_Driver_A_S3b` | Driver_A_S3b | 60.0 | 311.27 | **155.79%** | Cross-track error accumulation |
| 4 | `OUT-010s-017_Driver_A_S3b` | Driver_A_S3b | 10.0 | 31.24 | **154.35%** | Low-speed turn-in slip |
| 5 | `OUT-060s-132_Driver_A_S3b` | Driver_A_S3b | 60.0 | 295.18 | **144.55%** | Gyro bias random walk |
| 6 | `OUT-030s-089_Driver_A_S3b` | Driver_A_S3b | 30.0 | 41.20 | **140.82%** | Stop-and-go heading creep |
| 7 | `OUT-060s-129_Driver_A_S3b` | Driver_A_S3b | 60.0 | 374.12 | **140.23%** | Large radius curve map mismatch |
| 8 | `OUT-030s-092_Driver_A_S3b` | Driver_A_S3b | 30.0 | 204.50 | **134.46%** | Dynamic overtaking maneuver |
| 9 | `OUT-030s-087_Driver_A_S3b` | Driver_A_S3b | 30.0 | 155.60 | **133.58%** | Roundabout entry heading divergence |
| 10 | `OUT-060s-136_Driver_A_S3b` | Driver_A_S3b | 60.0 | 287.70 | **127.21%** | Prolonged 60s outage bias drift |

---

## 6. Mathematical Formulation of N7 NHC

In N7, the lateral measurement covariance $R_{\text{lat}}$ adapts based on instantaneous vehicle dynamics:

$$R_{\text{target}} = R_0 \cdot \left(1 + \alpha_\omega \omega_z^2 + \alpha_c (v_{\text{fwd}} \omega_z)^2 + \alpha_a a_y^2\right)$$

with parameters:
- $R_0 = (0.05\text{ m/s})^2 = 0.0025\text{ m}^2/\text{s}^2$
- $\alpha_\omega = 25.0$
- $\alpha_c = 12.0$
- $\alpha_a = 5.0$
- $R_{\text{max}} = (1.50\text{ m/s})^2 = 2.25\text{ m}^2/\text{s}^2$
- First-order smoothing: $R_{\text{lat}}[k] = 0.70 \cdot R_{\text{lat}}[k-1] + 0.30 \cdot \text{clip}(R_{\text{target}}, R_0, R_{\text{max}})$

This baseline is stored permanently in `BASELINE_B0_RESULTS.json`.
