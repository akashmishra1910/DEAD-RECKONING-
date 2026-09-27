# B0 vs. B1/N8 Head-to-Head Comparative Report
## Autonomous Engine Validation under Strict Regime B Conditions

**Document ID:** `VAL-B0-VS-B1N8-COMPARATIVE`  
**Date:** September 27, 2026  
**Architectures Compared:**
1. **Autonomous B0 (Frozen Baseline):** Standard GBDT speed + unconstrained lateral NHC ($R_{\max}=10.0$) + static gyro bias + standard 15-state ESKF.
2. **Autonomous B1 / N8 (Promoted Production Candidate):** Pre-outage GNSS Doppler calibration + bounded-slip lateral NHC ($R_{\max}=0.16\text{ m}^2/\text{s}^2$) + in-flight Kalman ZARU on straight segments + causal 3D HMM map matching.

---

## 1. Architectural Parameter Comparison

| Pipeline Component | Autonomous B0 Baseline | Autonomous B1 / N8 Promoted | Analytical Justification |
|---|---|---|---|
| **Speed Estimator** | GBDT Top-45 Huber | GBDT Top-45 Huber | Preserves validated causal ML model |
| **Speed Handover** | Uncalibrated Raw Output | Pre-outage GNSS Doppler Scale & Bias Calibration | Corrects mounting angle & tire pressure variance before outage |
| **Lateral NHC Covariance** | Unbounded ($R_{\max}=10.0\text{ m}^2/\text{s}^2$) | Bounded ($R_{\max}=0.16\text{ m}^2/\text{s}^2$) | Prevents excessive lateral drift during cornering without over-constraining slip |
| **Gyroscope Bias Tracking** | Static during outage | In-Flight Kalman ZARU during straight driving | Observes $b_{g,z}$ when $|\omega_z| < 0.005\text{ rad/s}$ and $v > 4.0\text{ m/s}$ |
| **Map Matching** | Baseline 3D HMM | Causal 3D HMM with elevation barrier ($|\Delta h| > 4.0\text{ m}$) | Eliminates unphysical flyover snapping while preserving causal Viterbi decoding |
| **Oracle Speed Access** | **Strictly None (0%)** | **Strictly None (0%)** | Full scientific causality maintained |

---

## 2. Statistical Distribution Comparison (158 Outages)

```
========================================================================================
METRIC                   AUTONOMOUS B0          AUTONOMOUS B1 / N8     DELTA (B1 - B0)
========================================================================================
Median Drift Error       60.10%                 54.86%                 -5.24% (-8.72%)
Mean Drift Error         128.75%                123.14%                -5.61% (-4.36%)
P25 (First Quartile)     38.20%                 31.45%                 -6.75% (-17.67%)
P75 (Third Quartile)     112.40%                104.80%                -7.60% (-6.76%)
P90 (Ninetieth Pct)      180.79%                183.53%                +2.74% (+1.51%)
P95 (Ninety-fifth Pct)   236.50%                229.92%                -6.58% (-2.78%)
Pass Count (<= 10%)      5 / 158 (3.16%)        9 / 158 (5.70%)        +4 (+80.0%)
Median Cross-Track Err   45.86 m                41.53 m                -4.33 m (-9.44%)
Median Along-Track Err   41.31 m                37.10 m                -4.21 m (-10.19%)
========================================================================================
```

---

## 3. Component Ablation Matrix (A0 through A7)

To isolate the exact contribution of each enhancement in B1/N8, an exhaustive 8-way ablation was executed across all 158 outages in Regime B:

| Ablation ID | Description | Median Drift (%) | Mean Drift (%) | Passes ($\le 10\%$) | Cross-Track (m) |
|:---:|---|:---:|:---:|:---:|:---:|
| **A0** | Raw B0 Baseline (GBDT only) | 60.10% | 128.75% | 5 | 45.86 |
| **A1** | A0 + Pre-Outage Doppler Calibration | 57.84% | 125.60% | 6 | 44.12 |
| **A2** | A0 + Bounded-Slip NHC ($R_{\max}=0.16$) | 56.40% | 124.90% | 7 | 42.80 |
| **A3** | A0 + In-Flight Kalman ZARU | 58.95% | 127.10% | 6 | 45.10 |
| **A4** | A0 + Doppler Cal + Bounded NHC | 55.62% | 123.85% | 8 | 42.10 |
| **A5** | A0 + Bounded NHC + Kalman ZARU | 55.90% | 124.15% | 7 | 42.35 |
| **A6** | A0 + Doppler Cal + Kalman ZARU | 57.10% | 125.02% | 6 | 43.90 |
| **A7 (B1/N8)** | Full Promoted System (A1 + A2 + A3 + 3D Map) | **54.86%** | **123.14%** | **9** | **41.53** |

### Key Ablation Insights:
1. **Bounded-Slip NHC (A2):** Contributes the single largest individual reduction in cross-track error (-3.06 m), eliminating artificial lateral divergence during turns.
2. **Pre-Outage Doppler Calibration (A1):** Improves along-track consistency by matching the GBDT's scale to the vehicle's specific velocity dynamic range during the preceding 3 seconds.
3. **Synergistic Integration (A7):** Combining all three features produces the minimum overall median drift (54.86%) and maximizes the pass count (9 passes).

---

## 4. Win / Loss Analysis

Across the 158 outages:
- **B1/N8 Won (lower drift):** 104 / 158 (65.8%)
- **B0 Won (lower drift):** 52 / 158 (32.9%)
- **Tied / Identical:** 2 / 158 (1.3%)

**Wilcoxon Signed-Rank Test:** $p = 0.00042$ ($p < 0.01$). The improvement of B1/N8 over B0 is statistically significant.
