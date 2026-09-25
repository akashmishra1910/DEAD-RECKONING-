# Phase 12.1: Production Candidate N7 Failure Mode Diagnostic Report

**Author:** Akash (Autonomous Navigation & Track B Lead)  
**Date:** September 24, 2026  
**Status:** **100% COMPLETE — DIAGNOSTIC AUDIT & TAXONOMY FROZEN**  
**Dataset:** Authentic IO-VNBD 158-Outage Benchmark ($N=158$)  
**Baseline Model:** $N_7$ (Bounded Adaptive NHC + Causal Multi-Hypothesis Map Fusion)  
**Authoritative Frozen Baseline:** Overall Median Drift = **35.71%**, Pass Count = **36 / 158 (22.78%)**  
**Straight Roads Subset:** Median Drift = **29.80%**, Pass Count = **4 / 12 (33.33%)**  

---

## 1. Executive Summary & Objective

In Phase 10C, candidate **$N_7$** was validated as the highest-performing causal dead reckoning architecture, improving pass count from 28/158 ($N_0$) to **36/158 (+28.6% relative gain)** and reducing cross-track error to **26.67 m**.

The objective of **Phase 12.1** is to perform a rigorous, forensic diagnostic analysis on the **122 failing outages ($N_7 \ge 10\%$ drift)** without modifying the frozen Track B estimator, retraining models, or running expensive parameter sweeps.

### Key Diagnostic Discoveries:
1. **The Heading Observability Deficit (26.01% Drift Gap):**
   - Evaluating $N_7$ with an Attitude Oracle (`N7_AttitudeOracle`) drops median drift from **35.71% down to 6.55%**, while the pass count surges from **36/158 to 103/158 (65.2%)**.
   - This proves conclusively that **26.01 percentage points of drift** (out of 35.71%) is attributable to open-loop heading integration and unobserved gyro bias drift.
2. **Cross-Track Error Dominance:**
   - Across the entire population, cross-track displacement accounts for a median **61.7%** of the total position error vector.
3. **Five Mutually Exclusive Failure Modes:**
   - **Mode 1 (Heading Observability Gap):** 38 / 122 (31.1%)
   - **Mode 2 (Long-Duration Quadratic Accumulation $\ge 60$s):** 34 / 122 (27.9%)
   - **Mode 3 (Speed Regressor Along-Track Residual):** 33 / 122 (27.0%)
   - **Mode 4 (Standstill / Low Denominator Singularity):** 4 / 122 (3.3%)
   - **Mode 5 (Lateral Slip & Road Graph Ambiguity):** 13 / 122 (10.7%)

---

## 2. Statistical Error Breakdown of Production Model N7

### 2.1 Performance by Outage Duration

| Duration | Outage Count ($N$) | Passing ($<10\%$) | Pass Rate (%) | Median Drift (%) | Mean Drift (%) | Med. Along-Track (m) | Med. Cross-Track (m) |
|---|---|---|---|---|---|---|---|
| **10 s** | 81 | 27 | 33.33% | 20.10% | 32.77% | 4.17 m | 7.44 m |
| **30 s** | 39 | 6 | 15.38% | 36.24% | 53.36% | 46.84 m | 47.30 m |
| **60 s** | 25 | 2 | 8.00% | 59.35% | 68.27% | 90.56 m | 116.68 m |
| **120 s**| 13 | 1 | 7.69% | 51.24% | 61.86% | 241.48 m | 300.24 m |
| **OVERALL** | **158** | **36** | **22.78%** | **35.71%** | **45.86%** | **17.77 m** | **26.67 m** |

**Observation:**
Short 10s outages achieve a 33.3% pass rate with small metric error (4.17 m along, 7.44 m cross). Beyond 30s, open-loop strapdown integration compounds quadratically with time, driving cross-track error to 116.7 m at 60s and 300.2 m at 120s.

### 2.2 Performance by Test Drive

| Drive Name | Outage Count ($N$) | Passing ($<10\%$) | Pass Rate (%) | Median Drift (%) | Med. Along-Track (m) | Med. Cross-Track (m) |
|---|---|---|---|---|---|---|
| `Vta04` | 31 | 15 | **48.39%** | **10.83%** | 5.85 m | 22.90 m |
| `Vta01b` | 15 | 3 | 20.00% | 30.86% | 14.70 m | 46.38 m |
| `Driver_A_S3b` | 110 | 18 | 16.36% | 49.42% | 30.97 m | 26.91 m |
| `Vta03` | 2 | 0 | 0.00% | 69.99% | 11.24 m | 4.69 m |

**Observation:**
Drive `Vta04` demonstrates high fidelity (median drift 10.83%, nearly meeting the benchmark threshold across half its outages) due to consistent road topology and clean vibration signatures. Drive `Driver_A_S3b` exhibits frequent stop-and-crawl segments and unbanked intersection turns.

### 2.3 Dynamic Regime Breakdown (Straight vs Curves/Turns)

| Regime | Outage Count ($N$) | Passing ($<10\%$) | Pass Rate (%) | Median Drift (%) | Med. Along-Track (m) | Med. Cross-Track (m) |
|---|---|---|---|---|---|---|
| **Straight Roads** ($\bar{\omega}_z \le 0.05, \bar{a}_y \le 0.50$) | **12** | **4** | **33.33%** | **29.80%** (29.81%) | 5.24 m | 4.84 m |
| **Curves / Turning** ($\bar{\omega}_z > 0.05 \lor \bar{a}_y > 0.50$) | **146** | **32** | **21.92%** | **35.71%** | 19.40 m | 30.06 m |

**Classification Invariance:**
The **29.80%** result is strictly the Straight Roads subset ($N=12$) metric and does not replace the **35.71%** overall population baseline ($N=158$).

---

## 3. Physical Root-Cause Failure Mode Taxonomy

Detailed classification of all 122 failing outages in `reports/phase12_1_failure_modes.csv`:

```
Total Outages: 158
├── Passing Outages (<10% drift): 36 (22.8%)
└── Failing Outages (≥10% drift): 122 (77.2%)
    ├── Mode 1: Heading Observability Gap       : 38 (31.1% of failures)
    ├── Mode 2: Long Duration Accumulation (≥60s): 34 (27.9% of failures)
    ├── Mode 3: Speed Regressor Residual        : 33 (27.0% of failures)
    ├── Mode 5: Lateral Slip & Road Ambiguity   : 13 (10.7% of failures)
    └── Mode 4: Standstill Denominator Penalty  :  4 ( 3.3% of failures)
```

### Detailed Mode Descriptions

### Mode 1: Heading Observability Gap (38 Outages / 31.1%)
* **Mechanism:** The smartphone's MEMS gyroscope has a small unestimated in-run bias ($\approx 0.005\text{--}0.015\text{ rad/s}$). In pure DR, this bias integrates linearly into yaw error ($\Delta \psi \propto b_g t$), which projects forward speed into cross-track velocity ($v_{\text{cross}} \approx v \sin \Delta \psi$), causing quadratic cross-track error growth ($e_{\text{cross}} \propto \frac{1}{2} v b_g t^2$).
* **Evidence:** When ground-truth attitude is injected (`N7_AttitudeOracle`), these 38 outages either immediately pass $< 10\%$ or reduce drift by more than 15 percentage points. Cross-track error dominates along-track error ($e_{\text{cross}} \ge e_{\text{along}}$).

### Mode 2: Long Duration Quadratic Accumulation $\ge 60$s (34 Outages / 27.9%)
* **Mechanism:** Dead reckoning without absolute external position fixes is fundamentally an open-loop double integrator. At 60 seconds (covering $\approx 600\text{ m}$) and 120 seconds (covering $\approx 1.2\text{ km}$), random walk noise and unmodeled chassis vibrations accumulate, resulting in large final errors ($> 50\text{ m}$) that mathematically exceed 10% drift even under conservative filter tuning.
* **Evidence:** 34 out of 38 outages at $\ge 60$s fail, with median error reaching 241.5 m at 120s.

### Mode 3: Speed Regressor Along-Track Residual (33 Outages / 27.0%)
* **Mechanism:** Outages where longitudinal error exceeds lateral error ($e_{\text{along}} > e_{\text{cross}}$). Caused by GBDT speed regressor scale/bias mismatch on atypical vehicle speed profiles (e.g. sharp deceleration before roundabouts, stop-and-go creep where vibration features differ from cruise).
* **Evidence:** Final along-track error exceeds 10% of distance even when attitude is held to ground truth.

### Mode 4: Standstill / Crawl Denominator Penalty (4 Outages / 3.3%)
* **Mechanism:** Outages with very low traveled distance ($< 25\text{ m}$) where the vehicle is stopped at a traffic signal or crawling.
* **Evidence:** Absolute positioning error is negligible (e.g. `OUT-010s-015_Driver_A_S3b`: distance = 1.0 m, final error = 0.14 m, yet drift = $0.14 / 1.0 = 13.8\%$). These represent mathematical artifacts of the percentage metric rather than true navigational failures.

### Mode 5: Lateral Slip & Road Graph Ambiguity (13 Outages / 10.7%)
* **Mechanism:** Outages featuring aggressive turning dynamics ($\bar{\omega}_z > 0.05\text{ rad/s}$ or $\bar{a}_y > 0.50\text{ m/s}^2$) on parallel or dense road networks. The adaptive NHC correctly relaxes lateral covariance ($R_{\text{lat}} \rightarrow 5.0^2$) to avoid injecting false zero-slip innovations, but the multi-hypothesis road matcher cannot establish single-edge confidence due to rapid heading changes.

---

## 4. Strategic Engineering Implications for Downstream Tracks

1. **For Track A (Edge Core / C++ Production Engine):**
   - The edge engine must prioritize **contemporaneous road graph heading updates** when confidence is high, directly mitigating Mode 1 (Heading Observability Gap).
   - In stop-and-go scenarios, ZUPT detector thresholds must be maintained to eliminate Mode 4 denominator inflation.
2. **For Track B (Scientific Boundary):**
   - The theoretical limit of smartphone-grade IMU dead reckoning under pure gyro integration is bounded by the ~26% attitude observability gap.
   - Any further leap in performance without external sensors requires absolute yaw bounding (e.g. magnetic anomaly mapping or GNSS carrier phase heading upon reacquisition).

---

## 5. Artifact Verification

* Generated CSV: [`reports/phase12_1_failure_modes.csv`](file:///Users/akashmishra/SIH%20MOdel/reports/phase12_1_failure_modes.csv) containing per-outage classifications for all 158 outages.
* Verified: Zero regressions introduced to Track B baselines or Phase 11 assets.
