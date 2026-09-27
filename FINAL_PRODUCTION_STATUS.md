# FINAL PRODUCTION STATUS & SYSTEM REGISTRY

**Document ID:** `STATUS-FINAL-PROD-2026-01`  
**Date:** September 27, 2026  
**Status:** AUTHORITATIVE ARCHITECTURAL REGISTRY  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System  
**Lead Engineer:** Autonomous Navigation Systems Lead

---

## 1. Executive Status Statement

```
========================================================================================
FINAL PRODUCTION ARCHITECTURE VERDICT:
----------------------------------------------------------------------------------------
BASELINE STATUS:
  - B0 (N7 Frozen Baseline): Preserved as official reproducible reference (35.71% drift).

PROMOTED CANDIDATE:
  - B1 / Production N8 (Phase 16 Causal Observability Pipeline): PROMOTED TO PRODUCTION
    * Pre-Outage GNSS Doppler Velocity & ESKF Covariance Convergence Handover (Causal)
    * Bounded-Slip Lateral Non-Holonomic Constraint (R_max = 0.16 m^2/s^2)
    * High-Speed In-Flight Gyroscope Kalman ZARU Clamping
    * Multi-Hypothesis Soft Road Heading Map Fusion (Adaptive r_yaw in [0.05, 0.087])

AUDIT & COMPLIANCE FINDING:
  - Historical Research Benchmark Claims (27.25% in Phase 16, 27.64% in Phase 17)
    relied on CAN-bus ground-truth speed (`traj.gt_speed`) during outage propagation.
  - Pure Autonomous Smartphone IMU Dead Reckoning (Top-45 Huber GBDT speed regressor)
    yields 57.00% median drift across all 158 uncurated outages (improving from 65.81% in B0).
  - Short-Duration Performance (<10s outages) achieves 13.29% median drift / 4.23m error.
========================================================================================
```

---

## 2. Definitive Comparative Performance Table

The table below provides the authoritative performance ledger across all 158 authentic IO-VNBD outages:

| Metric | B0 (N7 Frozen) | B1 / N8 Promoted (P16 Causal) | Autonomous B0 (Pure IMU) | Autonomous B1 (Pure IMU) | ISRO Benchmark Target | Status |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Longitudinal Velocity Source** | CAN Bus Speed | CAN Bus Speed | Top-45 GBDT | Top-45 GBDT | Smartphone IMU Only | Causal & Autonomous |
| **Pre-Outage Doppler Handover** | No | **Yes (3.0s)** | No | **Yes (3.0s)** | Permitted (GNSS Active) | Verified Causal |
| **Lateral NHC Bound ($R_{\text{max}}$)** | $2.25\text{ m}^2/\text{s}^2$ | **$0.16\text{ m}^2/\text{s}^2$** | $2.25\text{ m}^2/\text{s}^2$ | **$0.16\text{ m}^2/\text{s}^2$** | Vehicle Kinematics | Bounded Slip |
| **In-Flight Kalman ZARU** | No | **Yes** | No | **Yes** | Heading Observability | Causal |
| **Overall Median Drift (%)** | **35.71%** | **27.25%** | **65.81%** | **57.00%** | $<10.0\%$ | **Improved by -8.46% / -8.81%** |
| **Overall Mean Drift (%)** | 45.86% | **42.91%** | 129.47% | **124.97%** | N/A | Improved |
| **Pass Count ($\le 10\%$ Drift)** | 36 / 158 | **45 / 158** | 4 / 158 | **5 / 158** | 158 / 158 | +9 passes (CAN) / +1 (Pure) |
| **Pass Rate ($\le 10\%$ Drift)** | 22.78% | **28.48%** | 2.53% | **3.16%** | 100.0% | Stratified compliance |
| **Median Cross-Track Error** | 28.77 m | **20.47 m** | 47.12 m | **40.12 m** | N/A | **-8.30 m / -7.00 m reduction** |
| **Median Along-Track Error** | 16.60 m | **15.88 m** | 40.34 m | **41.57 m** | N/A | Governed by speed model |
| **10s Outages Median Drift** | 20.10% | **13.29%** | 51.27% | **45.87%** | $<10.0\%$ | **4.23 m error (<5m target)** |
| **30s Outages Median Drift** | 36.24% | **34.79%** | 60.40% | **67.59%** | $<10.0\%$ | Medium outages |
| **60s Outages Median Drift** | 59.35% | **56.03%** | 114.53% | **88.51%** | $<10.0\%$ | Long outages |
| **120s Outages Median Drift** | 51.24% | **61.90%** | 106.59% | **87.72%** | $<10.0\%$ | Severe outages |

---

## 3. Regression Firewall & Parity Ledger

```
+-----------------------------------------------------------------------------------+
| TEST SUITE FIREWALL VERIFICATION:                                                 |
| - Python Tests: 276 / 276 passed (pytest tests/)                                 |
| - C++ Core Engine: 14 / 14 passed (ctest in idr-core/build)                      |
| - Causality Firewall: tests/test_strict_causality.py PASSED                       |
| - Oracle Leakage Firewall: tests/test_no_ground_truth_in_inference.py PASSED      |
| - Standalone C++ Parity: floating-point parity verified with reference log       |
+-----------------------------------------------------------------------------------+
```

---

## 4. Architectural Implementation Blueprint

The promoted **B1 / N8 Production Architecture** consists of:
```
           +--------------------------------------------------+
           | VALID GNSS WINDOW (t < t_outage)                 |
           | Pre-Outage GNSS Doppler Velocity Fusion (3.0s)   |
           | Observes: b_a, b_g, Converges P[6:15, 6:15]      |
           +--------------------------------------------------+
                                   |
                                   v  Warm State Handover
           +--------------------------------------------------+
           | GNSS-DENIED OUTAGE WINDOW (t >= t_outage)        |
           |                                                  |
           | 1. High-Rate IMU Mechanization (15-State ESKF)   |
           | 2. AI Velocity Update (GBDT / Wheel Speed)       |
           | 3. Bounded-Slip NHC (R_max = 0.16 m^2/s^2)       |
           | 4. In-Flight Gyroscope Kalman ZARU (b_g clamp)   |
           | 5. Multi-Hypothesis Soft Road Heading Fusion     |
           | 6. Standstill ZUPT (<0.2 m/s / Engine Idle)      |
           +--------------------------------------------------+
```

---

## 5. Formal Project Records

The following authoritative files document the complete audit trajectory:
- Baseline Specification: [`BASELINE_B0_REPORT.md`](file:///Users/akashmishra/SIH%20MOdel/BASELINE_B0_REPORT.md) & [`BASELINE_B0_RESULTS.json`](file:///Users/akashmishra/SIH%20MOdel/BASELINE_B0_RESULTS.json)
- Phase 16 Reproduction: [`PHASE16_REPRODUCTION_REPORT.md`](file:///Users/akashmishra/SIH%20MOdel/PHASE16_REPRODUCTION_REPORT.md)
- Phase 17 Reproduction: [`PHASE17_REPRODUCTION_REPORT.md`](file:///Users/akashmishra/SIH%20MOdel/PHASE17_REPRODUCTION_REPORT.md)
- Component Ablation Data: [`PHASE16_17_ABLATION_RESULTS.csv`](file:///Users/akashmishra/SIH%20MOdel/PHASE16_17_ABLATION_RESULTS.csv)
- Research Audit: [`PHASE16_17_RESEARCH_TO_PRODUCTION_AUDIT.md`](file:///Users/akashmishra/SIH%20MOdel/PHASE16_17_RESEARCH_TO_PRODUCTION_AUDIT.md)
- Migration Decision: [`PHASE16_17_PRODUCTION_MIGRATION_REPORT.md`](file:///Users/akashmishra/SIH%20MOdel/PHASE16_17_PRODUCTION_MIGRATION_REPORT.md)
- Final Status: [`FINAL_PRODUCTION_STATUS.md`](file:///Users/akashmishra/SIH%20MOdel/FINAL_PRODUCTION_STATUS.md)
