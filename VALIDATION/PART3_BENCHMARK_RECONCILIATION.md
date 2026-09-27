# PART 3 BENCHMARK RECONCILIATION REPORT
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Exhaustive Historical Audit, Oracle Leakage Identification & Reconciliation

**Document Version:** 3.0.0  
**Status:** AUTHORITATIVE AUDIT & RECONCILIATION  
**Lead Navigation Systems Engineer:** Lead Core Navigation Systems Engineer  
**Audit Target:** All historical benchmark scripts from Phase 10c through Part 2  

---

## 1. Executive Summary

This report establishes why historical repository documents reported substantially lower drift numbers (e.g. ~27.25% in Phase 16, ~27.64% in Phase 17, ~35.71% in Phase 10c/N7), whereas recent strict autonomous evaluations reported ~59.63% median drift.

### The Decisive Reconciliation Finding:
Historical Phase 10c through Phase 17 benchmark scripts contained a line inside the GNSS-denied propagation loop:
```python
ai_spd = float(traj.gt_speed[k])
```
This line provided the Error-State Kalman Filter with the vehicle's **true ground-truth forward speed (equivalent to CAN-bus wheel speed)** at 10 Hz throughout the outage. 

Because forward velocity errors were continually eliminated by the ground-truth speed updates, the filter only had to manage lateral drift, resulting in artificial numbers around $27\text{--}35\%$. When this line is replaced with autonomous forward velocity inferred from IMU signals by the trained GBDT regressor, longitudinal speed errors accumulate naturally, shifting autonomous median drift to $\approx 57\text{--}60\%$.

---

## 2. Line-by-Line Code Audit of Historical Evaluators

| Script Path | Line Number | Code Statement | Physical Meaning | Status |
|---|---|---|---|---|
| `scripts/run_phase10c_adaptive_nhc.py` | Line 152 | `ai_spd = float(traj.gt_speed[k])` | Ground truth speed fed to ESKF | **LEAKAGE (CAN Surrogate)** |
| `scripts/run_phase14_ablation_benchmarks.py` | Line 212 | `ai_spd = float(traj.gt_speed[k])` | Ground truth speed fed to ESKF | **LEAKAGE (CAN Surrogate)** |
| `scripts/run_phase15_final_benchmark.py` | Line 198 | `ai_spd = float(traj.gt_speed[k])` | Ground truth speed fed to ESKF | **LEAKAGE (CAN Surrogate)** |
| `scripts/run_phase16_final_benchmark.py` | Line 220 | `ai_spd = float(traj.gt_speed[k])` | Ground truth speed fed to ESKF | **LEAKAGE (CAN Surrogate)** |
| `scripts/run_phase17_final_benchmark.py` | Line 145 | `ai_spd = float(traj.gt_speed[k])` | Ground truth speed fed to ESKF | **LEAKAGE (CAN Surrogate)** |
| `scripts/run_part2_controlled_ablations.py` | Line 215 | `ai_spd = float(gbdt_speeds[drive_name][k])` | Pure autonomous GBDT speed | **CLEAN & AUTONOMOUS** |
| `scripts/run_autonomous_validation_suite.py` | Line 172 | `ai_spd = float(v_ai_all[k])` | Pure autonomous GBDT speed | **CLEAN & AUTONOMOUS** |

---

## 3. Systematic Benchmark Reconciliation Table

To ensure complete scientific honesty, we re-ran both evaluation regimes on identical authentic 158 outages under both CAN-aided (historical oracle) and Pure Autonomous (strict IMU-only) conditions:

| Generation / Pipeline | Evaluation Mode | Speed Source | Median Drift (%) | Mean Drift (%) | $\le 10\%$ Pass | Median Cross-Track (m) | Median Along-Track (m) | Notes |
|---|---|---|---|---|---|---|---|---|
| **Phase 10c / N7 Baseline** | Historical CAN-aided | Ground Truth (`gt_speed`) | **37.60%** | 47.66% | 35 / 158 | 28.77 m | 14.20 m | Historical report cited ~35.7% on older subset |
| **Phase 16 Advanced Core** | Historical CAN-aided | Ground Truth (`gt_speed`) | **30.02%** | 43.72% | 43 / 158 | 20.51 m | 12.80 m | Historical report cited ~27.25% on 133 moving subset |
| **Phase 17 Full Advanced** | Historical CAN-aided | Ground Truth (`gt_speed`) | **31.04%** | 44.16% | 42 / 158 | 20.88 m | 13.10 m | Historical report cited ~27.64% on 133 moving subset |
| **B0 / N7 Autonomous** | Pure Autonomous | Inferred GBDT Speed | **65.81%** | 129.47% | 4 / 158 | 47.12 m | 46.80 m | Real unassisted baseline without CAN |
| **Phase 16 Autonomous** | Pure Autonomous | Inferred GBDT Speed | **57.00%** | 124.97% | 5 / 158 | 40.12 m | 39.50 m | Pre-outage Doppler + Bounded NHC |
| **Phase 17 Autonomous** | Pure Autonomous | Inferred GBDT Speed | **58.16%** | 126.12% | 5 / 158 | 42.09 m | 40.10 m | Cockpit lever-arm NHC |
| **Part 1 M6 Baseline** | Pure Autonomous | Inferred GBDT Speed | **60.32%** | 116.46% | 8 / 158 | 33.57 m | 37.20 m | Calibrated baro datum |
| **Part 2 P2-M4 (+ Map)** | Pure Autonomous | Inferred GBDT Speed | **56.40%** | 111.46% | 9 / 158 | **20.19 m** | 39.76 m | 3D Topological Map Matching |
| **Part 2 P2-M7 (Full Sys)** | Pure Autonomous | Inferred GBDT Speed | **59.63%** | 116.47% | 7 / 158 | 36.81 m | 41.32 m | Map Confidence + Integrity Gating |

---

## 4. Key Takeaways for Part 3

1. **The Old ~27% Numbers Were Not Autonomous Dead-Reckoning:** They demonstrated what the ESKF and NHC could do if an external CAN-bus/OBD-II odometer provided zero-error forward velocity.
2. **The True Autonomous Baseline is ~59.63%:** In the absence of CAN-bus wheel speed, forward speed estimation from IMU signals produces substantial along-track error ($41.32\text{ m}$ median), while open-loop gyro integration produces heading wander ($43.53^\circ$ median).
3. **The Target of Part 3:** Address the two root physical error drivers:
   - **Heading Observability:** In-flight gyro bias estimation and disturbance-gated magnetometer/kinematic aiding.
   - **Forward Speed Accuracy:** Temporal feature enhancements, motion-regime conditioning, and learned uncertainty scaling.
