# PHASE 16 & 17 RESEARCH-TO-PRODUCTION CAUSALITY & ARCHITECTURAL AUDIT

**Document ID:** `AUDIT-PHASE16-17-01`  
**Date:** September 27, 2026  
**Status:** COMPLETE & AUTHORITATIVE  
**Lead Auditor:** Autonomous Navigation Systems Engineering  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Executive Summary & Audit Mandate

This audit investigates the reported performance discrepancies between the **N7 / B0 Production Baseline (35.71% drift)** and recent advanced research iterations: **Phase 16 (27.25% drift)** and **Phase 17 (27.64% drift)**. 

Under the strict project mandate, **scientific honesty takes absolute priority over marketing metrics**. The goals of this audit are to:
1. Re-run and mathematically verify the reported benchmark figures.
2. Interrogate the codebase line-by-line for non-causal operations, future lookahead, or oracle leakage.
3. Quantify the contribution of each individual algorithmic mechanism through controlled ablation studies.
4. Establish whether the advancements can be safely migrated to the production pipeline without violating causality or autonomous operation constraints.

---

## 2. Key Audit Findings

### Finding 1: Lineage-Wide Ground-Truth Speed Oracle Leakage
Across the historical research scripts in `scripts/`:
- `scripts/run_phase10c_adaptive_nhc.py` (Line 152) — evaluated N0 through N7
- `scripts/run_phase14_ablation_benchmarks.py` (Line 212) — evaluated Phase 14
- `scripts/run_phase15_final_benchmark.py` (Line 198) — evaluated Phase 15
- `scripts/run_phase16_final_benchmark.py` (Line 214) — evaluated Phase 16
- `scripts/run_phase17_final_benchmark.py` (Line 139) — evaluated Phase 17

**Every single one of these scripts contained the identical line inside the GNSS-denied outage loop:**
```python
ai_spd = float(traj.gt_speed[k])  # CAN bus / OBD-II ground-truth velocity
```
**Impact:** During GNSS-denied propagation, the filter was updated with the vehicle's true wheel/CAN speed, rather than an autonomously inferred IMU velocity. While this was historically done as a surrogate to evaluate filter mechanics in isolation, it means **all historical numbers (35.71%, 31.65%, 30.88%, 27.25%, 27.64%) represent CAN-aided Dead Reckoning, not pure autonomous smartphone IMU dead reckoning**.

When the system is run in pure autonomous mode using the Top-45 Huber GBDT model, longitudinal velocity error accumulates:
- Baseline N7 drift shifts from **37.60%** (CAN speed) to **65.81%** (Autonomous GBDT).
- Phase 16 drift shifts from **30.02%** (CAN speed) to **57.00%** (Autonomous GBDT).
- Phase 17 drift shifts from **31.04%** (CAN speed) to **58.16%** (Autonomous GBDT).

### Finding 2: Genuine Algorithmic Improvements in Phase 16 & 17
Despite the speed oracle issue present in the benchmark harness, the **filter improvements introduced in Phase 16 and 17 are 100% causal and physically valid**:
1. **Pre-Outage Doppler Observability Handover (Phase 16):**
   - Causality: Validated causal. Runs during $t \in [t_0 - 3.0\text{s}, t_0]$ before outage onset, when GNSS Doppler fixes are fully available.
   - Mechanism: Fuses smartphone GNSS velocity vectors (`GPS SPEED` and `GPS ORIENTATION`) into an ESKF pre-filter. Observes accelerometer bias $b_a$ and gyroscope bias $b_g$, and converges the state covariance $P[6:15, 6:15]$ from high initial uncertainty to steady-state values.
   - Benefit: Reduces median cross-track error from **28.77 m to 23.74 m** and median drift from **37.60% to 35.64%** on its own.
2. **Bounded-Slip NHC Covariance Clamping ($R_{\text{max}} = 0.16\text{ m}^2/\text{s}^2$):**
   - Causality: Validated causal. In N7, $R_{\text{lat}}$ was allowed to inflate up to $R_{\text{max}} = 2.25\text{ m}^2/\text{s}^2$ during aggressive turns, effectively disabling the non-holonomic constraint and allowing unconstrained lateral divergence. Clamping $R_{\text{max}} = (0.40\text{ m/s})^2$ maintains lateral control during severe maneuvers.
3. **Dynamic Lateral Kinematic Confidence ($c_{\text{lat}}$ in Phase 17):**
   - Causality: Validated causal. Uses instantaneous $\omega_z, a_y, v_{\text{fwd}}$ to estimate whether the vehicle is traveling straight or turning, smoothly tightening $R_{\text{lat}}$ to $R_{\text{strong}} = 0.025^2$ during straight driving and relaxing it during curves.
4. **Cockpit Lever-Arm NHC ($v_{y,\text{kin}} = 1.2 \cdot \omega_z$ in Phase 17):**
   - Causality: Validated causal. Accounts for the lateral velocity caused by rotation around the vehicle center of gravity when the phone is mounted in the dashboard/windshield.

---

## 3. Systematic Component-Level Ablation Matrix

The following table summarizes the controlled ablation study conducted across all 158 authentic IO-VNBD outages:

| Configuration ID | Speed Regime | Pre-Outage Handover | NHC $R_{\text{max}}$ | In-Flight ZARU | Dynamic $c_{\text{lat}}$ | Lever-Arm NHC | Median Drift (%) | Mean Drift (%) | Pass (<10%) | Med Cross (m) |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `B0_N7_Baseline` | CAN (Historical) | No | 2.25 | No | No | No | **37.60%** | 47.66% | 35 / 158 | 28.77 m |
| `Abl_A_Handover_Only` | CAN (Historical) | **Yes** | 2.25 | No | No | No | **35.64%** | 46.08% | 36 / 158 | 23.74 m |
| `Abl_B_BoundedSlip_Only` | CAN (Historical) | No | **0.16** | No | No | No | **37.68%** | 47.20% | 36 / 158 | 30.54 m |
| `Abl_C_InFlightZARU_Only` | CAN (Historical) | No | 2.25 | **Yes** | No | No | **37.60%** | 47.66% | 35 / 158 | 28.77 m |
| `Phase16_Full` | CAN (Historical) | **Yes** | **0.16** | **Yes** | No | No | **30.02%** | **43.72%** | **43 / 158** | **20.51 m** |
| `Abl_D_DynamicConf_Only` | CAN (Historical) | **Yes** | **0.16** | No | **Yes** | No | **35.78%** | 45.38% | 38 / 158 | 22.49 m |
| `Abl_E_LeverArmNHC_Only` | CAN (Historical) | **Yes** | **0.16** | No | No | **Yes** | **35.01%** | 45.04% | 38 / 158 | 23.02 m |
| `Phase17_Full` | CAN (Historical) | **Yes** | **0.16** | **Yes** | **Yes** | **Yes** | **31.04%** | **44.16%** | **42 / 158** | **20.88 m** |
| `Auton_B0_N7` | Autonomous GBDT | No | 2.25 | No | No | No | **65.81%** | 129.47% | 4 / 158 | 47.12 m |
| `Auton_Phase16` | Autonomous GBDT | **Yes** | **0.16** | **Yes** | No | No | **57.00%** | 124.97% | 5 / 158 | **40.12 m** |
| `Auton_Phase17` | Autonomous GBDT | **Yes** | **0.16** | **Yes** | **Yes** | **Yes** | **58.16%** | 126.12% | 5 / 158 | 42.09 m |

---

## 4. Architectural Analysis: Why Synergy Matters

A vital insight from the ablation data is that **individual features in isolation produce modest gains, but their combination delivers a substantial breakthrough**:
- Pre-outage handover alone: 37.60% $\to$ 35.64% ($\Delta = -1.96\%$)
- Bounded slip alone: 37.60% $\to$ 37.68% ($\Delta = +0.08\%$)
- In-flight ZARU alone: 37.60% $\to$ 37.60% ($\Delta = 0.00\%$)
- **Combined Phase 16 (Handover + Bounded Slip + ZARU):** 37.60% $\to$ **30.02%** ($\Delta = -7.58\%$), with passes jumping from 35 to 43 (+8 passes) and cross-track error plummeting from 28.77 m to 20.51 m!

**Why?**
When the filter enters an outage with converged biases ($b_a, b_g$) and a low-uncertainty covariance $P$, the bounded-slip NHC can firmly constrain lateral velocity without fighting an unobserved gyro bias. Conversely, if $P$ is large and biases are unobserved, clamping NHC too tightly forces the filter to interpret attitude errors as velocity innovations, causing filter windup.

---

## 5. Firewall & Compliance Evaluation

1. **Strict Causality Firewall:**
   Both `tests/test_no_ground_truth_in_inference.py` and `tests/test_strict_causality.py` pass. Pre-outage Doppler handover uses data only prior to $t_0$.
2. **Production Code Independence:**
   Neither Phase 16 nor Phase 17 introduced any Android or external runtime dependencies.
3. **Deployment Feasibility:**
   Pre-outage Doppler velocity is standard GNSS receiver output available on Android via `Location.getSpeed()` and `Location.getBearing()` or `GnssClock/GnssMeasurement`.

---

## 6. Audit Conclusion & Recommendation

The Phase 16 observability upgrade (Pre-Outage Doppler Handover + Bounded-Slip NHC + In-Flight ZARU) represents a mathematically rigorous, causal improvement that reduces lateral drift by ~29% and overall median drift by ~7.6% under CAN speed, and improves autonomous drift from 65.81% to 57.00%.

However, to maintain absolute scientific integrity with ISRO evaluators:
1. All benchmark figures must explicitly state the speed source (CAN bus speed vs autonomous IMU GBDT speed).
2. The claim of "<10% drift across 158 outages" must never be made, because actual pass rates are 27–31% under CAN speed and 3.2% under pure autonomous IMU dead reckoning.
