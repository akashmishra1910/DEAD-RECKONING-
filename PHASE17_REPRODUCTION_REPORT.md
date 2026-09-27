# PHASE 17 REPRODUCTION & CAUSALITY AUDIT REPORT

**Document ID:** `REPORT-PHASE17-REPRO-01`  
**Date:** September 27, 2026  
**Status:** FULLY REPRODUCED WITH ORACLE LEAKAGE AUDIT  
**Author:** Autonomous Navigation Engineering Team  
**Script Path:** [`scripts/run_phase17_final_benchmark.py`](file:///Users/akashmishra/SIH%20MOdel/scripts/run_phase17_final_benchmark.py)  
**Evaluation Standard:** 158 Authentic IO-VNBD Naturalistic Outages  
**Problem Statement:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Executive Summary

Phase 17 introduced the **Causal Kinematic Lateral-Motion & Long-Outage Drift Containment Benchmark**, built upon Phase 16:
1. **Dynamic Lateral-Motion Confidence ($c_{\text{lat}} \in [0, 1]$):**
   Calculated continuously from current instantaneous sensor observations:
   $$c_{\text{lat}} = c_{\text{spd}} \cdot c_{\text{yaw}} \cdot c_{\text{lat\_acc}} \cdot c_{\text{kin}}$$
   - $c_{\text{spd}} = \text{clip}\left(\frac{v_{\text{fwd}} - 0.5}{2.0}, 0, 1\right)$
   - $c_{\text{yaw}} = \exp\left(-0.5 \cdot \left(\frac{\omega_z}{0.025}\right)^2\right)$
   - $c_{\text{lat\_acc}} = \exp\left(-0.5 \cdot \left(\frac{a_y}{0.35}\right)^2\right)$
   - $c_{\text{kin}} = \exp\left(-0.5 \cdot \left(\frac{\lvert a_y - v_{\text{fwd}} \omega_z \rvert}{0.40}\right)^2\right)$
2. **Cockpit Lever-Arm Kinematic Lateral-Motion NHC:**
   Models physical centripetal lever-arm velocity at the smartphone mount ($l_x \approx 1.2\text{ m}$ ahead of the rear non-steered axle):
   $$v_{y,\text{kin}} = l_x \omega_z$$
   Innovation vector: $\delta z = [v_{y,\text{kin}} - v_y^b, -v_z^b]^T$.
3. **Smooth Adaptive Covariance $R_{\text{lat}}$:**
   Interpolates between $R_{\text{strong}} = (0.025\text{ m/s})^2$ and $R_{\text{loose}} = (0.40\text{ m/s})^2$ based on $c_{\text{lat}}$.
4. **Kinematic Consistency ZARU:**
   Clamps in-flight yaw gyro bias when $c_{\text{lat}} > 0.50$.

---

## 2. Benchmark Reproduction Results

Execution of `scripts/run_phase17_final_benchmark.py` across all 158 authentic outages yielded exact reproduction:

| Metric | B0 Production Baseline | Phase 16 | Phase 17 Reproduced | Phase 17 vs B0 ($\Delta$) | Phase 17 vs Phase 16 |
|---|:---:|:---:|:---:|:---:|:---:|
| **Overall Median Drift** | **35.71%** | 27.25% | **27.64%** | **-8.07%** | +0.39% |
| **Overall Mean Drift** | **45.86%** | 42.91% | **43.69%** | **-2.17%** | +0.78% |
| **Pass Count ($\le 10\%$)** | **36 / 158** | 45 / 158 | **49 / 158** | **+13 passes** | **+4 passes** |
| **Pass Rate ($\le 10\%$)** | **22.78%** | 28.48% | **31.01%** | **+8.23%** | +2.53% |
| **Median Final Error** | 42.50 m | 32.80 m | **32.03 m** | **-10.47 m** | -0.77 m |
| **Median Along-Track** | 16.60 m | 15.88 m | **15.63 m** | -0.97 m | -0.25 m |
| **Median Cross-Track** | 28.77 m | 20.47 m | **19.50 m** | **-9.27 m** | -0.97 m |

---

## 3. Duration Stratification Breakdown

| Outage Duration | Sample Count ($N$) | Phase 17 Median Drift | Passes ($\le 10\%$) | Along Error (m) | Cross Error (m) |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **10 seconds** | 81 | **13.64%** | 36 / 81 (44.4%) | 4.14 m | 4.83 m |
| **30 seconds** | 39 | **37.21%** | 10 / 39 (25.6%) | 42.49 m | 37.13 m |
| **60 seconds** | 25 | **62.71%** | 3 / 25 (12.0%) | 85.74 m | 116.36 m |
| **120 seconds** | 13 | **56.81%** | 0 / 13 (0.0%) | 304.74 m | 260.34 m |

---

## 4. Causality & Oracle Leakage Audit

### 4.1 Critical Finding: Longitudinal Speed Oracle Leakage
An audit of `scripts/run_phase17_final_benchmark.py` reveals identical ground truth leakage in Line 139:
```python
# Strictly causal outage propagation from i0 to i1
for k in range(i0, i1):
  ...
  ai_spd = float(traj.gt_speed[k])  # <--- CRITICAL ORACLE LEAKAGE
```
Like Phase 16 and N0–N7, Phase 17 relied on OBD-II / CAN bus ground-truth velocity to constrain longitudinal velocity.

### 4.2 Causality of Dynamic Lateral Kinematic Model
- All four components of $c_{\text{lat}}$ ($c_{\text{spd}}, c_{\text{yaw}}, c_{\text{lat\_acc}}, c_{\text{kin}}$) depend strictly on instantaneous sensor measurements available at sample $k$ ($\omega_z[k], a_y[k], v_{\text{fwd}}[k]$).
- The lever arm velocity $v_{y,\text{kin}} = l_x \omega_z$ is an instantaneous physical constraint.
- The filter update uses strictly causal Kalman error-state equations with proper direction cosine matrix ($R^T$) attitude rotation.
- **Verdict:** Causal and physically sound.

### 4.3 True Autonomous Performance (Without Oracle Speed)
When evaluated using the production Top-45 Huber GBDT model:
- `Auton_B0_N7`: Median drift = **65.81%**, Mean = **129.47%**, Passes = 4 / 158 (2.5%)
- `Auton_Phase17`: Median drift = **58.16%**, Mean = **126.12%**, Passes = 5 / 158 (3.2%)
- **Conclusion:** Phase 17 delivers a strong pass rate increase (+13 passes under CAN speed, 49/158, 31.01%) and reduces lateral error from 28.77 m to 19.50 m. Under pure autonomous GBDT speed, it improves over baseline B0 by **7.65%**, but cannot bridge the gap to <10% drift without external velocity aiding.
