# PHASE 16 RECOVERY ARCHITECTURE AUDIT
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Exhaustive Code-Level Audit of Historical Phase 16 vs. Current Unified Architecture

**Audit Date:** September 27, 2026  
**Auditor:** Lead Navigation Systems Engineer, ISRO-SAC Team  
**Scope:** `scripts/run_phase16_final_benchmark.py`, `scripts/run_phase16_17_ablation_study.py`, `idr_pipeline/`, `idr-core/`  
**Status:** COMPLETE & AUTHORITATIVE  

---

## 1. Executive Summary

This audit reconstructs the exact historical implementation of **Phase 16** (`scripts/run_phase16_final_benchmark.py`) and contrasts every algorithmic mechanism against the current unified pipeline (`idr_pipeline/` and `scripts/run_part2_controlled_ablations.py`).

The objective is to identify why Phase 16 achieved high performance and determine exactly which mechanisms were lost, altered, or overly constrained during subsequent unification.

---

## 2. Systematic Mechanism Comparison Table

| Phase-16 Mechanism | Historical Phase-16 Implementation | Current Unified Implementation | Lost / Changed? | Recovery Required? |
|---|---|---|---|---|
| **Pre-Outage Doppler Handover** | 3.0s window ($t \in [t_0-3, t_0]$) fusing GNSS velocity (`r_vel=0.35`) and course heading (`r_yaw=0.08`, $\chi^2=16.0$) to converge $\mathbf{b}_a, \mathbf{b}_g, \mathbf{P}[6:15, 6:15]$. Loaded from `S-*.csv`. | Preserved in principle, but `csv_mapping` had `Smartphone_2019...csv` (file not found), falling back to `gt_vel`. | **CHANGED (Filename mismatch caused fallback to GT vel in benchmark)** | **YES: Point `csv_mapping` to actual `S-*.csv` files for pure autonomous Doppler.** |
| **Forward Velocity Input** | Used `ai_spd = float(traj.gt_speed[k])` inside outage loop (CAN-aided reference). | Uses pure autonomous GBDT forward speed (`gbdt_speeds[drive][k]`). | **CHANGED (Historical used GT speed oracle; unified uses autonomous GBDT)** | **NO: Keep autonomous GBDT. GT speed is strictly forbidden during outage.** |
| **Adaptive $R_{\text{AI}}$ Covariance** | $R_{\text{AI}} = 0.35 \cdot (1.0 + 0.5 \|\mathbf{a}_{\text{vib}}\| + 1.2 \|\omega_z\|)$. | Identical formula, but in Part 2 standstill clamped $R_{\text{AI}} = 0.05$ when $v_{\text{eff}} < 0.5$. | **PRESERVED** | **NO: Mechanism intact.** |
| **Non-Holonomic Constraints (NHC)** | Bounded slip: $R_{\text{max}} = 0.40^2 = 0.16\text{ m}^2/\text{s}^2$. $R_{\text{target}} = 0.05^2 (1 + 25\omega_z^2 + 12 a_{\text{cent}}^2 + 5 a_y^2)$. $\beta = 0.70$ smoothing. $R_{\text{vert}} = 0.05^2$. | Identical bounded-slip equation, but added dynamic $R_{\text{max}} = 0.09$ on straight cruise. | **PRESERVED** | **NO: Mechanism intact.** |
| **In-Flight Kalman ZARU** | Trigger: $|\omega_z| < 0.005\text{ rad/s}, |a_y| < 0.15\text{ m/s}^2, v_{\text{fwd}} > 4.0\text{ m/s}$ for $>8$ ticks. Updates $\mathbf{b}_{g,z}$ via $H = [0_{14}, 1]$, $R_{\text{zaru}} = 0.005^2$. | Identical implementation. | **PRESERVED** | **NO: Mechanism intact.** |
| **Map Matching Interaction** | **HEADING-ONLY SOFT FUSION.** Top-3 candidates, circular weighted mean, applies `update_road_heading(fused_az, r_yaw=0.050/0.087, chi2_gate=25.0)` if $|\Delta\psi| < 45^\circ$. **ZERO position modification.** | **3D POSITION + HEADING FUSION.** Multi-hypothesis forward Viterbi, $4.0\text{ m}$ elevation barrier, applies `update_road_position` along road normal $\mathbf{n}_{\text{road}}$, gated by $C_{\text{map}} \ge 0.35$. | **CRITICALLY CHANGED: Phase 16 updated ONLY heading with a relaxed 45° gate and 25.0 $\chi^2$ gate; unified applies position snapping which distorted trajectory.** | **YES: Test restoring Phase-16 heading-only soft road fusion.** |
| **Standstill & Idle Handling** | Basic check: `if vib_state.is_engine_idling or (ai_spd < 0.2): update_zupt(0.01)`. Idle detection was mostly failing in `Driver_A_S3b` due to tilt threshold. | Part 2 used identical logic. Part 3 tested AC dynamic acceleration blanking. | **CHANGED: Idle detection was failing in both.** | **YES: Physical AC dynamic acceleration standstill blanking.** |
| **Speed Calibration** | `cal_speed = engine.speed_calibrator.correct_speed(ai_spd)` (piecewise linear/polynomial speed calibration). | `speed_calibrator` was disabled in unified pipeline (`enable_speed_calibration=False`). | **LOST: Speed calibrator was disabled during Part 1/2 unification.** | **YES: Test recovering speed calibration.** |
| **Longitudinal Map Constraint** | Enabled `enable_longitudinal_map_constraint=True`. | Disabled in Part 2. | **LOST** | **YES: Evaluate if beneficial.** |

---

## 3. The Decisive Architectural Delta

The code audit reveals two major structural differences between historical Phase 16 and the current unified system:

1. **Map Fusion Modality:**  
   - **Phase 16** applied **road heading updates only** (`update_road_heading`) with a relaxed $45^\circ$ angle threshold and $\chi^2 = 25.0$ gate. It did **NOT** alter vehicle Cartesian position directly. This kept the gyroscope bias constrained and prevented heading wander along corridors without warping position.
   - **Current Unified System** introduced soft **position snapping** (`update_road_position`) along the road normal vector. When map confidence is borderline, position snapping introduces artificial cross-track impulses that compete with the inertial velocity state.
2. **Speed Calibration:**  
   - **Phase 16** included `engine.speed_calibrator.correct_speed()`, which applied causal polynomial scale corrections to forward speed. In Part 1 and Part 2, `speed_calibrator` was bypassed (`enable_speed_calibration=False`), allowing raw GBDT speed bias ($+2.5\text{--}3.9\text{ m/s}$) to enter the filter uncorrected.
