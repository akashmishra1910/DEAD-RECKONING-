# SCIENTIFIC AUDIT: INFORMATION LEAKAGE & ORACLE RECOVERY

**Document ID:** `AUDIT-GT-LEAKAGE-01`  
**Date:** September 27, 2026  
**Status:** COMPLETE & AUTHORITATIVE  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System  
**Authority:** Autonomous Navigation System Lead Auditor

---

## 1. Executive Summary & Root-Cause Analysis

During the forensic audit of the historical research scripts, a fundamental information leakage was identified across multiple phases of research code: **the vehicle's OBD-II / CAN bus wheel speed (`traj.gt_speed`) was injected directly into the Dead Reckoning Kalman filter during GNSS outages**.

This document traces the complete lineage of this leakage, explains its historical rationale, maps every affected subsystem, and details the permanent hard firewall that guarantees 100% autonomous operation for Track A production.

---

## 2. Lineage Trace of Ground-Truth Speed Ingestion

The table below catalogs every occurrence of ground-truth velocity ingestion in the repository's evaluation scripts:

| Script Path | Code Location | Ingestion Statement | Operational Context | Regime Classification |
|---|:---:|---|---|:---:|
| `scripts/run_phase10c_adaptive_nhc.py` | Line 152 | `ai_spd = float(traj.gt_speed[g_idx])` | Evaluated N0 through N7 models | **Track B (Historical)** |
| `scripts/run_phase10a_drift_investigation.py` | Lines 109, 111 | `ai_spd = float(traj.gt_speed[g_idx])` | Investigated heading vs speed drift | **Track B (Historical)** |
| `scripts/run_phase14_ablation_benchmarks.py` | Line 212 | `ai_spd = float(traj.gt_speed[k])` | Evaluated Phase 14 A1 model | **Track B (Historical)** |
| `scripts/run_phase15_final_benchmark.py` | Line 198 | `ai_spd = float(traj.gt_speed[k])` | Evaluated Phase 15 bounded NHC | **Track B (Historical)** |
| `scripts/run_phase16_final_benchmark.py` | Line 220 | `ai_spd = float(traj.gt_speed[k])` | Evaluated Phase 16 Doppler upgrade | **Track B (Historical)** |
| `scripts/run_phase17_final_benchmark.py` | Line 145 | `ai_spd = float(traj.gt_speed[k])` | Evaluated Phase 17 kinematic confidence | **Track B (Historical)** |

### Why Did This Occur?
In early filter research (Phase 10), researchers separated the problem into two distinct questions:
1. *Filter Mechanization Limits:* How well does the 15-state ESKF with non-holonomic constraints perform if given an idealized speed sensor? (Evaluated with `gt_speed`).
2. *Learned Speed Regressor:* How accurately can a GBDT predict vehicle speed from smartphone accelerometer vibration? (Evaluated in `evaluate_phase4_ai_velocity_validation.py`).

Unfortunately, in subsequent phases (Phase 14 through Phase 17), the speed oracle was left in the outage loop. While the resulting benchmarks demonstrated superior filter mechanics (reaching 27.25% in Phase 16 and 27.64% in Phase 17), **they were CAN-aided Dead Reckoning benchmarks, not autonomous smartphone IMU dead reckoning**.

---

## 3. Subsystem Dependency & Downstream Contamination

Tracing where `traj.gt_speed` propagated in historical scripts:
```
[traj.gt_speed[k]] (OBD-II CAN Speed)
       │
       ▼
   ai_spd = float(traj.gt_speed[k])
       │
       ├──► Speed Calibration: cal_speed = engine.speed_calibrator.correct_speed(ai_spd)
       │
       ├──► Idle Gate: eff_speed = 0.0 if vib_state.is_engine_idling else cal_speed
       │
       ├──► ESKF Velocity Update: engine.eskf.update_ai_velocity(eff_speed, r_ai=r_ai_adapt)
       │       └── Zeroed longitudinal velocity random-walk drift!
       │
       ├──► NHC Centripetal Term: a_cent = v_fwd * omega_z (using CAN speed!)
       │
       └──► Standstill ZUPT: if (ai_spd < 0.2): engine.eskf.update_zupt(0.01)
```

**Consequence of Removal:**
When `ai_spd` is replaced with the autonomous GBDT model (`v_ai = model.predict(scaler.transform(X_45))`):
- The model must predict speed purely from IMU vibrations, acceleration variance, and rotational rates.
- Unaccelerated highway cruising produces open-loop integration drift.
- Baseline N7 drift increases from **35.71% (CAN speed)** to **60.10% (Autonomous IMU)**.
- B1 / N8 drift increases from **27.25% (CAN speed)** to **54.86% (Autonomous IMU)**.

---

## 4. Legitimate vs. Illegitimate Usages of Ground Truth

Ground-truth variables exist across the repository. They are formally classified as follows:

| Variable Name | Location / Usage | Legitimacy | Technical Justification |
|---|---|:---:|---|
| `traj.gt_speed` | `scripts/train_velocity_model.py` | **ALLOWED** | Supervised learning offline target labels. |
| `traj.gt_pos`, `traj.gt_speed` | `idr_pipeline/outage_catalog.py` | **ALLOWED** | Offline calculation of outage distance $D = \int v_{\text{gt}} dt$ for error normalization. |
| `traj.gt_speed[:150]` | Initial 15 seconds of drive | **ALLOWED** | In-vehicle mounting matrix calibration while vehicle is in valid GNSS reception. |
| `traj.gt_speed[i0:i1]` | During GNSS-denied outage ($t \ge t_0$) | **STRICTLY PROHIBITED** | Oracle velocity leakage in autonomous dead reckoning. |
| `traj.gt_heading[i0:i1]` | During GNSS-denied outage | **STRICTLY PROHIBITED** | Oracle attitude leakage. |
| `traj.gt_pos[i0:i1]` | During GNSS-denied outage | **STRICTLY PROHIBITED** | Oracle position leakage. |

---

## 5. Permanent No-GT Enforcement Architecture

To guarantee that ground-truth data never enters autonomous production inference:

1. **Dynamic Runtime Traps (`TrajectoryOracleTrap`):**
   ```python
   class TrajectoryOracleTrap:

     def __getattr__(self, name: str):
       if self.inference_active and name in (
           "gt_speed",
           "gt_vel",
           "gt_pos",
           "gt_heading",
       ):
         raise AssertionError(
             f"ORACLE LEAKAGE DETECTED: Inference attempted to read '{name}'!"
         )
       return getattr(self._traj, name)
```
2. **Automated Static Audit in CI/Test Suite:**
   `tests/test_no_ground_truth_in_inference.py` scans all Python production code in `idr_pipeline/` to guarantee zero occurrences of `gt_speed[` or `gt_vel[` during inference.
3. **Hardware Engine Isolation:**
   Neither the C++ core engine (`idr-core`) nor the Android app (`android/`) contains any structures or fields for ground-truth velocity. They accept only physical IMU and GNSS inputs.
