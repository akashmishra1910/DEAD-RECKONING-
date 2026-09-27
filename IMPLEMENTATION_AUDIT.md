# IMPLEMENTATION AUDIT: TRACK B FINAL ARCHITECTURE

**Document ID:** `AUDIT-TRACK-B-FINAL-01`  
**Date:** September 27, 2026  
**Lead Systems Engineer:** Autonomous Navigation Engineering Team  
**Compliance Standard:** ISRO PS 26168 / Rule Zero (Scientific and Production Integrity)

---

## 1. Executive Summary

This forensic audit evaluates the entire Dead Reckoning / SIH 2026 repository across C++17 (`idr-core`), standalone edge binary (`idr-cli`), Android production application (`android/`), and Python algorithmic pipelines (`idr_pipeline/`, `scripts/`).

The audit specifically assesses:
1. Production entry points, ESKF mechanization, sensor preprocessing, and alignment.
2. Machine learning speed regressors, velocity models, and covariance formulations.
3. Causal constraints (adaptive NHC, ZUPT, ZARU, 3D map matching, and GNSS handover).
4. **Oracle and lookahead risk identification** — uncovering ground-truth speed leakage in historical benchmark scripts.
5. Exact file impact matrix defining code to modify, code to leave untouched, and the target Track B architecture.

---

## 2. Existing Architecture & Production Entry Points

### 2.1 Component Breakdown

```
┌────────────────────────────────────────────────────────────────────────┐
│                        DATA INGESTION & SENSORS                        │
│  Smartphone IMU (100 Hz), External FOG IMU (~200 Hz), GNSS / Barometer  │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                  STAGE 1: VEHICLE-FRAME ALIGNMENT                      │
│  Gravity Leveling (Roll/Pitch) + PCA Longitudinal Axis (Yaw)           │
│  Outputs mounting matrix R_p^b; Angular-rate watchdog monitors slip    │
│  C++: alignment_engine.cpp / Python: alignment_engine.py               │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│               STAGE 2: VIBRATION & PRE-FILTERING                       │
│  2nd-order Butterworth low-pass, Pothole impulse detection,            │
│  Idling standstill discriminator (var < 0.008 m²/s⁴)                   │
│  C++: vibration_filter.cpp / Python: vibration_filter.py               │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│              STAGE 3: LEARNED MOTION ESTIMATION & SCALING              │
│  Historical: 120-tree Huber GBDT predicting forward speed v_x          │
│  Online RLS calibrator adjusts v_corr = α·v_ai + β                     │
│  C++: gbdt_model.cpp, feature_extractor.cpp / Python: ai_velocity_model│
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│              STAGE 4: 15-STATE ERROR-STATE KALMAN FILTER               │
│  State: δx = [δp (3), δv (3), δθ (3), δb_a (3), δb_g (3)]ᵀ ∈ ℝ¹⁵       │
│  Updates: AI velocity, Adaptive NHC (v_y≈0, v_z≈0), ZUPT, ZARU         │
│  C++: eskf.cpp / Python: eskf_fusion.py                                │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│             STAGE 5: 3D HMM MAP-MATCHING & GNSS SUPERVISOR             │
│  Viterbi emission (|Δh| > 4.0 m barrier), Soft heading/cross-track aid  │
│  6-State FSM: INITIALIZING, GNSS_AIDED, DEGRADED, DR_ONLY, REACQUIRED  │
│  C++: map_matcher_3d.cpp, navigation_engine.cpp                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Entry Points
1. **Core C++ Library (`idr-core`)**:
   - `SeamlessGNSSDeficitEngine::process_step(...)` in [`navigation_engine.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-core/src/navigation_engine.cpp#L161).
   - `EdgeNavigationEngine::update(...)` in [`edge_engine.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-core/src/edge_engine.cpp#L57).
   - C ABI in [`c_api.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-core/src/c_api.cpp).
2. **Standalone Edge CLI (`idr-cli`)**:
   - Entry point in [`main.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-cli/main.cpp#L26), handling CSV streaming up to 200 Hz.
3. **Android Application (`android/`)**:
   - JNI bridge consuming 10 Hz sensor buffers into the C++ engine.
4. **Python Benchmark & Research Engine (`idr_pipeline/`, `scripts/`)**:
   - Evaluation scripts over the 158 authentic outages from `real_iovnbd_loader.py` and `outage_catalog.py`.

---

## 3. Forensic Identification of Oracle & Lookahead Leakage

### 3.1 The `gt_speed` Oracle Leak in Intermediate Python Scripts
During forensic inspection of `scripts/run_final_track_b_benchmark.py`, `scripts/run_phase17_final_benchmark.py`, and `scripts/run_phase16_final_benchmark.py`, a critical flaw was identified:
```python
# Causal outage propagation loop
for k in range(i0, i1):
    ...
    ai_spd = float(traj.gt_speed[k])  # <-- CRITICAL FLAW: GROUND-TRUTH LEAKAGE!
    ...
    # Standstill ZUPT
    if vib_state.is_engine_idling or (ai_spd < 0.2):
        engine.eskf.update_zupt(r_zupt=0.01)
```

In these benchmark scripts, the ground-truth speed from OBD-II (`traj.gt_speed`) was passed directly into the filter update rather than running inference with the trained model on causal IMU sliding windows!

### 3.2 Where C++ Already Did It Right
In contrast, [`idr-cli/main.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-cli/main.cpp#L71-L77) and [`idr-core/src/feature_extractor.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-core/src/feature_extractor.cpp) strictly push IMU samples into a causal sliding window, extract 45 features, and call `gbdt.predict(feats)`.

### 3.3 Remedy Mandate
1. Create a permanent automated verification test: `tests/test_no_ground_truth_in_inference.py`.
2. Ensure every Python and C++ inference loop uses only the trained causal motion model operating on causal IMU windows $[t - W, t]$.
3. Forbid any reference to `gt_speed`, `gt_vel`, `gt_pos`, or future trajectory data during inference.

---

## 4. Authoritative Frozen Baseline B0

The authoritative frozen production baseline is **N7** (Bounded Adaptive NHC + Causal Multi-Hypothesis Map Fusion), verified across all 158 authentic outages:

- **Benchmark Dataset:** IO-VNBD (5 naturalistic drives: `Driver_A_S3b`, `Vta01b`, `Vta04`, `Vta03`, `Vta05`).
- **Total Outages:** 158 (81 @ 10s, 39 @ 30s, 25 @ 60s, 13 @ 120s).
- **Median Drift:** **35.71%**
- **Mean Drift:** **45.86%**
- **P90 Drift:** **105.58%**
- **P95 Drift:** **133.71%**
- **Pass Count (<10%):** **36 / 158 (22.78%)**
- **Worst Outage:** **173.20%**
- **Straight Slices (N=12):** Median 29.81%, Pass 4/12 (33.33%)
- **Curved Slices (N=146):** Median 35.71%, Pass 32/146 (21.92%)

*This baseline is frozen and preserved in `BASELINE_B0_RESULTS.json` and `BASELINE_B0_REPORT.md`.*

---

## 5. Target Track B Architecture

```text
                 GNSS
                   │
                   │
IMU ───────────────┼────────────── Barometer
 │                 │
 ▼                 ▼
Vehicle-frame alignment (Gravity Leveling + Longitudinal PCA)
 │
 ▼
Causal temporal feature extraction (Causal sliding window [t-W, t])
 │
 ▼
Causal learned motion estimator (TCN / GRU / Multi-Output Model)
 │
 ├───────────────────────────────┬──────────────────────────────┐
 ▼                               ▼                              ▼
v_x, v_y, v_z              log_sigma_v                    bias correction (conservative)
 │                               │                              │
 └───────────────────────────────┴──────────────────────────────┘
                                 │
                                 ▼
                     15-State ESKF Mechanization
                      + 3D AI Velocity Measurement
                      + Dynamic Heteroscedastic Covariance R_v
                      + Statistical Innovation Gating (NIS <= 14.16)
                                 │
                 ┌───────────────┼───────────────┐
                 ▼               ▼               ▼
          Adaptive NHC         ZUPT            ZARU
          (Smooth lateral   (Standstill     (Kinematic
           variance)         lock)           straight lock)
                 │               │               │
                 └───────────────┼───────────────┘
                                 ▼
               Conservative Causal 3D Map Matching
               (OSM Road Heading & Elevation Barrier |Δh| > 4m)
                                 │
                                 ▼
                 Position / Velocity / Heading
```

---

## 6. Files Impact Matrix

### 6.1 Files to Modify / Create
| File Path | Action | Description |
|---|:---:|---|
| `IMPLEMENTATION_AUDIT.md` | CREATE | This repository audit document |
| `BASELINE_B0_RESULTS.json` | CREATE | Machine-readable frozen B0 baseline metrics |
| `BASELINE_B0_REPORT.md` | CREATE | Human-readable frozen B0 report |
| `idr_pipeline/causal_motion_model.py` | CREATE | Causal learned motion estimator predicting $[v_x, v_y, v_z, \log\sigma^2]$ with heteroscedastic NLL |
| `scripts/train_velocity_model.py` | CREATE | Reproducible, strictly drive-isolated training pipeline for the learned 3D motion estimator |
| `models/causal_velocity_model.json` | CREATE | Serialized learned model weights & architecture specification |
| `models/velocity_scaler.json` | CREATE | Feature normalization scalers |
| `models/velocity_model_metadata.json` | CREATE | Training provenance, drive splits, and validation metrics |
| `tests/test_no_ground_truth_in_inference.py` | CREATE | Permanent regression firewall test prohibiting oracle inputs |
| `tests/test_strict_causality.py` | CREATE | Permanent regression test verifying anti-lookahead invariance |
| `scripts/evaluate_candidate.py` | CREATE | Automated regression firewall comparing any candidate objectively against B0 |
| `scripts/run_final_track_b_benchmark.py` | REFACTOR | Remove `gt_speed` oracle path; plug in genuine causal learned model |
| `TRACK_B_FINAL_REPORT.md` | CREATE | Final technical evaluation report with candidate decision |
| `TRACK_B_COMPARISON.json` | CREATE | Objective B0 vs Final comparison ledger |

### 6.2 Files That Must Remain Unchanged
| File Path | Rationale |
|---|---|
| `idr_pipeline/eskf_fusion.py` | Core 15-state ESKF quaternion error formulation is mathematically validated |
| `idr_pipeline/alignment_engine.py` | Validated gravity leveling and PCA mounting matrix estimator |
| `idr_pipeline/vibration_filter.py` | Validated Butterworth filtering and standstill/idling detector |
| `idr_pipeline/map_matcher_3d.py` | Validated 3D HMM Viterbi matcher with $|\Delta h| > 4.0\text{ m}$ elevation barrier |
| `idr_pipeline/offline_osm_graph.py` | Validated multi-layer topological graph representation |
| `idr_pipeline/outage_catalog.py` | Authoritative definition of 158 authentic outages |
| `idr_pipeline/real_iovnbd_loader.py` | Generic authentic IO-VNBD dataset loader |
| `idr-core/*` | High-performance C++17 core engine |
