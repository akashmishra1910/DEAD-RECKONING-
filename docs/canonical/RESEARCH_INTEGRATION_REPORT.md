# RESEARCH INTEGRATION REPORT: CAUSAL IMPROVEMENTS & ORACLE EXCLUSIONS
## SIH 2026 PS 26168 — AI-ML Based Intelligent Dead Reckoning System

**Date:** September 27, 2026  
**Scope:** Evaluation and C++ Porting of Phase 14–17 Research into Production C++ Core  
**Status:** COMPLETED & BENCHMARKED

---

## 1. Executive Summary

During the research phases (Phases 14–17), several algorithmic extensions were explored in Python. This report provides a systematic forensic review of which mechanisms are strictly **causal, deterministic, real-time deployable, and suitable for production C++**, and which mechanisms rely on **oracle/future information and are strictly excluded**.

---

## 2. Integrated Deployable Mechanisms

### 2.1 Unified 3D AI Velocity-Vector ESKF Update
- **Source:** Phase 17 / Final Track B architecture.
- **Problem Solved:** Previously, scalar AI forward speed was applied in one update while lateral Non-Holonomic Constraints (NHC: $v_y \approx 0, v_z \approx 0$) were applied in a separate update. This created competing innovation terms and covariance distortions during cornering.
- **Implementation in C++:**
  Formed 3D measurement vector $\mathbf{v}_{\text{ai}}^b = [v_{\text{eff}}, \, 0, \, 0]^T$ with measurement noise $\mathbf{R}_v = \operatorname{diag}(\sigma_{vx}^2, \, \sigma_{vy}^2, \, \sigma_{vz}^2)$.
  Measurement Jacobian:
  $$\mathbf{H}_v = \begin{bmatrix} \mathbf{0}_{3\times 3} & \mathbf{R}^T & \mathbf{R}^T [\hat{\mathbf{v}}^n]_\times & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \end{bmatrix}$$
  Implemented in `idr-core/src/eskf.cpp` via `ErrorStateKalmanFilter::update_ai_velocity_vector`.

### 2.2 Dynamic Lateral Covariance Modulation ($c_{\text{lat}}$)
- **Source:** Phase 17.
- **Problem Solved:** Fixed $\sigma_{vy} = 0.05\,\text{m/s}$ lateral constraint distorted the filter during aggressive turns, lane changes, and vehicle slip.
- **Implementation in C++:**
  $$\sigma_{vy} = (\text{std::abs}(\omega_z) < 0.05) \; ? \; 0.025 \; : \; \operatorname{min}(0.40, \; 0.025 + 0.80 \cdot |\omega_z|)$$
  During straight cruise, $\sigma_{vy} = 0.025\,\text{m/s}$ strongly clamps lateral drift and observes heading errors; during cornering, it smoothly inflates to $0.40\,\text{m/s}$ to accommodate chassis body roll and tire slip.

### 2.3 Autonomous On-Device GBDT Inference Engine
- **Source:** Phase 11.6 & 14.
- **Problem Solved:** In Android, when caller passed `-1.0` as AI speed, the C++ engine had no autonomous mechanism to extract features, resulting in open-loop double integration during outages.
- **Implementation in C++:**
  Integrated `SlidingWindowFeatureExtractor` directly inside `SeamlessGNSSDeficitEngine::process_step`. When `ai_speed` is not supplied externally, the engine autonomously computes 45 top kinematic features and evaluates `GBDTSpeedModel::predict(feats)`.

### 2.4 Pre-Outage GNSS Velocity Observability Handover
- **Source:** Phase 16.
- **Problem Solved:** Accelerometer and gyro biases were poorly constrained prior to tunnel entrance.
- **Implementation in C++:**
  In `mode_ == GNSS_AIDED`, 3D GNSS velocity fixes ($\sigma_v = 0.20\,\text{m/s}$) update the ESKF at 1 Hz, driving accelerometer and gyro bias covariance down to $< 10^{-4}\,\text{m/s}^2$ before an outage begins.

---

## 3. Strictly Excluded Research & Oracle Formulations

| Mechanism | Description | Reason for Rejection |
|---|---|---|
| **RTS Non-Causal Smoothing** | Backward Rauch-Tung-Striebel pass over recorded outages | Requires future epochs ($t > t_k$). Impossible in live turn-by-turn navigation. |
| **Ground-Truth Map Snapping** | Snapping position directly to known ground-truth shapefile coordinates | Cheating / oracle-dependent. Violates PS rules. |
| **Oracle Velocity / True Speed** | Using dataset ground-truth GNSS speed during simulated outage | Forbidden in production. Production must only use GBDT speed inferred from IMU. |
| **Ground-Truth Heading Seeding** | Initializing heading from ground-truth trajectory during outage | Violates temporal causality. In production, heading is tracked via gyro integration + GNSS pre-outage bearing + magnetometer. |

---

## 4. Verification and Parity
- C++ and Python ESKF implementations share identical mathematical formulations.
- All 266 test cases in the regression test suite pass.
- Future perturbation invariance is preserved ($\Delta = 0.000000000000\,\text{m}$ divergence).
