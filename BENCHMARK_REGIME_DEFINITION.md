# BENCHMARK REGIME DEFINITION: HISTORICAL VS. AUTONOMOUS

**Document ID:** `DEF-BENCHMARK-REGIMES-01`  
**Date:** September 27, 2026  
**Status:** MANDATORY CLASSIFICATION STANDARD  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System  
**Authority:** System Architecture & Validation Lead

---

## 1. Context & Motivation

To eliminate any ambiguity or misrepresentation of system performance, the repository establishes two formally segregated benchmark evaluation regimes:

```
+-----------------------------------------------------------------------------------+
| REGIME A: CAN / GT-SPEED-AIDED RESEARCH BENCHMARKS (HISTORICAL)                   |
| Purpose: Evaluates filter mechanization, attitude tracking, and NHC in isolation  |
| Velocity Input: Wheel speed / CAN bus / OBD-II ground-truth velocity (gt_speed)   |
| Status: Historical Research Reference ONLY — NEVER claim as Autonomous DR         |
+-----------------------------------------------------------------------------------+
| REGIME B: AUTONOMOUS IMU-ONLY OUTAGE INFERENCE (PRODUCTION TRUTH)                 |
| Purpose: Evaluates true Dead Reckoning performance without external odometry      |
| Velocity Input: Autonomous Huber GBDT regressor inferenced from 1.0s IMU window   |
| Status: Authoritative Production Benchmark                                        |
+-----------------------------------------------------------------------------------+
```

---

## 2. Regime A: Historical CAN/GT-Speed-Assisted Benchmark

### 2.1 Specification
In Regime A, vehicle longitudinal speed during the GNSS outage is supplied directly from the vehicle's onboard diagnostics / CAN bus / ground-truth odometry (`ai_spd = float(traj.gt_speed[k])`).

### 2.2 Historical Results Under Regime A
Across the 158 authentic IO-VNBD outages:
- **Baseline N0 (Zero NHC):** 76.34% median drift
- **Baseline N7 / B0 (Adaptive NHC):** 35.71% median drift (36 / 158 passes)
- **Phase 14 (A1 Fast Architecture):** 31.65% median drift
- **Phase 15 (Bounded NHC + ZARU):** 30.88% median drift
- **Phase 16 (Doppler Handover + Bounded NHC):** 27.25% median drift (45 / 158 passes)
- **Phase 17 (Dynamic Confidence + Lever Arm):** 27.64% median drift (49 / 158 passes)

### 2.3 Boundary Rules
- **Rule A1:** Any number produced using `traj.gt_speed` during the outage must be prominently labeled:  
  `CAN/GT-SPEED-AIDED RESEARCH (HISTORICAL)`
- **Rule A2:** Regime A results must **NEVER** be presented to ISRO evaluators or external stakeholders as autonomous smartphone IMU dead reckoning.

---

## 3. Regime B: Autonomous IMU-Only Outage Inference

### 3.1 Specification
Regime B represents the operational reality mandated by ISRO PS 26168: a driver mounts a commercial smartphone in an arbitrary vehicle cradle with **no connection to CAN bus, OBD-II, wheel speed sensors, or external infrastructure**.

### 3.2 Permitted Information in Regime B
During the outage ($t \ge t_{\text{outage}}$), the system is strictly limited to:
1. Smartphone tri-axial accelerometer measurements: $a_{\text{phone}}[k]$.
2. Smartphone tri-axial gyroscope measurements: $\omega_{\text{phone}}[k]$.
3. Barometric pressure / altitude measurements: $h_{\text{baro}}[k]$.
4. Learned forward velocity inferenced exclusively from the production Huber GBDT model using a causal 1.0 s sliding IMU window.
5. In-vehicle mounting matrix $R_p^b$ estimated during pre-outage driving.
6. Pre-outage GNSS Doppler velocity vectors acquired while GNSS was valid ($t \in [t_{\text{outage}} - 3.0\text{s}, t_{\text{outage}}]$).
7. Offline OSM road network graph loaded locally on device.
8. Filter state and covariance propagated forward in time.

### 3.3 Strictly Forbidden Information in Regime B
The following inputs, shortcuts, or oracles are strictly prohibited during outage inference:
- Ground-truth speed (`traj.gt_speed`)
- CAN bus / OBD-II wheel speed
- Future GNSS fixes (lookahead)
- Post-outage GNSS reacquisition fixes used retrospectively
- Ground-truth trajectory coordinates (`traj.gt_pos`)
- Ground-truth attitude angles (`traj.gt_roll`, `traj.gt_pitch`, `traj.gt_heading`)
- Centered rolling windows or zero-phase filtering (`filtfilt`)
- Rauch-Tung-Striebel (RTS) or forward-backward batch smoothing
- Full-trip feature normalization

---

## 4. Summary Matrix of Disallowed vs. Allowed Operations

| Operation / Feature | Regime A (CAN-Aided) | Regime B (Autonomous IMU) | Operational Rationale |
|---|:---:|:---:|---|
| **Raw IMU Integration** | Allowed | **Allowed** | Core inertial mechanization |
| **GBDT IMU Speed Regressor** | Allowed | **Allowed** | Causal 45-feature model |
| **Barometer Altitude Update** | Allowed | **Allowed** | Bounded vertical drift |
| **Offline OSM Heading Fusion** | Allowed | **Allowed** | Pre-downloaded map cache |
| **Pre-Outage Doppler Handover** | Allowed | **Allowed** | GNSS is valid prior to outage |
| **Standstill ZUPT (<0.2 m/s)** | Allowed | **Allowed** | Idle vibration detection |
| **In-Flight Gyro ZARU** | Allowed | **Allowed** | Causal straight-line detection |
| **OBD-II / CAN Speed Input** | **ALLOWED** | **STRICTLY FORBIDDEN** | PS 26168 forbids OBD-II |
| **Ground-Truth Speed (`gt_speed`)** | **ALLOWED** | **STRICTLY FORBIDDEN** | Oracle leakage |
| **Future Window Lookahead** | **FORBIDDEN** | **STRICTLY FORBIDDEN** | Non-causal |
| **Post-Outage RTS Smoothing** | **FORBIDDEN** | **STRICTLY FORBIDDEN** | Offline-only post-processing |
