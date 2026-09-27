# FROZEN BASELINE SPECIFICATION: UNIFIED + PHASE 16 RECOVERY
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Authoritative Production Baseline Freeze & Next-Sprint Boundary Contract

**Document ID:** `SPEC-FREEZE-UNIFIED-P16`  
**Effective Date:** September 27, 2026  
**Status:** IMMUTABLE AUTHORITATIVE BASELINE  
**Evaluation Set:** 158 Canonical IO-VNBD Outages across `Driver_A_S3b`, `Vta01b`, `Vta04`  
**Harness Script:** `scripts/run_unified_phase16_master.py`  

---

## 1. Authoritative Frozen Baseline Metrics

Any future iteration, model candidate, or filter enhancement must be ablated directly against these exact numbers:

```
========================================================================================================
                                 FROZEN PRODUCTION BASELINE CONTROL
========================================================================================================
Metric                                     Baseline Value           Validation Rule
--------------------------------------------------------------------------------------------------------
Median Positional Drift (%)                   54.06%                Must improve without subset cherry-picking
Mean Positional Drift (%)                     78.03%                Must not regress catastrophic outlier bound
Benchmark Pass Count (<=10% Drift)         18 / 158 (11.4%)         Must increase or preserve pass rate
Median Along-Track Error (m)                  27.27 m               Longitudinal error metric
Median Cross-Track Error (m)                  41.24 m               Lateral error metric
Median Yaw Error (deg)                        31.22°                Orientation convergence metric
Median Vertical Error (m)                      1.04 m               Barometric altimeter bound
Ground-Truth Leakage Count                     0 / 158              TrajectoryOracleTrap mandatory
Android UI Files Modified                      0                    UI Strictly Frozen
========================================================================================================
```

### Granular Per-Drive Breakdown (Frozen):
- **`Vta04` (Arterial Highway):** Median Drift = **19.63%**, Mean Drift = **24.94%**, Passes = **8 / 31 (25.8%)**, Cross = 20.67 m, Yaw = 9.20°.
- **`Vta01b` (Highway Cruise):** Median Drift = **21.37%**, Mean Drift = **27.53%**, Passes = **5 / 15 (33.3%)**, Along = 15.65 m, Yaw = 8.28°.
- **`Driver_A_S3b` (Urban Roundabouts):** Median Drift = **78.91%**, Mean Drift = **99.58%**, Passes = **5 / 110 (4.5%)**, Along = 32.09 m, Cross = 60.44 m, Yaw = 62.77°.

---

## 2. Frozen Pipeline Architecture

The baseline pipeline topology is frozen. Downstream tasks must NOT alter this structure:

1. **Mounting Alignment:** Gravity vector leveling + PCA forward longitudinal axis.
2. **Causal Pre-Filter:** 4th-order zero-phase Butterworth bandpass (0.1–15 Hz) with shock blanking (>2.5g) and dynamic AC acceleration standstill detection ($\text{std}(a_x, a_y) < 0.18\text{ m/s}^2$).
3. **AI Forward Speed Estimation:** GBDT regressor (120 trees, depth 5, $\eta=0.08$) on 45 sliding-window vibration features.
4. **Pre-Outage Calibration:** Online Recursive Least Squares (`OnlineSpeedScaleBiasCalibrator`) updating scale $\alpha$ and bias $\beta$ during active pre-outage GNSS aiding and frozen at outage onset.
5. **Pre-Outage Observability Handover:** 3.0 s causal window before outage ($[t_0-3\text{s}, t_0]$) using smartphone Doppler velocity from `S-*.csv`.
6. **15-State ESKF:** States $\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{q}, \mathbf{b}_a, \mathbf{b}_g]$.
   - Adaptive AI measurement noise: $R_{\text{AI}} = 0.35 \cdot (1.0 + 0.5 \|\mathbf{a}_{\text{vib}}\| + 1.2 |\omega_z|)$.
   - Bounded-slip lateral NHC: $R_{\text{target}} = 0.05^2 (1 + 25\omega_z^2 + 12 a_{\text{cent}}^2 + 5 a_y^2), R_{\text{max}} = 0.16\text{ m}^2/\text{s}^2$.
   - Standstill ZUPT / ZARU.
   - In-flight straight-cruise Kalman ZARU ($|\omega_z| < 0.005, |a_y| < 0.15, v > 4.0\text{ m/s}$ for $>8$ ticks).
7. **Map Fusion Modality:** Phase-16 soft road heading fusion (`update_road_heading` with top-3 circular weighted azimuth, $45^\circ$ gate, $\chi^2=25.0$) with **zero Cartesian position displacement**.
8. **Android Interface:** JNI bridge calling portable `idr-core/` C++17 library. Zero UI modifications.

---

## 3. Strict Development Scope for Next Sprint

The search space for the next optimization phase is strictly bounded:

### PERMITTED MODIFICATIONS:
1. **Speed Estimator Refinement (Speed Model V2):**
   - Training new model architectures (retrained GBDT, lightweight 1D-CNN / TCN, or ensemble).
   - Frequency-domain FFT vibration peak tracking features.
   - Negative penalty data augmentation on engine idle harmonics to eliminate false positive forward speed.
2. **Gyroscope / Heading Estimation Refinement:**
   - Pre-outage dynamic turn Doppler velocity heading observable calibration.
   - Nonlinear gyro scale-factor compensation during high angular rates ($|\omega_z| > 100^\circ/\text{s}$).

### STRICTLY PROHIBITED (FROZEN):
- **NO Ground-Truth Ingestion:** Ground truth speed (`gt_speed`, CAN, OBD) during outages remains strictly forbidden.
- **NO Position Snapping:** Do not restore Part 2 road-normal position snapping.
- **NO Android UI Changes:** Android UI screens, layouts, and flows remain frozen.
- **NO Baseline Redefinition:** The 158-outage population and metric definitions must not be altered to manufacture artificial improvements.

---

## 4. Mandatory Ablation Protocol for All Next-Sprint Changes

Every future experiment MUST:
1. Be executed through `scripts/run_unified_phase16_master.py`.
2. Evaluate all 158 authentic outages with `TrajectoryOracleTrap` active.
3. Report the exact deltas against this frozen baseline:
   $$\Delta \text{Median Drift} = \text{Candidate Median} - 54.06\%$$
   $$\Delta \text{Mean Drift} = \text{Candidate Mean} - 78.03\%$$
   $$\Delta \text{Pass Count} = \text{Candidate Passes} - 18$$
4. Include per-drive breakdowns for `Driver_A_S3b`, `Vta01b`, and `Vta04`.
