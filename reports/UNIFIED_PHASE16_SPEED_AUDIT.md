# UNIFIED PHASE 16 AI SPEED ESTIMATOR AUDIT
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Comprehensive Empirical Characterization of Forward Velocity Estimation across Authentic IO-VNBD Regimes

**Document ID:** `AUDIT-UNIFIED-SPEED-V1`  
**Date:** September 27, 2026  
**Auditor:** Senior Navigation Systems Engineer, ISRO-SAC Dead Reckoning Team  
**Evaluation Dataset:** 9,458 Authentic 10 Hz Samples across IO-VNBD (`Driver_A_S3b`, `Vta01b`, `Vta04`)  
**Firewall Status:** Strict Zero-GT Inference Verified  

---

## 1. Executive Summary

In dead-reckoning navigation, positional displacement along the corridor is governed by the time integral of forward velocity:
$$p_{\text{along}}(t) = p_{\text{along}}(t_0) + \int_{t_0}^t v_{\text{fwd}}(\tau) \, d\tau$$
When external GNSS is denied and vehicle wheel odometry / CAN bus is prohibited (PS 26168 constraint), forward speed must be estimated solely from smartphone IMU accelerations and angular rates.

This audit quantitatively evaluates the production GBDT speed regressor (`gbdt_speed_model_iovnbd_top45.joblib`) against ground truth across all operational dynamics, evaluates the online pre-outage RLS speed calibrator, and determines whether an architectural upgrade (Speed Model V2) is scientifically justified.

---

## 2. Speed Regressor Architecture & Feature Formulation

The baseline velocity model is a Gradient Boosted Decision Tree (GBDT) regressor:
- **Ensemble:** 120 trees, max depth 5, learning rate $\eta = 0.08$.
- **Input Features:** 45 selected features extracted from a 1.0 s sliding window (10 samples @ 10 Hz) of phone-to-vehicle aligned acceleration and angular velocity.
- **Key Feature Domains:**
  - Acceleration statistics: mean, standard deviation, root-mean-square (RMS).
  - High-frequency chassis vibration energy: $E_{\text{vib}} = \text{mean}((a_x - \bar{a}_x)^2 + (a_z - \bar{a}_z)^2)$.
  - Centripetal coupling proxy: $a_y \approx v \cdot \omega_z$.
  - Jerk variance: $\text{var}(\Delta a / \Delta t)$.
- **Output:** Forward velocity $v_{\text{fwd}} \in [0.0, 35.0]\text{ m/s}$.

---

## 3. Stratified Performance Breakdown across Dynamic Regimes

Evaluating the model across 9,458 ground-truth time steps reveals severe regime-dependent disparities:

| Operating Dynamic Regime | Ground Truth Speed Range | Sample Count | Share (%) | MAE (m/s) | RMSE (m/s) | Mean Bias (m/s) | Relative Error (%) |
|---|---|---|---|---|---|---|---|
| **True Standstill / Red Light** | $v < 0.2\text{ m/s}$ | 494 | 5.2% | **3.55 m/s** | **4.12 m/s** | **+3.55 m/s** | $+\infty$ (Phantom Motion) |
| **Low-Speed Crawl** | $0.2 \le v < 1.4\text{ m/s}$ | 212 | 2.2% | **4.08 m/s** | **4.65 m/s** | **+3.92 m/s** | $+480\%$ Overestimate |
| **Urban Low Speed** | $1.4 \le v < 5.0\text{ m/s}$ | 1,280 | 13.5% | **5.42 m/s** | **6.10 m/s** | **+4.85 m/s** | $+155\%$ Overestimate |
| **Arterial Medium Speed** | $5.0 \le v < 15.0\text{ m/s}$ | 6,825 | 72.2% | **3.31 m/s** | **4.18 m/s** | **+1.98 m/s** | $+22\%$ Overestimate |
| **Highway Cruise** | $v \ge 15.0\text{ m/s}$ | 647 | 6.8% | **4.01 m/s** | **4.82 m/s** | **-3.12 m/s** | $-16\%$ Underestimate |
| **Overall Dataset** | $0.0 \le v \le 22.0\text{ m/s}$ | 9,458 | 100.0% | **3.70 m/s** | **4.76 m/s** | **+2.55 m/s** | $+31.4\%$ Mean Bias |

```
                       ========================================
                       PREDICTED VS. ACTUAL FORWARD SPEED BIAS
                       ========================================

    Predicted Speed (m/s)
      ^
 20.0 |                                                * (Under-prediction)
      |                                           *
 15.0 |                                      *
      |                                 *
 10.0 |                            *
      |                       *
  5.0 |                  *
      |   (Phantom Walk)
  3.5 | + - - - - - - - * (Over-prediction at zero speed)
  0.0 +---+-------------+-------------+-------------+-------------> True Speed (m/s)
         0.0           5.0           10.0          15.0          20.0
```

---

## 4. Multi-Drive Forensic Diagnostics

Comparing the speed estimation performance across individual drives isolates the environmental conditions driving the error:

| Forensic Parameter | Driver_A_S3b | Vta01b | Vta04 | Physical Reason |
|---|---|---|---|---|
| **Total Samples** | 6,768 | 951 | 1,739 | Recording length |
| **Overall MAE** | **3.87 m/s** | 3.29 m/s | 3.28 m/s | Higher error in urban route |
| **Overall RMSE** | **4.97 m/s** | 3.89 m/s | 4.24 m/s | Large positive error spikes |
| **Mean Bias** | **+2.89 m/s** | **-0.06 m/s** | **+2.58 m/s** | Urban routes suffer positive bias |
| **Stopped Share** | **7.2% (488 ticks)** | **0.6% (6 ticks)** | **0.0% (0 ticks)** | Frequency of traffic stops |
| **Stopped Bias** | **+3.57 m/s (12.8 km/h)** | +2.01 m/s | N/A (continuous) | Engine idle vibration bleed |
| **Crawl MAE ($0.2\text{--}1.4\text{ m/s}$)** | **4.11 m/s** | 2.49 m/s | N/A | Low signal-to-noise ratio |
| **Model $R^2$ Score** | **-1.74** | **+0.33** | **-2.36** | Poor baseline generalization |

---

## 5. Physical Root Cause: The Engine Idle Vibration Artifact

The dominant failure mode in the current GBDT regressor is the **false positive speed prediction during stationary idling**:
1. When a vehicle stops at a traffic intersection, the internal combustion engine continues idling at 700–900 RPM (12–15 Hz fundamental frequency).
2. The smartphone's MEMS accelerometer measures continuous chassis vibration ($E_{\text{vib}} \approx 0.72\text{ m/s}^2$).
3. Because the GBDT regressor was trained on highway driving segments where high vibration energy correlated with high speed, the decision trees route stationary vibration samples to high-velocity leaf nodes, predicting **$+3.57\text{ m/s}$ ($12.8\text{ km/h}$)**.
4. Over a typical 10-second traffic stop, integrating this phantom speed accumulates **$35.7\text{ m}$ of longitudinal drift** while the car is physically motionless.

---

## 6. Pre-Outage RLS Speed Calibrator Evaluation

To mitigate this systematic speed bias without ground-truth leakage, an online Recursive Least Squares (RLS) calibrator was deployed (`OnlineSpeedScaleBiasCalibrator`):
- **Model:** $v_{\text{corr}} = \alpha \cdot v_{\text{ai}} + \beta$
- **Operation:** During the 30 seconds of active GNSS preceding an outage, the calibrator compares smartphone Doppler speed against GBDT predictions, converging $\alpha \in [0.70, 1.35]$ and $\beta \in [-3.5, +3.5]\text{ m/s}$.
- **Outage Freeze:** At outage onset $t_0$, the calibrator freezes $\hat{\alpha}(t_0)$ and $\hat{\beta}(t_0)$ and causally applies them during DR propagation.

### Empirical Calibration Results across 158 Outages (`EXP_SPEED_CALIB`):
- **Median Along-Track Error:** Dropped from **32.56 m** to **26.66 m** (18.1% improvement).
- **Mean Outage Drift (%):** Dropped from **126.80%** to **94.16%** (32.64 percentage point reduction in catastrophic outliers).
- **Pass Count ($\le 10\%$):** Increased from **11 / 158** to **14 / 158**.

---

## 7. Justification for Speed Model V2

The counterfactual diagnostics (`DIAG_B_GT_SPD_EST`) prove that replacing the autonomous GBDT speed with perfect forward speed drops median drift from **54.89%** to **29.85%** (a 45.6% relative improvement).

Therefore, developing **Speed Model V2** is fully justified. The design specifications for Speed Model V2 must include:
1. **Engine Idle Negative Penalty:** Explicit synthetic stationary data augmentation ($v_{\text{gt}} = 0.0$ paired with idle vibration spectra) during training to force zero output under stationary harmonics.
2. **Dynamic High-Pass Acceleration Features:** Demeaning sliding windows to eliminate windshield cradle mounting tilt.
3. **Rotational Invariance:** Training with random mounting rotation perturbations so speed inference is decoupled from cradle orientation.
4. **Frequency-Domain Peak Tracking:** Extracting dominant spectral peak frequency via FFT to track wheel-rotation rate directly from road noise.
