# AUTHORITATIVE PRODUCTION FEATURE SPECIFICATION

**Document ID:** `SPEC-FEATURE-AUTHORITATIVE-01`  
**Date:** September 27, 2026  
**Status:** FROZEN PRODUCTION STANDARD  
**Model Identity:** Top-45 Huber GBDT Speed Regressor  
**Artifact Path:** [`models/production/gbdt_speed_model_iovnbd_top45.joblib`](file:///Users/akashmishra/SIH%20MOdel/models/production/gbdt_speed_model_iovnbd_top45.joblib)  
**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Overview & Causality Guarantees

All 45 features are computed from a strictly backward sliding window of vehicle body-frame acceleration and angular rates:
- **Window Length ($W$):** 10 samples (1.00 second at 10 Hz)
- **Stride ($S$):** 1 sample (0.10 second per step)
- **Temporal Causality:** For step $i$, window spans indices $[\max(0, i - W + 1), i + 1)$. **Zero future samples are ever accessed**.
- **Preprocessing:** Accelerometer and gyroscope data are transformed from phone frame $p$ to vehicle body frame $b$ via the mounting matrix:
  $$a_b = R_p^b a_p, \quad \omega_b = R_p^b \omega_p$$

---

## 2. Definitive Top-45 Feature Ledger

| Model Index | Base Index | Feature Name | Physical Dimension / Units | Source Sensor | Mathematical Formulation | Causal Availability |
|:---:|:---:|---|:---:|:---:|---|:---:|
| 0 | 5 | `std_az` | $\text{m/s}^2$ | Accel Z | Standard deviation of vertical body accel | Instantaneous past |
| 1 | 16 | `std_gy` | $\text{rad/s}$ | Gyro Y | Standard deviation of pitch angular rate | Instantaneous past |
| 2 | 32 | `roughness` | $(\text{m/s}^2)^2$ | Accel 3D | High-frequency acceleration variance | Instantaneous past |
| 3 | 49 | `std_gyro_rate_gx` | $\text{rad/s}^2$ | Gyro X | Standard deviation of roll angular jerk | Instantaneous past |
| 4 | 52 | `std_gyro_rate_gy` | $\text{rad/s}^2$ | Gyro Y | Standard deviation of pitch angular jerk | Instantaneous past |
| 5 | 7 | `rms_ay` | $\text{m/s}^2$ | Accel Y | Root mean square of lateral acceleration | Instantaneous past |
| 6 | 15 | `std_gx` | $\text{rad/s}$ | Gyro X | Standard deviation of roll rate | Instantaneous past |
| 7 | 55 | `std_gyro_rate_gz` | $\text{rad/s}^2$ | Gyro Z | Standard deviation of yaw angular jerk | Instantaneous past |
| 8 | 20 | `rms_gz` | $\text{rad/s}$ | Gyro Z | Root mean square of yaw rate | Instantaneous past |
| 9 | 11 | `ptp_az` | $\text{m/s}^2$ | Accel Z | Peak-to-peak amplitude of vertical accel | Instantaneous past |
| 10 | 53 | `rms_gyro_rate_gy` | $\text{rad/s}^2$ | Gyro Y | RMS of pitch angular jerk | Instantaneous past |
| 11 | 37 | `mean_tilt_rate` | $\text{rad/s}$ | Gyro 2D | Mean off-vertical rotational rate | Instantaneous past |
| 12 | 56 | `rms_gyro_rate_gz` | $\text{rad/s}^2$ | Gyro Z | RMS of yaw angular jerk | Instantaneous past |
| 13 | 1 | `mean_ay` | $\text{m/s}^2$ | Accel Y | Mean lateral acceleration | Instantaneous past |
| 14 | 25 | `sma_gyr` | $\text{rad/s}$ | Gyro 3D | Signal magnitude area of angular velocity | Instantaneous past |
| 15 | 18 | `rms_gx` | $\text{rad/s}$ | Gyro X | Root mean square of roll rate | Instantaneous past |
| 16 | 33 | `dyn_acc_norm` | $\text{m/s}^2$ | Accel 3D | Norm of dynamic acceleration minus gravity | Instantaneous past |
| 17 | 40 | `std_jerk_ax` | $\text{m/s}^3$ | Accel X | Standard deviation of longitudinal jerk | Instantaneous past |
| 18 | 38 | `pitch_sin_hint` | dimensionless | Accel X | Sinusoid of estimated vehicle road pitch | Instantaneous past |
| 19 | 13 | `mean_gy` | $\text{rad/s}$ | Gyro Y | Mean pitch angular rate | Instantaneous past |
| 20 | 43 | `std_jerk_ay` | $\text{m/s}^3$ | Accel Y | Standard deviation of lateral jerk | Instantaneous past |
| 21 | 19 | `rms_gy` | $\text{rad/s}$ | Gyro Y | Root mean square of pitch rate | Instantaneous past |
| 22 | 24 | `sma_acc` | $\text{m/s}^2$ | Accel 3D | Signal magnitude area of acceleration | Instantaneous past |
| 23 | 17 | `std_gz` | $\text{rad/s}$ | Gyro Z | Standard deviation of yaw rate | Instantaneous past |
| 24 | 41 | `rms_jerk_ax` | $\text{m/s}^3$ | Accel X | RMS of longitudinal jerk | Instantaneous past |
| 25 | 47 | `rms_jerk_az` | $\text{m/s}^3$ | Accel Z | RMS of vertical jerk | Instantaneous past |
| 26 | 22 | `ptp_gy` | $\text{rad/s}$ | Gyro Y | Peak-to-peak amplitude of pitch rate | Instantaneous past |
| 27 | 50 | `rms_gyro_rate_gx` | $\text{rad/s}^2$ | Gyro X | RMS of roll angular jerk | Instantaneous past |
| 28 | 46 | `std_jerk_az` | $\text{m/s}^3$ | Accel Z | Standard deviation of vertical jerk | Instantaneous past |
| 29 | 0 | `mean_ax` | $\text{m/s}^2$ | Accel X | Mean longitudinal acceleration | Instantaneous past |
| 30 | 12 | `mean_gx` | $\text{rad/s}$ | Gyro X | Mean roll angular rate | Instantaneous past |
| 31 | 8 | `rms_az` | $\text{m/s}^2$ | Accel Z | Root mean square of vertical acceleration | Instantaneous past |
| 32 | 3 | `std_ax` | $\text{m/s}^2$ | Accel X | Standard deviation of longitudinal accel | Instantaneous past |
| 33 | 36 | `std_yaw_rate` | $\text{rad/s}$ | Gyro Z | Standard deviation of gravity-projected yaw | Instantaneous past |
| 34 | 29 | `vib_ratio` | dimensionless | Accel 3D | Ratio of vertical to horizontal vibration | Instantaneous past |
| 35 | 35 | `mean_yaw_rate` | $\text{rad/s}$ | Gyro Z | Mean gravity-projected yaw rate | Instantaneous past |
| 36 | 21 | `ptp_gx` | $\text{rad/s}$ | Gyro X | Peak-to-peak amplitude of roll rate | Instantaneous past |
| 37 | 44 | `rms_jerk_ay` | $\text{m/s}^3$ | Accel Y | RMS of lateral jerk | Instantaneous past |
| 38 | 23 | `ptp_gz` | $\text{rad/s}$ | Gyro Z | Peak-to-peak amplitude of yaw rate | Instantaneous past |
| 39 | 34 | `norm_vib_energy` | $(\text{m/s}^2)^2$ | Accel 3D | Normalized road vibration energy envelope | Instantaneous past |
| 40 | 14 | `mean_gz` | $\text{rad/s}$ | Gyro Z | Mean yaw angular rate | Instantaneous past |
| 41 | 2 | `mean_az` | $\text{m/s}^2$ | Accel Z | Mean vertical acceleration | Instantaneous past |
| 42 | 28 | `var_horiz` | $(\text{m/s}^2)^2$ | Accel 2D | Horizontal plane dynamic variance | Instantaneous past |
| 43 | 51 | `mean_gyro_rate_gy` | $\text{rad/s}^2$ | Gyro Y | Mean pitch angular jerk | Instantaneous past |
| 44 | 48 | `mean_gyro_rate_gx` | $\text{rad/s}^2$ | Gyro X | Mean roll angular jerk | Instantaneous past |

---

## 3. Standardization & Normalization Pipeline

```
Feature Vector X_45
       │
       ▼
[StandardScaler Normalization]
  x_norm[k] = (X_45[k] - mean[k]) / scale[k]
       │
       ▼
[Huber GBDT Inference: 120 Trees]
  v_raw = sum(tree[j].predict(x_norm))
       │
       ▼
[Physiological Leaf Clamping]
  v_ai = clip(v_raw, 0.0 m/s, 35.0 m/s)  (0 to 126 km/h)
```
- **Integrity Guarantee:** Zero ground-truth velocity, CAN bus speed, or wheel sensor input is ingested anywhere in this pipeline.
