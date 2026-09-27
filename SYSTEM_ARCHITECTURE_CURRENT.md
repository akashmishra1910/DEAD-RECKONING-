# Current System Architecture Baseline: B1/N8 Autonomous Engine
## SIH 2026 / ISRO-SAC PS 26168

**Document ID:** `ARCH-B1N8-CURRENT-01`  
**Date:** September 27, 2026  
**Status:** VALIDATED PRODUCTION BASELINE (GOLDEN REFERENCE)

---

## 1. System Overview

The **B1/N8 Autonomous Dead Reckoning Engine** is a fully causal, on-device navigation system that fuses smartphone inertial (IMU) and barometric data with offline multi-layer OpenStreetMap (OSM) topological networks during GNSS denial. It operates strictly without ground-truth speed, CAN bus, or future lookahead.

```
                    ┌─────────────────────────┐
                    │      SMARTPHONE IMU     │
                    │  Accel (100Hz) / Gyro   │
                    └───────────┬─────────────┘
                                │
                                ↓
                    ┌─────────────────────────┐
                    │ Phone→Vehicle Alignment │
                    │ Gravity Leveling + PCA  │
                    └───────────┬─────────────┘
                                │
                                ↓
                    ┌─────────────────────────┐
                    │ Vibration Pre-Filter    │
                    │ 2nd-order Butterworth   │
                    └──────┬────────────┬─────┘
                           │            │
                           ↓            ↓
                    ┌────────────┐ ┌───────────────┐
                    │ Top-45     │ │ Pothole /     │
                    │ Huber GBDT │ │ Idle Detector │
                    └──────┬─────┘ └────────┬──────┘
                           │                │
                     Causal Speed      Idle Flag
                           │                │
                           └───────┬────────┘
                                   │
                                   ↓
                    ┌─────────────────────────┐
                    │ Pre-Outage Doppler Cal  │
                    │ Scale / Bias Correction │
                    └──────────────┬──────────┘
                                   │
                                   ↓
                    ┌─────────────────────────┐
                    │ 15-State Error-State KF │
                    │ p, v, q, b_a, b_g       │
                    └──────┬─────┬──────┬─────┘
                           │     │      │
                           │     │      └─────────┐
                           ↓     ↓                ↓
                     Bounded   Standstill    In-Flight Straight
                     Slip NHC    ZUPT           Kalman ZARU
                     (R_max)  (v < 0.2)      (|w_z| < 0.005)
                           │     │                │
                           └─────┼────────────────┘
                                 │
                                 ↓
                    ┌─────────────────────────┐
                    │ Causal 3D HMM Map Match │
                    │ Elevation Barrier |dh|>4│
                    └────────────┬────────────┘
                                 │
                                 ↓
                    ┌─────────────────────────┐
                    │ Authoritative Telemetry │
                    │ Pos, Vel, Att, Cov, Conf│
                    └─────────────────────────┘
```

---

## 2. Mathematical Pipeline Formulation

### 2.1 State Representation ($x \in \mathbb{R}^{15}$)
The continuous-time state vector consists of:
$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^n & \mathbf{v}^n & \mathbf{q}_b^n & \mathbf{b}_a^b & \mathbf{b}_g^b \end{bmatrix}^T$$
where $\mathbf{p}^n \in \mathbb{R}^3$ is position in East-North-Up (ENU), $\mathbf{v}^n \in \mathbb{R}^3$ is velocity in ENU, $\mathbf{q}_b^n \in \mathbb{H}$ is attitude quaternion, $\mathbf{b}_a \in \mathbb{R}^3$ is accelerometer bias, and $\mathbf{b}_g \in \mathbb{R}^3$ is gyroscope bias.

### 2.2 Discrete Propagation ($f_s = 10\text{ Hz}$ or $200\text{ Hz}$)
$$\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2} \left(\mathbf{R}(\mathbf{q}_k) (\mathbf{a}_{b,k} - \mathbf{b}_{a,k}) + \mathbf{g}^n\right) \Delta t^2$$
$$\mathbf{v}_{k+1} = \mathbf{v}_k + \left(\mathbf{R}(\mathbf{q}_k) (\mathbf{a}_{b,k} - \mathbf{b}_{a,k}) + \mathbf{g}^n\right) \Delta t$$
$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \text{Exp}\left( (\boldsymbol{\omega}_{b,k} - \mathbf{b}_{g,k}) \Delta t \right)$$

### 2.3 Longitudinal Causal AI Speed Update
Forward speed is predicted from 45 causal features extracted over a 1.0 s retrospective sliding window ($W = 10$ samples at $10\text{ Hz}$):
$$v_{\text{ai}} = \text{GBDT}(\mathbf{f}_{45})$$
$$z_v = v_{\text{ai}} - \mathbf{e}_x^T \mathbf{R}^T \mathbf{v}, \quad H_v = \begin{bmatrix} \mathbf{0}_{1 \times 3} & \mathbf{r}_0^T & (\mathbf{r}_0 \times \mathbf{v})^T & \mathbf{0}_{1 \times 6} \end{bmatrix}, \quad R_v = \sigma_v^2$$

### 2.4 Bounded-Slip Non-Holonomic Constraints (NHC)
To enforce vehicle non-holonomic motion without artificial rigidity during cornering:
$$R_{\text{lat}} = \text{clip}\left(0.70 R_{\text{lat}, k-1} + 0.30 R_0 \left(1 + 25 \omega_z^2 + 12 a_{\text{cent}}^2 + 5 a_y^2\right), R_0, R_{\max}\right)$$
where $R_0 = (0.05)^2\text{ m}^2/\text{s}^2$ and $R_{\max} = 0.16\text{ m}^2/\text{s}^2$.

### 2.5 In-Flight Straight-Line Kalman ZARU
When driving straight at speed ($|\omega_z| < 0.005\text{ rad/s}$, $|a_y| < 0.15\text{ m/s}^2$, $v > 4.0\text{ m/s}$) for $>8$ consecutive epochs:
$$\delta z_{\omega} = \omega_{z, b} - b_{g, z}, \quad H = \begin{bmatrix} \mathbf{0}_{1 \times 14} & 1 \end{bmatrix}, \quad R_{\text{zaru}} = (0.005)^2\text{ rad}^2/\text{s}^2$$

---

## 3. Baseline Performance Metrics (158 Authentic Outages)

- **Overall Median Drift:** 54.86%
- **Overall Mean Drift:** 123.14%
- **Pass Count ($\le 10\%$ Drift):** 9 / 158 (5.7%)
- **Median Cross-Track Error:** 41.53 m
- **Median Along-Track Error:** 37.10 m
- **Execution Throughput (C++17):** 83,823 Hz (11.93 µs per step)
- **Memory Footprint:** < 2.4 MB resident
