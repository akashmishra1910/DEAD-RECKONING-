# SENSOR MATH & COORDINATE FRAME AUDIT REPORT
## SIH 2026 PS 26168 — AI-ML Based Intelligent Dead Reckoning System

**Audit Date:** September 27, 2026  
**Module:** Mathematical Formulations, Sensor Units, Coordinate Frames & Kinematic Mechanization  
**Status:** COMPLETE & VERIFIED

---

## 1. Physical Units Verification

| Sensor / Quantity | Android HAL Unit | Internal Filter Unit | Conversion Factor | Validation Status |
|---|---|---|:---:|:---:|
| **Accelerometer** | $\text{m/s}^2$ (`Sensor.TYPE_ACCELEROMETER`) | $\text{m/s}^2$ (SI) | $1.0$ (No conversion needed) | **CORRECT** |
| **Gyroscope** | $\text{rad/s}$ (`Sensor.TYPE_GYROSCOPE`) | $\text{rad/s}$ (SI) | $1.0$ (No conversion needed) | **CORRECT** |
| **Magnetometer** | $\mu\text{T}$ (`Sensor.TYPE_MAGNETIC_FIELD`) | $\mu\text{T}$ | $1.0$ | **CORRECT** |
| **Barometer** | $\text{hPa}$ (`Sensor.TYPE_PRESSURE`) | Meters ($\text{m}$) | $44330.0 \cdot (1.0 - (P / 1013.25)^{0.190295})$ | **CORRECT** |
| **GNSS Ground Speed** | $\text{m/s}$ (`location.speed`) | $\text{m/s}$ & $\text{km/h}$ | $\text{km/h} = \text{speed}_{\text{m/s}} \times 3.6$ | **CORRECT** |
| **GNSS Course / Bearing** | Degrees CW from North ($0^\circ \dots 360^\circ$) | Radians & Degrees | $\text{rad} = \text{deg} \times \frac{\pi}{180.0}$ | **CORRECT** |
| **Timestamps** | Nanoseconds (`elapsedRealtimeNanos`) | Epoch Seconds ($\text{s}$) | $t_{\text{sec}} = (t_{\text{ns}} - t_{0,\text{ns}}) \times 10^{-9}$ | **CORRECT** |

---

## 2. Coordinate Frame Conventions

```mermaid
flowchart LR
    Phone["Phone Frame (p)<br/>+X: Right, +Y: Up, +Z: Screen"] -->|Alignment Matrix R_p^b| Body["Vehicle Body Frame (b)<br/>+X: Forward, +Y: Left, +Z: Up"]
    Body -->|Attitude DCM R(t)| Nav["Navigation Frame (n - ENU)<br/>+X: East, +Y: North, +Z: Up"]
    Nav -->|Geodetic Anchor (Lat0, Lon0, Alt0)| Earth["WGS-84 Geodetic<br/>Latitude, Longitude, Altitude"]
```

### 2.1 Frame Definitions
1. **Phone Frame ($p$)**: Standard Android HAL convention (X right, Y up/top of screen, Z out of display toward user).
2. **Vehicle Body Frame ($b$)**: Standard automotive ISO/SAE convention (X forward along longitudinal chassis axis, Y left along lateral axle, Z up perpendicular to deck).
3. **Navigation Frame ($n$)**: Local Tangent East-North-Up (ENU) Cartesian frame centered at the first valid GNSS fix anchor $(\phi_0, \lambda_0, h_0)$.
4. **WGS-84 Frame**: Global geodetic reference $(\phi, \lambda, h)$.

### 2.2 Alignment & Mounting Transformation ($\mathbf{R}_p^b$)
- Gravity vector $\mathbf{g}^p$ is estimated via stationary low-pass leveling ($\alpha = 0.02$).
- The upward deck unit normal is: $\mathbf{z}_b^p = \frac{\mathbf{g}^p}{\|\mathbf{g}^p\|}$.
- Forward longitudinal transient PCA defines $\mathbf{x}_b^p$.
- Lateral cross-product gives $\mathbf{y}_b^p = \mathbf{z}_b^p \times \mathbf{x}_b^p$.
- Mounting rotation matrix is formed as: $\mathbf{R}_b^p = [\mathbf{x}_b^p, \, \mathbf{y}_b^p, \, \mathbf{z}_b^p]$, and $\mathbf{R}_p^b = (\mathbf{R}_b^p)^T$.
- Phone-to-body conversion: $\mathbf{a}^b = \mathbf{R}_p^b \mathbf{a}^p$, $\boldsymbol{\omega}^b = \mathbf{R}_p^b \boldsymbol{\omega}^p$.

---

## 3. Kinematic Equations & Gravity Handling

### 3.1 Specific Force & Gravity Subtraction
The accelerometer measures specific force (upward contact force minus true gravity):
$$\mathbf{f}^b = \mathbf{a}_{\text{meas}}^b - \hat{\mathbf{b}}_a$$
$$\mathbf{f}^n = \mathbf{R} \mathbf{f}^b$$
True linear acceleration in the ENU navigation frame is:
$$\mathbf{a}^n = \mathbf{f}^n + \mathbf{g}^n = \mathbf{R} (\mathbf{a}_{\text{meas}}^b - \hat{\mathbf{b}}_a) + \begin{bmatrix} 0 \\ 0 \\ -9.80665 \end{bmatrix}$$
When stationary in level flight ($\mathbf{R} = \mathbf{I}$), $\mathbf{a}_{\text{meas}}^b \approx [0, 0, +9.80665]^T$, yielding $\mathbf{a}^n = \mathbf{0}$. Gravity projection is exact and non-singular.

### 3.2 Attitude Kinematics
Attitude matrix $\mathbf{R} \in \mathrm{SO}(3)$ propagates via Rodrigues rotation:
$$\Delta\boldsymbol{\theta} = (\boldsymbol{\omega}_{\text{meas}}^b - \hat{\mathbf{b}}_g) \Delta t$$
$$\mathbf{R}(t + \Delta t) = \mathbf{R}(t) \cdot \operatorname{Exp}([\Delta\boldsymbol{\theta}]_\times)$$
followed by polar decomposition SO(3) renormalization to prevent non-orthogonal drift.

---

## 4. 15-State ESKF Jacobian Audit

The continuous error-state dynamics matrix $\mathbf{F} \in \mathbb{R}^{15 \times 15}$ is:
$$\mathbf{F} = \begin{bmatrix}
\mathbf{0}_{3\times 3} & \mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & -[\mathbf{f}^n]_\times & -\mathbf{R} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & -\mathbf{R} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\
\mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3}
\end{bmatrix}$$

- Discrete state transition matrix: $\boldsymbol{\Phi} = \mathbf{I} + \mathbf{F} \Delta t$.
- Joseph-form covariance update: $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K} \mathbf{R}_{\text{cov}} \mathbf{K}^T$.
- Error-state attitude reset Jacobian: $\mathbf{G}_{\text{reset}} = \mathbf{I} - \frac{1}{2} [\delta\boldsymbol{\theta}^n]_\times$ applied to covariance block $\mathbf{P}_{\theta\theta}$.

---

## 5. Summary Findings
1. All physical units are strictly standard SI ($\text{m/s}^2$, $\text{rad/s}$, $\text{m}$, $\text{s}$).
2. Frame transformations $\text{Phone} \to \text{Body} \to \text{ENU} \to \text{WGS84}$ are mathematically verified, closed-form, and non-singular.
3. The previous issue where AI speed was not being computed when caller passed `-1.0` has been resolved by embedding `SlidingWindowFeatureExtractor` and `GBDTSpeedModel` directly inside `SeamlessGNSSDeficitEngine`.
