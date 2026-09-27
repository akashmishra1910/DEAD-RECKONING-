# PART 1 CONFIGURATION SPECIFICATION
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Document Version:** 1.0.0  
**Phase:** Part 1 — Intelligent Autonomous Dead-Reckoning Core  
**Authoritative Baseline:** B1/N8 Autonomous Pipeline  
**Target Environments:** `idr-core` (C++17), Android NDK (`jni_bridge.cpp`), Python Reference (`src/dead_reckoning/`)

---

## 1. System Dimensions & Sampling Parameters

| Parameter | Symbol | Nominal Value | Unit | Description |
|---|---|---|---|---|
| IMU Sampling Rate (Phone) | $f_{\text{imu,phone}}$ | 100 | Hz | High-rate tri-axial accelerometer and gyroscope sampling rate |
| IMU Sampling Rate (Edge/FOG) | $f_{\text{imu,edge}}$ | 200 | Hz | External industrial/FOG IMU stream rate |
| Nominal Time Step | $\Delta t_{\text{nom}}$ | 0.010 | s | Expected epoch duration ($1/f$) |
| Minimum Time Step | $\Delta t_{\text{min}}$ | 0.001 | s | Lower clamp for outlier rejection |
| Maximum Time Step | $\Delta t_{\text{max}}$ | 0.200 | s | Upper clamp for dropped packet recovery |
| GNSS Fix Rate | $f_{\text{gnss}}$ | 1.0 | Hz | GNSS fix and Doppler velocity packet arrival rate |
| Barometer Rate | $f_{\text{baro}}$ | 10.0 | Hz | Barometric altitude measurement rate |
| Feature Window Length | $N_{\text{win}}$ | 100 | samples | 1.0 second trailing window for GBDT inference |
| Ring Buffer Length | $N_{\text{ring}}$ | 5 | samples | Trailing median shock-blanking buffer |

---

## 2. In-Vehicle Alignment & Cradle Watchdog Parameters

| Parameter | Symbol | Nominal Value | Unit | Description |
|---|---|---|---|---|
| Stationary Detection Accel Variance | $\sigma_{a,\text{stat}}^2$ | 0.05 | $\text{m}^2/\text{s}^4$ | Threshold on $\text{Var}(\|\mathbf{a}\|)$ to confirm vehicle rest |
| Stationary Detection Gyro Variance | $\sigma_{\omega,\text{stat}}^2$ | 0.005 | $\text{rad}^2/\text{s}^2$ | Threshold on $\text{Var}(\|\boldsymbol{\omega}\|)$ to confirm vehicle rest |
| Gravity Magnitude Clamp | $g_{\text{nom}}$ | 9.80665 | $\text{m/s}^2$ | Reference Earth gravitational acceleration |
| Min Forward Accel for PCA | $a_{\text{fwd,min}}$ | 0.50 | $\text{m/s}^2$ | Acceleration burst threshold required for longitudinal PCA |
| Alignment Dwell Window | $T_{\text{align}}$ | 2.0 | s | Minimum duration of stationary data for initial leveling |
| Cradle Slip Gyro Variance | $\sigma_{\text{slip}}^2$ | 1.80 | $\text{rad}^2/\text{s}^2$ | Sliding window rate variance indicating physical phone slip |
| Recalibration Max Dwell | $T_{\text{recal}}$ | 4.0 | s | Maximum duration to re-level gravity after phone movement |

---

## 3. Causal Vibration & Pothole Filter Parameters

| Parameter | Symbol | Nominal Value | Unit | Description |
|---|---|---|---|---|
| Ring Buffer History | $L$ | 5 | samples | 50 ms trailing history |
| Outlier Z-Score Threshold | $k_{\text{outlier}}$ | 3.5 | - | Median Absolute Deviation multiplier for shock blanking |
| Engine Idle Low Band | $f_{\text{idle,low}}$ | 3.0 | Hz | Lower cutoff frequency for engine vibration envelope |
| Engine Idle High Band | $f_{\text{idle,high}}$ | 8.0 | Hz | Upper cutoff frequency for engine vibration envelope |
| Idle Energy Threshold | $E_{\text{idle,thresh}}$ | 0.12 | $\text{m/s}^2$ | Accel vibration energy confirming engine idle at standstill |

---

## 4. GBDT Speed Estimator Model Specification

| Parameter | Value | Description |
|---|---|---|
| Model Architecture | Gradient Boosted Decision Trees (GBDT) | Regressor trained on real IO-VNBD dataset |
| Stored Artifact Path | `models/gbdt_speed_model_iovnbd_top45.joblib` | Python model artifact |
| Transpiled Source | `idr-core/src/gbdt_speed_model.cpp` | Branchless static C++ implementation |
| Artifact SHA256 | `dc05b8de795732233082c3c687553d983d358bd4581514c5ffc4bdd9d58ad242` | Cryptographic verification hash |
| Number of Decision Trees | 120 | Total ensemble size |
| Maximum Tree Depth | 5 | Maximum leaf depth per tree |
| Learning Rate ($\eta$) | 0.08 | Shrinkage parameter |
| Number of Input Features | 45 | Top discriminative trailing features |
| Speed Output Clamp Min | 0.0 | $\text{m/s}$ (No negative forward speed allowed) |
| Speed Output Clamp Max | 35.0 | $\text{m/s}$ ($126\text{ km/h}$ vehicle speed limit) |
| Runtime Memory Allocation | 0 bytes | Strictly static stack memory execution |

---

## 5. 15-State ESKF Initial Covariances ($\mathbf{P}_0$)

The error-state covariance matrix $\mathbf{P}_0 \in \mathbb{R}^{15 \times 15}$ is initialized as a diagonal block matrix:

$$\mathbf{P}_0 = \text{diag}(\mathbf{P}_{p,0}, \mathbf{P}_{v,0}, \mathbf{P}_{\theta,0}, \mathbf{P}_{ba,0}, \mathbf{P}_{bg,0})$$

| State Block | Index Range | Initial $1\sigma$ Uncertainty | Variance Diagonal Value | Unit |
|---|---|---|---|---|
| Position $\delta\mathbf{p}$ | 0:3 | $1.0\text{ m}$ (horiz), $2.0\text{ m}$ (vert) | $\text{diag}(1.0, 1.0, 4.0)$ | $\text{m}^2$ |
| Velocity $\delta\mathbf{v}$ | 3:6 | $0.20\text{ m/s}$ (horiz), $0.40\text{ m/s}$ (vert) | $\text{diag}(0.04, 0.04, 0.16)$ | $\text{m}^2/\text{s}^2$ |
| Attitude $\delta\boldsymbol{\theta}$ | 6:9 | $1.0^\circ$ (roll/pitch), $3.0^\circ$ (yaw) | $\text{diag}(0.00030, 0.00030, 0.00274)$ | $\text{rad}^2$ |
| Accel Bias $\delta\mathbf{b}_a$ | 9:12 | $0.05\text{ m/s}^2$ | $\text{diag}(0.0025, 0.0025, 0.0025)$ | $\text{m}^2/\text{s}^4$ |
| Gyro Bias $\delta\mathbf{b}_g$ | 12:15 | $0.005\text{ rad/s}$ ($0.28^\circ/\text{s}$) | $\text{diag}(2.5\times 10^{-5}, 2.5\times 10^{-5}, 2.5\times 10^{-5})$ | $\text{rad}^2/\text{s}^2$ |

---

## 6. Continuous Process Noise Power Spectral Densities ($\mathbf{Q}$)

The discrete process noise matrix $\mathbf{Q}_k \approx \mathbf{F}_i \mathbf{Q}_c \mathbf{F}_i^T \Delta t$ is parameterized by continuous PSDs:

| Noise Source | Parameter | Continuous PSD Value | Unit | Description |
|---|---|---|---|---|
| Accelerometer White Noise | $q_{a}$ | $0.02^2 = 4.0 \times 10^{-4}$ | $\text{m}^2/\text{s}^3$ | Velocity random walk (VRW) |
| Gyroscope White Noise | $q_{g}$ | $0.002^2 = 4.0 \times 10^{-6}$ | $\text{rad}^2/\text{s}$ | Angular random walk (ARW) |
| Accelerometer Bias Random Walk | $q_{ba}$ | $1.0 \times 10^{-6}$ | $\text{m}^2/\text{s}^5$ | Slow thermal drift of accelerometer bias |
| Gyroscope Bias Random Walk | $q_{bg}$ | $1.0 \times 10^{-8}$ | $\text{rad}^2/\text{s}^3$ | Slow thermal drift of gyroscope bias |

---

## 7. Measurement Noise Variances ($\mathbf{R}$)

| Measurement Channel | State Observed | Nominal Noise Variance ($\mathbf{R}$) | Dynamic Scaling Rule |
|---|---|---|---|
| **GNSS Position (Nominal)** | $\mathbf{p}_{0:3}^n$ | $\mathbf{R}_{\text{gnss,pos}} = (2.5\text{ m})^2 \, \mathbf{I}_3$ | Fixed |
| **GNSS Position (Degraded)** | $\mathbf{p}_{0:3}^n$ | $\mathbf{R}_{\text{gnss,deg}} = (6.0\text{ m})^2 \, \mathbf{I}_3$ | Accepted if $\gamma_{\text{pos}} \le 11.35$ |
| **GNSS Velocity (Doppler)** | $\mathbf{v}_{0:3}^n$ | $\mathbf{R}_{\text{gnss,vel}} = (0.20\text{ m/s})^2 \, \mathbf{I}_3$ | Accepted if $\gamma_{\text{vel}} \le 11.35$ |
| **Barometric Altitude** | $p_z^n$ | $R_{\text{baro}} = (0.80\text{ m})^2 = 0.64\text{ m}^2$ | Constant vertical dampening |
| **AI Speed Forward** | $v_x^b$ | $R_{vx} = \sigma_{vx}^2$ | $\sigma_{vx} = 0.35 \cdot (1.0 + 0.5 \cdot E_{\text{vib}} + 1.2 \cdot |\omega_z|)$ |
| **NHC Lateral** | $v_y^b$ | $R_{vy} = \sigma_{vy}^2$ | $\sigma_{vy} = \begin{cases} 0.025\text{ m/s} & |\omega_z| < 0.05 \\ \min(0.40, 0.025 + 0.80|\omega_z|) & |\omega_z| \ge 0.05 \end{cases}$ |
| **NHC Vertical** | $v_z^b$ | $R_{vz} = (0.05\text{ m/s})^2 = 0.0025\text{ m}^2/\text{s}^2$ | Constant planar ground constraint |
| **ZUPT Standstill** | $\mathbf{v}^n$ | $\mathbf{R}_{\text{zupt}} = (0.01\text{ m/s})^2 \, \mathbf{I}_3$ | Active when stopped ($v < 0.2\text{ m/s}$) |
| **ZARU Gyroscope** | $b_{g,z}^b$ | $R_{\text{zaru}} = (0.005\text{ rad/s})^2 = 2.5 \times 10^{-5}$ | Active during steady straight cruise |

---

## 8. Supervisor & Integrity Thresholds

| Supervisor Parameter | Symbol | Value | Unit | Description |
|---|---|---|---|---|
| Chi-Square Threshold Position | $\chi^2_{\text{pos}}$ | 11.35 | - | $\alpha=0.01, m=3$ DOF innovation gate |
| Chi-Square Threshold Velocity | $\chi^2_{\text{vel}}$ | 11.35 | - | $\alpha=0.01, m=3$ DOF innovation gate |
| Pre-Outage Doppler Window | $T_{\text{doppler}}$ | 3.0 | s | Trailing buffer duration of valid GNSS Doppler velocities |
| Minimum Reacquisition Dwell | $N_{\text{dwell}}$ | 3 | epochs | Consecutive valid GNSS epochs before exiting DR mode |
| Outage Inactivity Timeout | $T_{\text{timeout}}$ | 1.5 | s | Maximum elapsed time without GNSS fix before triggering DR mode |
| Maximum Allowed Innovation Residual | $r_{\text{max,v}}$ | 8.0 | m/s | Safety threshold for speed measurement sanity |
