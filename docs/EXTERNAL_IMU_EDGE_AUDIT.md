# External IMU & Edge Engine Data Path Audit
## SIH 2026 / ISRO PS 26168 — Deliverable 2 Architecture Verification

---

## 1. Executive Summary

This document establishes the architectural audit and validation of the **Edge Navigation Engine** under external IMU inputs, satisfying the requirements of **ISRO PS 26168 Deliverable 2**:
> *"An edge-deployable software engine — the same algorithms, decoupled from Android, consuming any external IMU stream, running at ~200 Hz with FOG-grade IMU data."*

The audit confirms that:
1. **Core Decoupling:** The navigation core (`idr-core` C++17 and `idr_pipeline` common core) contains **zero Android/JNI dependencies**, zero smartphone sensor HAL assumptions, and zero hardcoded cradle mount orientations.
2. **Normalized Sensor Contract:** A standardized, SI-unit physical contract (`SensorFrame`) decouples the mathematical core from upstream hardware vendors.
3. **Rigorous Sensor Flexibility:** The engine handles any combination of sensor channels: External IMU only, External IMU + GNSS, External IMU + Baro, or External IMU + GNSS + Baro.
4. **Physical Ingestion Layer:** Unit conversions ($g \to \text{m/s}^2$, $\text{deg/s} \to \text{rad/s}$), sensor-to-body rotation matrix $R_{s2b}$, scale factors, bias calibrations, timestamp monotonicity checks, gap detection, and physical sanity bounds ($|a| \le 200\text{ m/s}^2$, $|\omega| \le 50\text{ rad/s}$) are strictly enforced before fusion.
5. **AI/ML Generalization Transparency:** The centripetal kinematic aiding ($a_y \approx v \cdot \omega_z$) is a hardware-invariant rigid-body law that holds for any IMU mounting. GBDT vibration energy features, originally trained on smartphone cradle data, are documented with complete honesty regarding domain-shift on vehicle chassis mounts.

---

## 2. Code Path Audit: Sensor Input to Telemetry Output

### 2.1 C++ Production Engine (`idr-core/`)

```
   Raw External IMU / GNSS / Baro
                │
                ▼
   idr::EdgeNavigationEngine::pushImu(ImuSample)
   idr::EdgeNavigationEngine::pushGnss(GnssSample)
   idr::EdgeNavigationEngine::pushBarometer(BarometerSample)
                │
                ▼
   idr::EdgeNavigationEngine::update(std::optional<double> ai_speed)
                │
                ├─► [Monotonicity & Dropped Packet Detection]: t_k > t_{k-1}
                ├─► [Finite & Extreme Sanity Checks]: NaN/Inf check; |a| <= 200 m/s^2; |w| <= 50 rad/s
                ├─► [Unit Normalization]: g -> m/s^2; deg/s -> rad/s
                ├─► [Scale & Bias Calibration]: a_cal = S_a * a_si - b_a; w_cal = S_w * w_si - b_w
                ├─► [Sensor-to-Body Transformation]: a_b = R_s2b * a_cal; w_b = R_s2b * w_cal
                │
                ▼
   idr::SeamlessGNSSDeficitEngine::process_step(...)
                │
                ├─► [15-State Error State Kalman Filter Mechanization] (dt = 1 / imu_hz)
                ├─► [AI Speed & Vibration Feature Filter Update]
                ├─► [Kinematic Non-Holonomic Constraints (NHC) via body FLU frame]
                ├─► [Zero Angular Rate Update (ZARU) & Standstill ZUPT]
                ├─► [Optional 3D Map-Matching Constraint Aiding]
                │
                ▼
   idr::NavigationTelemetry / idr::EdgeDiagnostics
```

### 2.2 Python Common Core (`idr_pipeline/dual_profile.py`)

The Python reference implementation strictly mirrors the C++ production data flow:
1. `EdgeSensorAdapter`: Transforms raw sensor samples, applies $R_{s2b}$, scale/bias, monotonicity check, gap detection ($> 3\Delta t$), and sanity checks.
2. `ExternalImuReplayAdapter`: High-level ingestion for CSV datasets and streamed arrays.
3. `EdgeNavigationEngine`: Public API conforming to the 9 standard lifecycle methods.
4. `CommonNavigationCore`: Shared 15-state ESKF with causal 3D AI velocity-vector fusion.

### 2.3 Verification Against Smartphone / Android Assumptions

| Potential Leakage | Smartphone Mobile Profile | Edge External IMU Profile | Audit Finding |
|---|---|---|---|
| **Operating System** | Android NDK / JNI bridge | Native C++17 / POSIX / Win32 | **PASSED:** Zero Android headers (`android/*`, `jni.h`) in `idr-core/include/idr/`. |
| **Coordinate Frame** | Phone screen coordinates (X right, Y up, Z out) | Standard body FLU (Forward-Left-Up) | **PASSED:** Configurable mounting rotation matrix $R_{s2b}$ applied before ESKF mechanization. |
| **Mount Orientation** | Arbitrary cradle with phone roll/pitch leveling | Rigid chassis / vehicle mounting matrix | **PASSED:** Fixed mounting matrix $R_{s2b}$ explicitly configured in `EdgeCalibrationConfig`. |
| **Update Rate** | 10 Hz navigation, 100 Hz IMU | 200 Hz navigation & IMU mechanization | **PASSED:** Dynamic $\Delta t = 1 / f_{\text{IMU}}$ parameterized per profile; zero hardcoded 0.1s assumptions. |
| **Sensor Presence** | Barometer + Mag assumed available | External IMU only / optional GNSS / optional Baro | **PASSED:** Core handles missing barometer, GNSS, or magnetometer gracefully via `std::optional`. |
| **Noise Model** | Smartphone MEMS ($\sigma_a = 0.15\text{ m/s}^2, \sigma_\omega = 0.008\text{ rad/s}$) | FOG / Industrial ($\sigma_a = 0.02\text{ m/s}^2, \sigma_\omega = 0.0005\text{ rad/s}$) | **PASSED:** Explicitly separated profiles in `NavigationProfileConfig`. |

---

## 3. Normalized External IMU Contract

### 3.1 Coordinate Conventions
- **Standard Frame:** Vehicle Body Frame — **FLU (Forward-Left-Up)**:
  - **$+X$ (Forward):** Longitudinal axis pointing toward vehicle travel direction.
  - **$+Y$ (Left):** Lateral axis pointing to the driver's left.
  - **$+Z$ (Up):** Vertical axis pointing upward, perpendicular to road surface.
- **Sensor Misalignment Matrix ($R_{s2b}$):** $3 \times 3$ direction cosine matrix rotating the sensor's physical frame into the vehicle FLU body frame:
  $$\mathbf{a}_{\text{body}} = R_{s2b} (\mathbf{S}_a \mathbf{a}_{\text{SI}} - \mathbf{b}_a)$$
  $$\boldsymbol{\omega}_{\text{body}} = R_{s2b} (\mathbf{S}_g \boldsymbol{\omega}_{\text{SI}} - \mathbf{b}_g)$$

### 3.2 Standard Physical Units
All values consumed by the mathematical core must be normalized SI units:
- **Linear Acceleration:** $\text{m/s}^2$ (standard gravity $g_0 = 9.80665\text{ m/s}^2$ applied if input in $g$).
- **Angular Velocity:** $\text{rad/s}$ (converted via $\pi/180$ if input in $\text{deg/s}$).
- **Timestamp:** Epoch seconds (float64) with monotonic progression ($t_k > t_{k-1}$).
- **Position:** Local Cartesian East-North-Up (ENU) in meters.
- **Velocity:** Local Cartesian ENU velocity vector $[v_E, v_N, v_U]$ in m/s.
- **Barometric Elevation:** Altitude in meters (WGS-84 / MSL).

### 3.3 Sensor Presence Flexibility
The Edge Engine operates deterministically across all 4 sensor availability permutations:
1. **External IMU Only:** Mechanizes 15-state ESKF with AI speed / NHC ($v_y^b \approx 0, v_z^b \approx 0$), ZARU, and ZUPT. Covariance trace grows predictably to reflect dead reckoning uncertainty.
2. **External IMU + GNSS:** Full high-rate inertial navigation with GNSS position/velocity Kalman corrections and heading updates.
3. **External IMU + Barometer:** Constrains vertical channel elevation drift ($v_z, p_z$) during extended GNSS outages.
4. **External IMU + GNSS + Barometer:** Complete optimal multi-sensor fusion state.

---

## 4. Ingestion Pre-Processing & Validation Layer

The pre-processing pipeline validates every incoming sample against strict physical bounds:
1. **Timestamp Monotonicity:** Reject any sample with $t_k \le t_{k-1}$. The dropped-samples counter is incremented, and state is preserved.
2. **Gap Detection:** If $\Delta t = t_k - t_{k-1} > 3 \times \Delta t_{\text{nominal}}$, a sensor packet gap event is flagged in diagnostics.
3. **Sanity Clamping / Rejection:**
   - Samples containing `NaN` or `Inf` are rejected immediately.
   - Accelerations exceeding $200\text{ m/s}^2$ (~$20g$) or angular rates exceeding $50\text{ rad/s}$ (~$2860^\circ/\text{s}$) are rejected as sensor hardware faults.
4. **Scale & Bias Calibration:** Applies user-provided per-axis scale factors $\mathbf{S}$ and static bias offsets $\mathbf{b}$ prior to frame transformation.

---

## 5. AI/ML Speed Model Compatibility Analysis

### 5.1 Centripetal Kinematics ($a_y \approx v \cdot \omega_z$)
- **Kinematic Invariant:** For any rigid vehicle executing planar motion, lateral acceleration in body frame obeys:
  $$a_y = v_{\text{fwd}} \cdot \omega_z + \dot{v}_y + g \sin(\phi)$$
  At highway and urban speeds with lever arm $l_x$, the relationship $v_{\text{fwd}} \approx \frac{a_y - l_x \dot{\omega}_z}{\omega_z}$ is a fundamental kinematic constraint.
- **Hardware Invariance:** This relationship holds identically for FOG gyroscopes, industrial MEMS, or smartphone sensors when mounted rigidly to the vehicle frame.

### 5.2 Vibration Energy Features & Domain Shift
- **Smartphone Cradle vs Chassis Mount:** Smartphone IMUs mounted in a windshield cradle capture engine vibrations filtered through the cradle elasticity ($\sim 15 - 35\text{ Hz}$). Rigid chassis-mounted FOG/industrial IMUs measure higher-frequency road and structural vibrations ($\sim 50 - 200\text{ Hz}$) with lower baseline noise.
- **Honest ISRO Notice:**
  > *"The navigation pipeline architecture is 100% compatible with external IMU data. However, GBDT decision-tree weights trained exclusively on smartphone cradle datasets (such as IO-VNBD) are subject to vibrational domain-shift. For production deployment with tactical FOG hardware on vehicle chassis, fine-tuning the vibration regressor on chassis-mounted calibration runs is recommended. The kinematic velocity-vector and NHC constraints require zero retraining and function unconditionally."*
