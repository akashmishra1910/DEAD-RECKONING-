# ISRO PS 26168: Deliverable 2 External IMU Edge Engine Compliance
## Problem Statement 26168 — Deliverable 2 Formal Audit

---

## 1. Compliance Matrix

| PS 26168 Requirement | Status | Implementation Evidence | Verification Method |
|---|---|---|---|
| **Edge-Deployable Engine**<br>Decoupled from Android, deployable on automotive edge compute | **COMPLIANT** | `idr-core/` portable C++17 library (CMake, POSIX, zero Android headers) | Verified via `ctest` (14/14 tests) and standalone edge binaries. |
| **Consume ANY External IMU**<br>Vendor-agnostic sensor ingestion interface | **COMPLIANT** | `SensorFrame`, `EdgeSensorAdapter`, `ExternalImuReplayAdapter` in `idr_pipeline/dual_profile.py` | Validated in `tests/test_external_imu_edge.py` across diverse schemas. |
| **High-Rate Operation (~200 Hz)**<br>Full Kalman mechanization at 200 Hz | **COMPLIANT** | Native C++ core runs at **49,085 Hz** (245x); Python adapter at **2,834 Hz** (14x) | Verified in `benchmark_tactical_100k` (100k continuous cycles). |
| **FOG-Grade Sensor Profile**<br>Noise modeling matching optical tactical sensors | **COMPLIANT** | `NavigationProfileConfig.edge_fog_defaults()` with $\sigma_\omega = 0.0005\text{ rad/s}$ | Validated in ESKF propagation and covariance evolution tests. |
| **Physical Units Normalization**<br>Standard SI ($\text{m/s}^2$, $\text{rad/s}$, ENU, meters) | **COMPLIANT** | Unit conversion layer handles $g \to \text{m/s}^2$ and $\text{deg/s} \to \text{rad/s}$ | Validated in `EdgeSensorAdapter.adapt_raw_imu()`. |
| **Sensor Misalignment ($R_{s2b}$)**<br>Explicit sensor-to-body frame rotation | **COMPLIANT** | Direction Cosine Matrix $R_{s2b}$ configured in `EdgeCalibrationConfig` | Tested in `test_10_multi_rate_operation_and_csv_replay`. |
| **Scale & Bias Calibration**<br>Explicit per-axis scale factor and bias offset | **COMPLIANT** | `accel_scale_factor`, `gyro_scale_factor`, `accel_bias_b`, `gyro_bias_b` | Applied prior to frame transformation in `EdgeSensorAdapter`. |
| **Out-of-Order / Corrupt Fault Rejection**<br>Monotonicity, gaps, and extreme value bounds | **COMPLIANT** | Rejects $t_k \le t_{k-1}$, detects gaps $> 3\Delta t$, clamps $|a| > 200\text{ m/s}^2, |\omega| > 50\text{ rad/s}$ | Tested in `test_8` and `test_9` of `test_external_imu_edge.py`. |
| **Multi-Sensor Flexibility**<br>Operates with IMU-only, +GNSS, +Baro, +GNSS+Baro | **COMPLIANT** | Core gracefully accepts `std::nullopt` for GNSS and Barometer | Tested in `test_1`, `test_2`, `test_3`, `test_4`. |
| **GNSS-Denied Dead Reckoning**<br>Seamless transition into and out of outages | **COMPLIANT** | Mode switches between `GNSS_AIDED_INS` and `INTELLIGENT_DEAD_RECKONING` | Tested in `test_5` and `test_6`. |
| **Physical FOG Hardware Validation**<br>Live test on physical FOG hardware | **PENDING** | Marked explicitly as `SOFTWARE_ONLY_VALIDATION_PENDING_FOG_HARDWARE` | Transparently stated; no physical FOG hardware in repo. |

---

## 2. Dual Profile Architecture: Mobile vs Edge

```
                                  COMMON NAVIGATION CORE
                     (15-State ESKF + 3D AI Velocity Fusion + NHC + ZUPT)
                                            ▲
                                            │
                     ┌──────────────────────┴──────────────────────┐
                     │                                             │
               MOBILE PROFILE                               EDGE_FOG PROFILE
   (Smartphone MEMS + Android HAL)                (External IMU + Native Edge Core)
   - Sensor: InvenSense / STMicro MEMS            - Sensor: FOG / Quartz Tactical IMU
   - Sample Rate: 100 Hz IMU, 10 Hz Nav           - Sample Rate: 200 Hz IMU & Nav
   - Accel Noise: 0.15 m/s^2                      - Accel Noise: 0.02 m/s^2
   - Gyro Noise: 0.008 rad/s                      - Gyro Noise: 0.0005 rad/s
   - Mount: Windshield cradle (lever arm 0.5m)    - Mount: Rigid chassis (lever arm 0.0m)
   - Calibration: Auto-gravity leveling           - Calibration: Explicit R_s2b, scale, bias
   - Deployment: Kotlin + Android NDK JNI         - Deployment: Native C++17 CLI / shared lib
```

---

## 3. Public Edge Navigation Engine API Contract

The external-IMU edge deployment communicates strictly via this clean 9-method interface:

```cpp
namespace idr {

class EdgeNavigationEngine {
public:
    // Lifecycle Management
    bool initialize(const EngineConfig& config);
    void reset();
    void shutdown();
    bool isInitialized() const;

    // Sensor Ingestion Streaming (Vendor-Agnostic)
    bool pushImu(const ImuSample& imu);
    bool pushGnss(const GnssSample& gnss);
    bool pushBarometer(const BarometerSample& baro);

    // Filter Step Execution
    bool update(std::optional<double> ai_speed = std::nullopt);

    // State & Diagnostics Inspection
    NavigationTelemetry getNavigationState() const;
    EdgeDiagnostics getDiagnostics() const;
};

} // namespace idr
```

---

## 4. Verification Evidence & Artifact Locations

- **C++ Native Core Header:** [`idr-core/include/idr/edge_engine.hpp`](file:///Users/akashmishra/SIH%20MOdel/idr-core/include/idr/edge_engine.hpp)
- **C++ Native Core Source:** [`idr-core/src/edge_engine.cpp`](file:///Users/akashmishra/SIH%20MOdel/idr-core/src/edge_engine.cpp)
- **Python Dual Profile Implementation:** [`idr_pipeline/dual_profile.py`](file:///Users/akashmishra/SIH%20MOdel/idr_pipeline/dual_profile.py)
- **External IMU Automated Test Suite:** [`tests/test_external_imu_edge.py`](file:///Users/akashmishra/SIH%20MOdel/tests/test_external_imu_edge.py)
- **Edge Dashboard Controller & UI:** [`edge_dashboard/controller.py`](file:///Users/akashmishra/SIH%20MOdel/edge_dashboard/controller.py), [`edge_dashboard/static/index.html`](file:///Users/akashmishra/SIH%20MOdel/edge_dashboard/static/index.html)
- **C++ 100k-Cycle Benchmark Executable:** `idr-core/build/tests/benchmark_tactical_100k`
- **Data Path Audit Document:** [`docs/EXTERNAL_IMU_EDGE_AUDIT.md`](file:///Users/akashmishra/SIH%20MOdel/docs/EXTERNAL_IMU_EDGE_AUDIT.md)
- **Validation Report:** [`docs/EXTERNAL_IMU_EDGE_VALIDATION.md`](file:///Users/akashmishra/SIH%20MOdel/docs/EXTERNAL_IMU_EDGE_VALIDATION.md)
