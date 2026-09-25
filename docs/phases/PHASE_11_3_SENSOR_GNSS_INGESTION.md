# Phase 11.3 — Android Sensor & GNSS Ingestion Layer

## 1. Executive Summary

Phase 11.3 implements the **Android Sensor & GNSS Ingestion Layer** for the ISRO/SIH PS 26168 Intelligent Dead Reckoning system. Building on the hardened C API (Phase 11.1) and the JNI bridge (Phase 11.2), this phase provides a real-time, thread-safe, and causal ingestion pipeline that interfaces Android hardware sensors and platform GNSS with the native navigation engine (`NativeNavigationEngine`).

### Core Guarantees & Constraints Preserved:
- **Track B Estimator Frozen**: Zero modifications to ESKF formulation, GBDT speed regressor, adaptive NHC (Phase 10C), heading supervisor, or map-matching mathematics.
- **Strictly Causal**: All sensor synchronization operates forward in time ($t_{\text{current}} \ge t_{\text{sample}}$). No future GNSS interpolation, lookahead, or post-hoc trajectory smoothing.
- **Offline-First**: Closed-form WGS-84 $\to$ Local Tangent Plane ENU geodetic projection with zero cloud dependencies or network roundtrips.
- **ISRO/NavIC Compliant**: Real-time extraction of NavIC (IRNSS, `CONSTELLATION_IRNSS == 7`) satellite counts and signal status via Android `GnssStatus.Callback`.
- **Two-Wheeler Logic Excluded**: Strict car/four-wheeler kinematic assumptions maintained.

---

## 2. Sensor Ingestion Architecture

```
Android Hardware Sensors
├── Sensor.TYPE_ACCELEROMETER (or UNCALIBRATED) @ ~100 Hz
├── Sensor.TYPE_GYROSCOPE (or UNCALIBRATED)     @ ~100 Hz
├── Sensor.TYPE_MAGNETIC_FIELD                  @ ~25 Hz
└── Sensor.TYPE_PRESSURE (Barometer)            @ ~10 Hz
             │
             ▼ (SensorEventListener on dedicated HandlerThread)
     BoundedSensorQueue (Capacity: 500, Drop Monitoring)
             │
             ▼
Android Location Services
├── LocationListener (GPS / Fused / NavIC)      @ ~1 Hz
└── GnssStatus.Callback (IRNSS / GPS / GLONASS)
             │
             ▼
     GnssIngestionManager (WGS-84 -> Local ENU Projection)
             │
             ▼
NavigationPipelineSynchronizer (Causal Dispatcher Thread)
             │
             ▼
NativeNavigationEngine (JNI -> idr-core C++17)
             │
             ▼
NavigationTelemetry (Position, Velocity, Attitude, Mode, NavIC Counts)
```

---

## 3. Coordinate Frames & Unit Conventions

| Measurement | Android Sensor Source | Raw Unit | Ingestion Output / Engine Unit | Frame / Sign Convention |
|---|---|---|---|---|
| **Acceleration** | `TYPE_ACCELEROMETER` | $\text{m/s}^2$ | $\text{m/s}^2$ | Phone Body Frame ($p$): $+X$ right, $+Y$ up/forward along screen, $+Z$ outward |
| **Angular Velocity** | `TYPE_GYROSCOPE` | $\text{rad/s}$ | $\text{rad/s}$ | Phone Body Frame ($p$): Right-handed rotation about $+X, +Y, +Z$ |
| **Magnetic Field** | `TYPE_MAGNETIC_FIELD` | $\mu\text{T}$ | $\mu\text{T}$ | Phone Body Frame ($p$) |
| **Barometric Pressure**| `TYPE_PRESSURE` | $\text{hPa}$ / $\text{mbar}$ | $\text{m}$ (altitude) | Standard Atmosphere Hypsometric Formula: $h = 44330 \cdot (1 - (P/1013.25)^{0.1903})$ |
| **GNSS Position** | `Location.getLatitude()`, `.getLongitude()`, `.getAltitude()` | $\text{deg}$, $\text{deg}$, $\text{m}$ | Local ENU $\text{m}$ ($X_E, Y_N, Z_U$) | WGS-84 ellipsoidal tangent plane centered at initial valid fix $(\phi_0, \lambda_0, h_0)$ |
| **GNSS Velocity** | `Location.getSpeed()`, `.getBearing()` | $\text{m/s}$, $\text{deg}$ | ENU Velocity $\text{m/s}$ ($V_E, V_N, V_U$) | $V_E = v \sin(\theta)$, $V_N = v \cos(\theta)$, $V_U = 0$ (or vertical speed if available) |

---

## 4. Geodesy: WGS-84 to Local ENU Conversion

The geodetic conversion in `GeoCoordinates.kt` uses the standard WGS-84 reference ellipsoid:
- Semi-major axis $a = 6378137.0\text{ m}$
- Flattening $f = 1 / 298.257223563$
- First eccentricity squared $e^2 = 2f - f^2 \approx 6.69437999014 \times 10^{-3}$

### Transformation Steps:
1. **Curvature in Prime Vertical**:
   $$N(\phi) = \frac{a}{\sqrt{1 - e^2 \sin^2\phi}}$$
2. **Geodetic to Earth-Centered Earth-Fixed (ECEF)**:
   $$X = (N(\phi) + h) \cos\phi \cos\lambda$$
   $$Y = (N(\phi) + h) \cos\phi \sin\lambda$$
   $$Z = \left(N(\phi)(1 - e^2) + h\right) \sin\phi$$
3. **ECEF Difference to Local East-North-Up (ENU)**:
   $$\Delta \mathbf{r} = \mathbf{r}_{\text{ECEF}} - \mathbf{r}_{0,\text{ECEF}}$$
   $$\begin{bmatrix} X_E \\ Y_N \\ Z_U \end{bmatrix} = \begin{bmatrix} -\sin\lambda_0 & \cos\lambda_0 & 0 \\ -\sin\phi_0 \cos\lambda_0 & -\sin\phi_0 \sin\lambda_0 & \cos\phi_0 \\ \cos\phi_0 \cos\lambda_0 & \cos\phi_0 \sin\lambda_0 & \sin\phi_0 \end{bmatrix} \Delta \mathbf{r}$$
4. **Local Tangent Plane Direct Approximation** (for $|p| < 50\text{ km}$ fast path):
   $$R_M = \frac{a(1 - e^2)}{(1 - e^2 \sin^2\phi_0)^{3/2}}, \quad R_N = \frac{a}{\sqrt{1 - e^2 \sin^2\phi_0}}$$
   $$X_E = (\lambda - \lambda_0) \cdot R_N \cos\phi_0$$
   $$Y_N = (\phi - \phi_0) \cdot R_M$$
   $$Z_U = h - h_0$$

---

## 5. Monotonic Timestamp Management & Rate Monitoring

Android `SensorEvent.timestamp` reports nanoseconds in the `SystemClock.elapsedRealtimeNanos()` timebase.
`TimestampPolicy.kt` governs timestamp progression:
- **Baseline Conversion**: Converts nanosecond monotonic timestamps to continuous seconds ($t_{\text{sec}}$).
- **Time Step Validation**: Asserts $\Delta t \in [0.001\text{ s}, 0.5\text{ s}]$.
- **Anomaly Detection**:
  - `DUPLICATE`: $\Delta t == 0$
  - `BACKWARDS`: $\Delta t < 0$ (clock jitter/out-of-order delivery)
  - `GAP`: $\Delta t > 0.1\text{ s}$ (dropped sensor batches or OS scheduling stalls)
- **Exponential Moving Average (EMA) Frequency Tracking**: Tracks instantaneous sample rate ($\text{Hz}$) and jitter ($\text{ms}$).

---

## 6. GNSS Ingestion & NavIC / IRNSS Monitoring

The `GnssIngestionManager.kt` monitors platform GNSS fixes:
- **Anchor Locking**: The first valid 3D GNSS fix establishes the geodetic anchor $(\phi_0, \lambda_0, h_0)$. The anchor remains immutable during dead-reckoning outages.
- **NavIC Satellite Tracking**: Listens to `GnssStatus.Callback` and counts satellites with `constellationType == GnssStatus.CONSTELLATION_IRNSS` (constant value `7`). Reports `navicSatellitesInView` and `navicSatellitesUsedInFix`.
- **Validity Gate**: Fixes are marked valid if:
  1. Fix age $\le 1.5\text{ s}$ relative to current IMU timestamp.
  2. Horizontal accuracy $\le 25.0\text{ m}$.
  3. Non-zero coordinates and valid speed/bearing estimates.

---

## 7. Causal Pipeline Synchronization

`NavigationPipelineSynchronizer.kt` operates a single-threaded dispatch loop:
1. **High-Rate IMU Ingestion**: Accel and Gyro events trigger step integration.
2. **Asynchronous GNSS Latching**: Pulls `latestGnssSample`. If `gnssSample.timestamp <= imuTimestamp` and `imuTimestamp - gnssSample.timestamp <= 1.5s`, passes `gnssValid = true` with ENU coordinates. Once consumed, the fix is marked consumed to prevent duplicate GNSS measurement updates on subsequent IMU ticks.
3. **Auxiliary Latching**: Pulls latest calibrated Magnetometer vector and Barometric altitude.
4. **JNI Execution**: Calls `nativeProcessStepExt()` or `nativeProcessStepDirectBuffer()`.
5. **Telemetry Dispatch**: Dispatches `NavigationTelemetry` to registered UI/logging callbacks.

---

## 8. Threading & Concurrency Architecture

| Thread | Role | Synchronization Mechanism |
|---|---|---|
| **Sensor HandlerThread** | Delivers raw Android `SensorEvent` callbacks | `BoundedSensorQueue` (ReentrantLock + Condition) |
| **Location HandlerThread** | Delivers Android `Location` & `GnssStatus` updates | Atomic reference swapping (`AtomicReference<GnssSample>`) |
| **Navigation Dispatcher Thread** | Dequeues IMU, latches GNSS/Mag/Baro, executes JNI step | Blocking queue dequeue + direct native invocation |
| **Main / UI Thread** | Renders map and status banners | Immutable `NavigationTelemetry` callback post |

---

## 9. Ingestion Inventory Summary

| Component | Class | Source | Rate Target | Anomaly / Error Protection |
|---|---|---|---|---|
| Geodesy | `GeoCoordinates` | WGS-84 Coordinates | On-demand | Latitude clamp $[-90^\circ, 90^\circ]$, longitude wrapping, singular pole protection |
| Timestamps | `TimestampPolicy` | Monotonic Nanos | 100 Hz | Backwards clamp, duplicate rejection, gap detection |
| Queue | `BoundedSensorQueue` | Sensor Events | $\le 1000\text{ Hz}$ | Bounded capacity (500), non-blocking drop with counter |
| Sensors | `SensorIngestionManager` | Android SensorManager | 100 Hz / 25 Hz / 10 Hz | Missing sensor detection, uncalibrated sensor fallback |
| GNSS / NavIC | `GnssIngestionManager` | LocationManager / GnssStatus | 1 Hz | Stale fix rejection ($>1.5\text{ s}$), accuracy gating, NavIC constellation filter |
| Synchronizer | `NavigationPipelineSynchronizer` | All Ingestion Managers | 100 Hz | Strict causal order, thread-safe lifecycle state machine |
| Diagnostics | `IngestionDiagnostics` | Ingestion Managers | 1 Hz | Live JSON/Snapshot generation for rate, jitter, loss |

---

## 10. Verification & Test Coverage

The ingestion layer is verified across both Kotlin JVM unit tests and Python test suites:
- `GeoCoordinatesTest.kt`: WGS-84 to ENU roundtrip accuracy ($<1\text{ mm}$ error), zero displacement at anchor, quadrant sign correctness.
- `TimestampPolicyTest.kt`: Monotonic progression, duplicate/backwards/gap detection, EMA sample rate calculation.
- `BoundedSensorQueueTest.kt`: FIFO order preservation, bounded overflow dropping, thread-safe concurrent producer-consumer stress testing.
- `test_phase11_3_ingestion_layer.py`: Full synthetic 100 Hz IMU + 1 Hz GNSS causal synchronization pipeline, geodetic conversion tests, NavIC constellation tracking tests, anomaly recovery tests.

All **169 tests** across the project pass cleanly with zero warnings or regressions.
