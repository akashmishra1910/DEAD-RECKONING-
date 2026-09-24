# Phase 11.8B — Production Geographic Map + Navigation UI Integration

**Project**: ISRO / SIH PS 26168 — AI/ML Based Intelligent Dead Reckoning System  
**Phase**: 11.8B (Production Geographic Map & Navigation UI Integration)  
**Date**: September 2026  
**Status**: COMPLETE & VERIFIED  
**Track-B Baselines**: Strictly FROZEN ($C_0 = 76.34\%$, $N_0 = 34.43\%$, $N_7 = 35.71\%$)  

---

## 1. Forensic Audit & Coordinate Transformation Proof

### 1.1 Forensic Root Cause Analysis
During physical hardware evaluation in Phase 11.8, the Android application displayed primarily a tactical ENU grid rather than recognizable geographic roads due to four distinct architectural factors:
1. **Minimal Default Fixture**: The default bundled dataset (`sample_prayagraj.idrmap`) contained only 8 road edges spanning a 1.5 km box, leaving over 95% of the screen as empty space filled only with grid lines (`drawTacticalGrid`).
2. **Geodetic Anchor Disconnect**: In field testing away from the synthetic bounding box center, the vehicle's ENU coordinates were tens or hundreds of kilometers away from the 8-road fixture ($E > 100,000\text{ m}$), placing the roads off-screen.
3. **Absence of Visual Basemap Polygons**: The canvas previously drew only single-line topological centerlines without background landuse, rivers, road casings, or street labels.
4. **Lack of Hierarchical Visual Styling**: Roads were drawn with uniform line widths without visual distinction between major arterial highways, secondary roads, bridges, and tunnels.

### 1.2 Mathematical Coordinate Proof on Known Road Segments
The complete coordinate transformation pipeline was mathematically verified from WGS-84 geodetic coordinates to local tangent ENU coordinates and screen pixels:

#### Known Segment 1: Mahatma Gandhi Marg (Edge 0 in `sample_prayagraj.idrmap`)
* **Geodetic Anchor**: $(\phi_0 = 25.437750^\circ\text{ N}, \lambda_0 = 81.849000^\circ\text{ E}, h_0 = 0.0\text{ m})$
* **Node A (WGS-84)**: $\text{Lat} = 25.435000^\circ\text{ N}, \text{Lon} = 81.845000^\circ\text{ E}$
* **Node B (WGS-84)**: $\text{Lat} = 25.435000^\circ\text{ N}, \text{Lon} = 81.850000^\circ\text{ E}$
* **Node A (Local ENU)**: $\text{East} = -402.37\text{ m}, \text{North} = -304.64\text{ m}, \text{Up} = -0.02\text{ m}$
* **Node B (Local ENU)**: $\text{East} = 100.59\text{ m}, \text{North} = -304.64\text{ m}, \text{Up} = -0.01\text{ m}$
* **Screen Position** ($1080 \times 1920\text{ px}$ canvas, center at $(540, 960)$, vehicle at origin, scale $= 0.50\text{ px/m}$):
  * $\text{Node A (Screen)}: X = 338.8\text{ px}, Y = 1112.3\text{ px}$
  * $\text{Node B (Screen)}: X = 590.3\text{ px}, Y = 1112.3\text{ px}$

#### Known Segment 2: Sardar Patel Marg (Edge 2 in `sample_prayagraj.idrmap`)
* **Node A (WGS-84)**: $\text{Lat} = 25.430000^\circ\text{ N}, \text{Lon} = 81.850000^\circ\text{ E}$
* **Node B (WGS-84)**: $\text{Lat} = 25.435000^\circ\text{ N}, \text{Lon} = 81.850000^\circ\text{ E}$
* **Node A (Local ENU)**: $\text{East} = 100.60\text{ m}, \text{North} = -858.54\text{ m}, \text{Up} = -0.06\text{ m}$
* **Node B (Local ENU)**: $\text{East} = 100.59\text{ m}, \text{North} = -304.64\text{ m}, \text{Up} = -0.01\text{ m}$
* **Screen Position** (scale $= 0.50\text{ px/m}$):
  * $\text{Node A (Screen)}: X = 590.3\text{ px}, Y = 1389.3\text{ px}$
  * $\text{Node B (Screen)}: X = 590.3\text{ px}, Y = 1112.3\text{ px}$

---

## 2. Two-Layer Map Architecture

The system enforces strict decoupling between the estimation engine and the visual display layer:

```
+-------------------------------------------------------------------------+
|                         APPLICATION ARCHITECTURE                        |
+-------------------------------------------------------------------------+
|                                                                         |
|  [ LAYER A: ESTIMATOR NAVIGATION GRAPH ]                                |
|  OSM XML -> .idrmap v1.0 -> IdrMapReader -> C++ 3D HMM Map Matcher      |
|  * Strict IMMUTABILITY: 15-State ESKF, GBDT, NHC, ZUPT, Heading Aiding |
|  * Zero visual dependencies; purely topological for state estimation    |
|                                                                         |
|  [ LAYER B: VISUAL GEOGRAPHIC BASEMAP ]                                 |
|  Vector Basemap JSON + .idrmap Geometry -> TacticalGridMapCanvas.kt     |
|  * 100% Offline: Rivers Ganga/Yamuna, Parks, Campuses, Landmarks        |
|  * Hierarchical Road Styling: Motorways, Primary, Secondary, Tunnels     |
|  * Multi-Trajectory Overlays: GNSS (Blue), IDR (Saffron), Map (Purple)  |
|  * Vehicle Marker: Directional Chevron + Forward Illumination Cone      |
|                                                                         |
+-------------------------------------------------------------------------+
```

---

## 3. Truthful Sensor Rate Architecture (Phase Accumulator)

### 3.1 Resolving the 9.5 ms Gate Anomaly
On physical Android devices, `SensorManager.SENSOR_DELAY_FASTEST` produces raw hardware callbacks at ~468.8 Hz ($T_{cb} \approx 2.133\text{ ms}$).
* A fixed time-gate of $\Delta t \ge 9.5\text{ ms}$ required $5 \times 2.133\text{ ms} = 10.667\text{ ms}$ before triggering, yielding $\frac{1000}{10.667} \approx 93.75\text{ Hz}$.
* Implemented a **Deterministic Phase Accumulator Resampler** in `SensorIngestionManager.kt`:
  $$\text{PhaseAccumulator} \mathrel{+}= (t_{\text{raw}} - t_{\text{last}})$$
  When $\text{PhaseAccumulator} \ge 10,000,000\text{ ns}$ ($10.0\text{ ms}$):
  * Emit sample for navigation fusion.
  * Subtract $10,000,000\text{ ns}$ from accumulator (retaining fractional phase).
* **Result**: Emits exactly $100.0\text{ Hz}$ on average with zero phase drift across hours of operation.

### 3.2 Truthful UI Terminology
Updated all UI and settings screens:
* **Raw Callback Rate**: `~468.8 Hz (Hardware Interrupt)`
* **Navigation Consumed**: `100.0 Hz (Target: 100.0 Hz)`
* **Resampling Mode**: `Deterministic Phase Accumulator Causal Resampling`

---

## 4. UI Redesign & Settings Overhaul

1. **Map-Dominant Navigation Screen**: Map occupies dominant view with floating Top HUD Banner (Mode, Outage Duration, GNSS state, Satellite counts) and compact Bottom Instrument HUD (Speed, Heading, Map Confidence, Trajectory toggles).
2. **Comprehensive 6-Section Settings Screen**:
   - `1. DEVICE`: Android Version, API, Native ABI, Hardware Model, Offline Status.
   - `2. SENSOR PIPELINE`: Raw callback rate, Consumed rate, Resampler mode, Buffer health, P95 jitter.
   - `3. NAVIGATION ENGINE`: 15-State ESKF, GBDT regressor, NHC constraints, Heading supervisor, Chi-Square innovation gate.
   - `4. MAP ENGINE`: 3D Causal HMM, Elevation barrier $|Δh| > 4.0\text{ m}$, .idrmap v1.0 format, active hypotheses.
   - `5. OFFLINE DATA`: Active dataset, WGS-84 geodetic anchor, road edge count, basemap features.
   - `6. SAFETY`: 4-Wheeler dynamics mode, Two-wheeler disabled, Track-B frozen baseline proofs.

---

## 5. Verification & Test Suite Reconciliation

### 5.1 Test Count Audit
* **Pytest Test Count**: **195 / 195 tests passing** (0 failures, 8 version warnings).
* **Reconciliation**: Full root execution `python -m pytest -q` executes all 16 test modules totaling 195 tests without omission.

### 5.2 Frozen Track-B Baseline Proof
* $C_0$ (Kinematic DR): **76.34%** median drift (12/158 pass)
* $N_0$ (Classical EKF + GBDT): **34.43%** median drift (28/158 pass)
* $N_7$ (Production Estimator): **35.71%** median drift (36/158 pass)

---

## 6. Android APK Build Artifact

| Property | Value |
|---|---|
| **APK File** | [`android/app/build/outputs/apk/debug/app-debug.apk`](file:///c:/Users/adira/OneDrive/Desktop/DR/android/app/build/outputs/apk/debug/app-debug.apk) |
| **Size** | `10,825,830 bytes` (10.32 MB) |
| **SHA-256** | `CCEBEB0378566041A08E1702009C46102F596DE86E76E561D5B9045573C449DE` |
| **Architectures** | `arm64-v8a`, `x86_64` |
| **Native Libraries** | `libidr_jni.so`, `libc++_shared.so` |
| **Bundled Offline Maps** | `prayagraj_corridor.idrmap`, `prayagraj_basemap.json`, `dense_metro.idrmap`, `dense_metro_basemap.json`, `regional_network.idrmap`, `regional_network_basemap.json`, `sample_prayagraj.idrmap` |
