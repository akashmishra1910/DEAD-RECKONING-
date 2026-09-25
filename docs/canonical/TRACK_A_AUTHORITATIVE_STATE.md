# TRACK A: AUTHORITATIVE STATE — PRODUCTION DELIVERABLES
## C++ Core Engine, Edge CLI Runner, and Android Mobile Application

**Document ID:** `DOC-CANONICAL-TRACK-A-03`  
**Version:** 2.0.0  
**Date:** September 25, 2026  
**Status:** VALIDATED & OPERATIONAL  

---

## 1. Scope and Objective

Track A represents the deployable software deliverables required by ISRO Problem Statement 26168:
1. **Portable C++17 Core Engine (`idr-core/`):** Self-contained, sample-rate agnostic, running with zero external ML dependencies.
2. **Edge CLI Tool (`idr-cli/`):** Consumes external high-rate IMU streams (~200 Hz FOG/MEMS) via file or socket.
3. **Android Navigation Product (`android/`):** 12-Screen Jetpack Compose global geographic navigation application with real-time JNI integration.

---

## 2. Core Engine Architecture (`idr-core`)

### 2.1 State Vector Formulation (15 States)
$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^n \\ \mathbf{v}^n \\ \mathbf{q}_b^n \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$
- $\mathbf{p}^n = [p_x, p_y, p_z]^T$: Local navigation position (ENU).
- $\mathbf{v}^n = [v_x, v_y, v_z]^T$: Velocity in ENU frame.
- $\mathbf{q}_b^n$: Unit quaternion representing orientation from body to navigation frame.
- $\mathbf{b}_a = [b_{ax}, b_{ay}, b_{az}]^T$: Accelerometer bias drift.
- $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$: Gyroscope bias drift.

### 2.2 Measurement Update Modalities
1. **AI Speed Update:** Forward velocity pseudo-measurement derived from transpiled GBDT decision tables:
   $$z_{\text{ai}} = v_{\text{gbdt}} + \eta_{\text{ai}}, \quad R_{\text{ai}} = \sigma_{\text{ai}}^2$$
2. **Non-Holonomic Constraints (NHC):** Vehicle lateral and vertical velocity suppression:
   $$v_y^b \approx 0, \quad v_z^b \approx 0, \quad R_{\text{nhc}} = \operatorname{diag}(\sigma_{\text{lat}}^2, \sigma_{\text{vert}}^2)$$
3. **Zero-Velocity Update (ZUPT):** STANDSTILL detection via accelerometer and gyroscope variance thresholds:
   $$\|\boldsymbol{\omega}\| < \epsilon_\omega, \quad \|\mathbf{a} - \mathbf{g}\| < \epsilon_a \implies \mathbf{v}^n = \mathbf{0}$$
4. **GNSS Deficit Supervisor:** Innovation gating on GNSS position fixes:
   $$\gamma = \mathbf{r}^T \mathbf{S}^{-1} \mathbf{r} \le 11.35 \quad (\alpha=0.01, \text{dof}=3)$$

### 2.3 3D HMM Multi-Layer Map Matching
- **Cross-track Gaussian emission probability** computed against topological road segments.
- **Barometric elevation difference barrier:** Transition probability set to zero if $|\Delta h| > 4.0\text{ m}$, preventing false snaps to elevated flyovers or subterranean underpasses.
- **Viterbi trellis decoding** operating in a causal sliding window.

---

## 3. Edge CLI Engine (`idr-cli`)

- **Execution Model:** High-throughput streaming loop.
- **Throughput:** Processes up to 50,000 IMU samples per second (>250x real-time at 200 Hz).
- **External IMU Compatibility:** Accepts arbitrary CSV and binary packet streams with timestamp, 3-axis gyro, 3-axis accel, optional magnetometer, and optional GNSS.
- **Output:** Timestamped ENU trajectory, speed estimates, covariance diagonals, and map-matched coordinates.

---

## 4. Android Mobile Application (`android/`)

- **UI Framework:** Kotlin + Jetpack Compose (12-Screen Global Geographic Product).
- **Map Subsystem:** OSM + Valhalla vector rendering and routing canvas with standard, dark, terrain, and satellite tile support.
- **NavIC Constellation Monitor:** Direct integration with Android `GnssStatus.Callback` displaying live PRN, constellation identity (`CONSTELLATION_IRNSS`), azimuth/elevation radar polar plot, and C/N0 signal-to-noise ratio.
- **Thread Safety & Buffering:** `BoundedSensorQueue` circular FIFO buffer decoupling high-frequency hardware sensor interrupts from 10 Hz UI StateFlow updates.
- **Parity Guarantee:** JNI wrapper calls native `idr-core` compiled via Android NDK (C++17, `-O3`), ensuring identical math across mobile and edge.
