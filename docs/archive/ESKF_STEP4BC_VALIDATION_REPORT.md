# ESKF Step 4B-C Validation Report: Unified FLU + ENU Rotation Convention

**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System  
**Checkpoint:** ESKF Step 4B-C (Controlled Implementation Checkpoint)  
**Status:** **SUCCESS & VERIFIED**  
**Date:** 2026-09-22  

---

## 1. Executive Summary

In Checkpoints 4B-A and 4B-B, a frame convention discrepancy was discovered between the in-vehicle alignment engine and the ESKF:
- **Alignment Engine (`alignment_engine.py`)**: Defined body frame natively as **Forward, Left, Up (FLU)** with $\det(\mathbf{R}_{p2b}) = +1.0$ (right-handed $\text{SO}(3)$).
- **Previous ESKF (`eskf_fusion.py`)**: Used a **Forward, Right, Up (FRU)** basis with column 1 as Right ($[+\cos\psi, -\sin\psi, 0]$), yielding $\det(\mathbf{R}_b^n) = -1.0$ (improper reflection matrix).

In this checkpoint (**Step 4B-C**), the inertial navigation pipeline was unified and standardized on:
1. **Navigation Frame ($n$):** **ENU** ($+x=\text{East}$, $+y=\text{North}$, $+z=\text{Up}$, right-handed, $\det = +1.0$).
2. **Body Frame ($b$):** **FLU** ($+x=\text{Forward}$, $+y=\text{Left}$, $+z=\text{Up}$, right-handed, $\det = +1.0$).
3. **Direction Cosine Matrix ($\mathbf{R}_b^n$):** Belongs strictly to the Special Orthogonal Group $\text{SO}(3)$ ($\det(\mathbf{R}_b^n) = +1.0$, $\mathbf{R}\mathbf{R}^T = \mathbf{I}$) across all nominal mechanization, Rodrigues propagation, and Kalman injection phases.

---

## 2. Key Mathematical Formulations

### 2.1 Forward DCM: `euler_to_rotation_matrix(roll, pitch, yaw)`
Given azimuth heading $\psi$ (clockwise from North: $0=\text{North}$, $\pi/2=\text{East}$), pitch elevation $\theta$ (positive nose up), and bank roll $\phi$ (positive right side down / left side up):

The body basis vectors expressed in ENU navigation coordinates are:
- **Forward Axis ($\mathbf{r}_1$):**
  $$\mathbf{r}_1 = \begin{bmatrix} \sin\psi \cos\theta \\ \cos\psi \cos\theta \\ \sin\theta \end{bmatrix}$$
- **Base Lateral Axis before Roll ($\mathbf{r}_{2,\text{base}}$) [Left, 90° CCW from Forward in horizontal plane]:**
  $$\mathbf{r}_{2,\text{base}} = \begin{bmatrix} -\cos\psi \\ \sin\psi \\ 0 \end{bmatrix}$$
- **Base Vertical Axis before Roll ($\mathbf{r}_{3,\text{base}} = \mathbf{r}_1 \times \mathbf{r}_{2,\text{base}}$):**
  $$\mathbf{r}_{3,\text{base}} = \begin{bmatrix} -\sin\psi \sin\theta \\ -\cos\psi \sin\theta \\ \cos\theta \end{bmatrix}$$
- **Transverse Vectors after Bank Roll $\phi$ around Forward Axis $\mathbf{r}_1$:**
  $$\mathbf{r}_2 = \cos\phi \cdot \mathbf{r}_{2,\text{base}} + \sin\phi \cdot \mathbf{r}_{3,\text{base}}$$
  $$\mathbf{r}_3 = -\sin\phi \cdot \mathbf{r}_{2,\text{base}} + \cos\phi \cdot \mathbf{r}_{3,\text{base}} = \mathbf{r}_1 \times \mathbf{r}_2$$

$$\mathbf{R}_b^n = [\mathbf{r}_1 \quad \mathbf{r}_2 \quad \mathbf{r}_3] \implies \det(\mathbf{R}_b^n) = +1.0 \quad \forall (\phi, \theta, \psi) \in \mathbb{R}^3$$

### 2.2 Inverse Euler Extraction: `rotation_matrix_to_euler(R)`
Exact inversion from $\mathbf{R}_b^n \in \text{SO}(3)$ back to navigation Euler angles:
- **Pitch ($\theta$):** $\theta = \arcsin(\text{clip}(\mathbf{R}[2, 0], -1.0, 1.0))$
- **Yaw ($\psi$):** $\psi = \text{atan2}(\mathbf{R}[0, 0], \mathbf{R}[1, 0])$
- **Roll ($\phi$):** $\phi = \text{atan2}(\mathbf{R}[2, 1], \mathbf{R}[2, 2])$

Round-trip consistency error across arbitrary orientation space:
$$\max(|\phi - \phi_{\text{rec}}|, |\theta - \theta_{\text{rec}}|, |\psi - \psi_{\text{rec}}|) < 5.55 \times 10^{-17} \text{ rad}$$

### 2.3 SVD Re-orthogonalization: `renormalize_rotation_matrix(R)`
Because the matrix is in $\text{SO}(3)$, projection to the nearest orthogonal matrix in Frobenius norm is performed via Singular Value Decomposition:
$$\mathbf{U}, \boldsymbol{\Sigma}, \mathbf{V}^T = \text{SVD}(\mathbf{R}), \quad \mathbf{R}_{\text{ortho}} = \mathbf{U} \mathbf{V}^T$$
If $\det(\mathbf{R}_{\text{ortho}}) < 0$, the last column of $\mathbf{U}$ is flipped to ensure the solution resides strictly on the $\text{SO}(3)$ manifold.

### 2.4 Kinematic Consistency & Heading Observation
- In right-handed FLU ($+z_b=\text{Up}$), vehicle angular rate $\omega_{b, z} > 0$ represents counter-clockwise rotation (left turn).
- In ENU navigation frame, azimuth $\psi$ is clockwise from North. Therefore, a counter-clockwise left turn naturally decreases azimuth yaw: $\Delta \psi = -\omega_{b, z} \Delta t$.
- In the error-state vector $\delta\mathbf{x} \in \mathbb{R}^{15}$, error state $\delta\boldsymbol{\theta}^n = \delta\mathbf{x}[6:9]$ represents attitude errors in ENU tangent coordinates. Because $\delta\theta_z^n$ is counter-clockwise around $+z^n$ (Up), $\delta\psi = -\delta\theta_z^n = -\delta\mathbf{x}[8]$.
- Consequently, for heading/compass measurement updates:
  $$\mathbf{H}_{\text{heading}} = [0, \dots, 0, \underbrace{-1.0}_{\text{index 8}}, 0, \dots, 0]$$
  guaranteeing that positive measurement residuals $\delta z = \psi_{\text{meas}} - \psi_{\text{nominal}}$ drive nominal yaw monotonically towards the observed heading.

---

## 3. Test Suite Verification

### 3.1 Dedicated 3D Attitude ESKF Suite (`tests/test_eskf_fusion.py`)
All 14 isolated unit and integration tests passed:

| Test # | Description | Result | Details |
|---|---|---|---|
| **Test 1** | 500-step arbitrary gyro propagation | **PASS** | $\text{ortho\_err} = 5.30 \times 10^{-16}$, $\det(\mathbf{R}) = +1.0000$ |
| **Test 2** | Zero angular rate constancy | **PASS** | $\max\Delta \mathbf{R} = 2.84 \times 10^{-15}$, $\max\Delta \text{Euler} = 0.0$ |
| **Test 3** | Pure yaw rotation | **PASS** | $\Delta \psi = -\omega_z \Delta t \cdot N = -0.40$ rad, $\phi = 0, \theta = 0$ |
| **Test 4** | Pure roll rotation | **PASS** | $\phi = 0.2250$ rad, $\psi = 0, \theta = 0$ |
| **Test 5** | Pure pitch rotation | **PASS** | $\theta = 0.2500$ rad, $\phi = 0, \psi = 0$ |
| **Test 6** | Combined 3-axis continuous rotation | **PASS** | $\text{ortho\_err} = 1.24 \times 10^{-16}$, finite and normalized |
| **Test 7** | Full 3D attitude error injection | **PASS** | All 3 axes updated ($\Delta\phi=-0.035, \Delta\theta=0.040, \Delta\psi=-0.073$) |
| **Test 8** | AI speed measurement update ($1 \times 15$) | **PASS** | $v_{\text{updated}} = [12.31, 0.00, -0.01]$ m/s, $\mathbf{P}$ stable |
| **Test 9** | Non-Holonomic Constraints update ($2 \times 15$) | **PASS** | $v_{\text{lat}}^b = -0.136$ m/s, $v_{\text{vert}}^b = 0.050$ m/s (damped) |
| **Test 10** | Standstill ZUPT ($3 \times 15$) | **PASS** | Velocity collapses: $2.722 \to 0.0003$ m/s, $\mathbf{P}[3, 3] < 0.01$ |
| **Test 11** | Covariance stability & symmetry (100 cycles) | **PASS** | $\text{symm\_err} = 0.0$, $\min\lambda(\mathbf{P}) = 4.44 \times 10^{-6} > 0$ |
| **Test 12** | Synthetic multi-maneuver trajectory | **PASS** | Straight $\to$ turn $\to$ pitch $\to$ halt cleanly executed |
| **Test 13** | **Alignment $\to$ ESKF Turning Integration** | **PASS** | $\det(\mathbf{R}_{p2b}) = +1.0000$, $\det(\mathbf{R}_b^n) = +1.0000$, left turn from East ($90^\circ \to 68.4^\circ$) develops $+v_y^n = 3.71$ m/s (North) without left/right inversion |
| **Test 14** | **Heading Measurement Convergence** | **PASS** | Prior $0.500 \to 0.638$ toward target $0.650$ rad; $\det(\mathbf{R}) = +1.0000$ |

### 3.2 End-to-End System Suite (`test_suite.py`)
All 6 system integration tests passed:
- `test_01_alignment_engine_leveling_and_heading`: **PASS**
- `test_02_pothole_and_vibration_filtering`: **PASS**
- `test_02b_butterworth_prefilter_and_shock_blanking`: **PASS**
- `test_03_seamless_gnss_deficit_and_chi2_recovery`: **PASS** (sub-5m drift over 50m outage; $\chi^2$ multipath gate active)
- `test_04_3d_map_matching_elevation_disambiguation`: **PASS** (100% flyover deck disambiguation)
- `test_05_edge_engine_throughput_200hz`: **PASS** ($3130.2 \pm 137.4$ Hz throughput, $15.7\times$ real-time headroom)

---

## 4. Specific Audit Questions & Answers

### A. Was the alignment engine modified?
**NO.** `idr_pipeline/alignment_engine.py` was **NOT modified**. It natively outputs FLU ($\mathbf{x}_b = \text{Forward}$, $\mathbf{y}_b = \mathbf{z}_b \times \mathbf{x}_b = \text{Left}$, $\mathbf{z}_b = \text{Up}$) with $\det(\mathbf{R}_{p2b}) = +1.0000$.

### B. Were GBDT models or features modified?
**NO.** Neither `models/` nor `idr_pipeline/ai_velocity_model.py` was modified. The **Top45 GBDT model remains strictly frozen**.

### C. Did any downstream component break?
**NO.** All downstream components (`map_matcher_3d.py`, `gnss_ins_fusion_engine.py`, `vibration_filter.py`, `edge_engine.py`) operate either strictly in the navigation frame (ENU coordinates) or on the forward body axis $\mathbf{x}_b = \mathbf{R}[:, 0]$, which is invariant under the Left vs. Right lateral definition. 100% of the integration test suite passed.

### D. What are the numerical properties of the unified DCM?
- **Determinant:** $\det(\mathbf{R}_b^n) = +1.000000$ (numerical error $< 10^{-15}$).
- **Orthonormality:** $\|\mathbf{R} \mathbf{R}^T - \mathbf{I}\|_F < 5.30 \times 10^{-16}$.
- **Euler Round-Trip:** $\|\text{Euler}(\mathbf{R}(\text{Euler})) - \text{Euler}\|_\infty < 5.55 \times 10^{-17}$ rad.

### E. How does turning evolve?
- A vehicle traveling East ($\psi = 90^\circ = \pi/2$) executing a counter-clockwise turn ($\omega_{b, z} > 0$) turns toward North ($\psi \to 0^\circ$).
- Velocity in navigation frame develops positive North velocity ($v_y^n > 0$).
- Position displaces toward North ($p_y^n > 0$).
- Left/right turning kinematics are strictly preserve-handed and physically authentic.

