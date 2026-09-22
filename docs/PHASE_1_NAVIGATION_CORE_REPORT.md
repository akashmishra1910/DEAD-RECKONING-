# Phase 1: Navigation Core Validation Report
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Date:** 2026-09-23  
**Phase Status:** **PASSED**  
**Phase Gate Acceptance:** **100% (All 20 Invariants & 33/33 Unit & Regression Tests PASSED)**  
**Commit Reference:** `phase-01-navigation-core`  

---

## 1. Executive Summary

Phase 1 focused strictly on mathematical, physical, and numerical validation of the navigation filter core in `idr_pipeline/eskf_fusion.py`. All coordinate frames, rotation representations, kinematic propagation steps, measurement updates, and numerical Jacobians were audited and tested against analytical closed-form solutions, finite-difference perturbations, and real driving traces from the IO-VNBD dataset.

---

## 2. Invariants & Mathematical Formulations Validated

| Invariant / Property | Mathematical Formulation / Law | Verification Method | Status |
|---|---|---|:---:|
| **1. SO(3) Group Orthonormality** | $\mathbf{R} \mathbf{R}^T = \mathbf{I}_{3\times 3}, \ \|\mathbf{R} \mathbf{R}^T - \mathbf{I}\| < 10^{-13}$ | 1000 arbitrary randomized 3D rotations | **PASS** |
| **2. Proper Rotation Determinant** | $\det(\mathbf{R}_b^n) = +1.000000000000$ | Evaluated across 1000 SO(3) rotations | **PASS** |
| **3. ENU Navigation Frame** | $+X=\text{East}, +Y=\text{North}, +Z=\text{Up}$, $\mathbf{g}^n = [0, 0, -9.80665]^T$ | Cardinal direction tests (North, East, South, West) | **PASS** |
| **4. FLU Vehicle Body Frame** | $+X=\text{Forward}, +Y=\text{Left}, +Z=\text{Up}$ | Right-handed cross-product: $\mathbf{r}_1 \times \mathbf{r}_2 = \mathbf{r}_3$ | **PASS** |
| **5. Turning Direction Kinematics** | East ($\psi=90^\circ$) + left-turn gyro ($\omega_z > 0$) $\implies$ yaw decreases to North ($\psi \to 0^\circ$) | Dynamic single-step & multi-step kinematic integration | **PASS** |
| **6. Euler $\leftrightarrow$ DCM Roundtrip** | $\mathbf{R}(\text{roll}, \text{pitch}, \text{yaw}) \leftrightarrow (\text{roll}, \text{pitch}, \text{yaw})$, error $<10^{-12}$ | 500 arbitrary attitude configs (away from gimbal lock) | **PASS** |
| **7. Rodrigues DCM Formula** | $\mathbf{R} = \mathbf{I} + \frac{\sin\theta}{\theta}\lfloor\boldsymbol{\phi}\rfloor_\times + \frac{1-\cos\theta}{\theta^2}\lfloor\boldsymbol{\phi}\rfloor_\times^2 \equiv \exp(\lfloor\boldsymbol{\phi}\rfloor_\times)$ | Checked vs `scipy.linalg.expm` (diff $< 10^{-13}$) | **PASS** |
| **8. Stationary Gravity Balance** | $\mathbf{a}^n = \mathbf{R}_b^n \mathbf{f}^b + \mathbf{g}^n = \mathbf{0}$, position/velocity static | 20 seconds stationary integration ($<10^{-12}\text{ m/s}$) | **PASS** |
| **9. Linear Kinematic Equations** | $p(t) = 0.5 a t^2, \ v(t) = a t$ | 5.0s constant acceleration ($a_{\text{long}}=2\text{ m/s}^2$) | **PASS** |
| **10. Kinetic Deceleration / Braking** | $v(t) = v_0 - a_{\text{brake}} t, \ p(t) = v_0 t - 0.5 a t^2$ | Braking from $20\text{ m/s}$ to $0\text{ m/s}$ at $-4\text{ m/s}^2$ | **PASS** |
| **11. Bank & Elevation Handling** | $10^\circ$ pitch up (hill climb) & $15^\circ$ roll (banked turn) | 3D attitude specific-force static equilibrium | **PASS** |
| **12. Vibration & Shock Immunity** | High-frequency noise + $>3.5g$ impulse pothole shocks | 100-step filter stability (no NaNs, no Infs) | **PASS** |
| **13. Standstill ZUPT Clamping** | Velocity collapsed to $<10^{-3}\text{ m/s}$, variance to $<10^{-4}$ | Standstill zero-velocity update sequence | **PASS** |
| **14. Bias Estimation Observability** | Accelerometer $\mathbf{b}_a$ & gyro $\mathbf{b}_g$ state tracking | 30s biased flight under position/velocity aiding | **PASS** |
| **15. AI Velocity Jacobian Sign** | $\mathbf{H}_v = [\mathbf{0}, \mathbf{R}[:, 0]^T, \mathbf{R}[:, 0]^T \lfloor\mathbf{v}^n\rfloor_\times, \mathbf{0}, \mathbf{0}]$ | Finite-difference numerical perturbation ($<10^{-5}$) | **PASS** |
| **16. NHC Velocity & Att Jacobians** | $\mathbf{H}_{\text{nhc}} = [\mathbf{0}, \mathbf{R}[:, 1:3]^T, \mathbf{R}[:, 1:3]^T \lfloor\mathbf{v}^n\rfloor_\times, \mathbf{0}, \mathbf{0}]$ | 2-DOF finite-difference perturbation ($<10^{-5}$) | **PASS** |
| **17. Heading Jacobian & Innovation** | $H_{\psi}[8] = -1.0, \ dz = \psi_{\text{meas}} - \psi_{\text{prior}}$ | Convergence without overshoot or sign inversion | **PASS** |
| **18. Covariance Stability & Symmetry** | $\mathbf{P} = \mathbf{P}^T, \ \lambda_{\min}(\mathbf{P}) > 0$ across 1000 mixed cycles | Symmetrization error $<10^{-12}$, all eigenvalues $>0$ | **PASS** |
| **19. Seamless GNSS Transition** | Handover cycle (Aided $\to$ Outage $\to$ Reacquisition) | Covariance expansion in blackout; smooth recovery | **PASS** |
| **20. Real IO-VNBD Subset Execution** | Real logs (`Driver_A_S3b` 681s, `Vta01b` 95s) | Finite outputs, zero NaNs/Infs, valid ENU trajectory | **PASS** |

---

## 3. Defect Identification & Fixes Applied in Phase 1

### Defect 1: Attitude Error Jacobian Sign in AI Velocity and NHC Updates
- **Discovery:** In `test_13_ai_velocity_jacobian_numerical_check`, the analytical attitude Jacobian `H[0, 6:9]` was `[-1.4488, 0.6914, -3.6894]`, while numerical finite difference perturbation yielded `[+1.4488, -0.6914, +3.6894]`.
- **Root Cause Analysis:** The first column of $\mathbf{R}_b^n$ is the vehicle forward vector $\mathbf{r}_1$. In navigation frame error-state representation $\mathbf{R}_{\text{true}} \approx (\mathbf{I} + \lfloor\delta\boldsymbol{\theta}^n\rfloor_\times) \mathbf{R}$, the perturbation of body forward velocity is:
  $$\delta v_x^b = (\mathbf{r}_1 \times \mathbf{v}^n)^T \delta\boldsymbol{\theta}^n = \mathbf{r}_1^T \lfloor\mathbf{v}^n\rfloor_\times \delta\boldsymbol{\theta}^n$$
  The original code included an erroneous negative sign (`-self.R[:, 0] @ skew_symmetric(self.v)`), inverting the attitude feedback during velocity updates.
- **Fix:** Corrected lines 227, 242, and 245 in `idr_pipeline/eskf_fusion.py` to `+self.R[:, i] @ skew_symmetric(self.v)`.
- **Validation:** Analytical Jacobians now match finite-difference numerical perturbations to $< 10^{-5}$ tolerance across all axes.

### Defect 2: Missing GNSS Velocity Update Method
- **Discovery:** The filter only supported 3D position GNSS updates (`update_gnss`), lacking a dedicated Doppler / GNSS velocity update.
- **Fix:** Implemented `update_gnss_velocity(self, gnss_vel: np.ndarray, r_vel: float = 0.2)` in `idr_pipeline/eskf_fusion.py`.

---

## 4. Test Suite Execution Summary

- **Total Tests Executed:** 33 tests across `tests/test_phase1_navigation_core.py` (19 tests) and `tests/test_eskf_fusion.py` (14 tests).
- **Test Results:** **33 Passed / 0 Failed**.
- **Integration Test Suite (`test_suite.py`):** **6 Passed / 0 Failed** (Edge engine throughput: 1064.3 Hz).
- **No Regressions:** All baseline tests and Step 4C invariants preserved.

---

## 5. Phase 1 Sign-Off & Next Steps

All 20 Phase 1 mathematical validation objectives have passed. The navigation filter core is verified, orthonormal, and physically consistent.

**Next Phase:** **PHASE 2 — ALIGNMENT & INITIALIZATION HARDENING** (Multi-mode alignment engine, stationary start protection, confidence quality metrics).
