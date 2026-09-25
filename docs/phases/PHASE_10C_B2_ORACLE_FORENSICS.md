# Phase 10C: B2 Oracle Forensic Analysis & Information Audit

**Author:** Autonomous Navigation Software Lead  
**Date:** September 23, 2026  
**Repository State:** Baseline Commit `158e280`, Working Tree CLEAN  

---

## 1. Mathematical Formulation of the B2 Oracle

In Phase 10B, the diagnostic oracle experiment $B_2$ achieved **3.52% median drift** and an **89.24% pass rate** (<10% drift). To understand why $B_2$ performed so well and why it is not a deployable solution, we trace its mathematical and algorithmic construction.

### 1.1 Measurement Injection in B2
In standard Error-State Kalman Filtering for road vehicles, Non-Holonomic Constraints (NHC) assume:
$$v_y^b \approx 0 \quad (\text{zero lateral sliding / no tire slip})$$
$$v_z^b \approx 0 \quad (\text{zero vertical jumping / road contact})$$

Under normal NHC, the observation vector is:
$$z_{\text{nhc}} = \begin{bmatrix} 0 \\ 0 \end{bmatrix}, \quad H = \begin{bmatrix} \mathbf{r}_{\text{left}}^T \\ \mathbf{r}_{\text{up}}^T \end{bmatrix}_{2 \times 3} \text{ on velocity state } \delta \mathbf{v}^n, \quad R = \begin{bmatrix} \sigma_{\text{lat}}^2 & 0 \\ 0 & \sigma_{\text{vert}}^2 \end{bmatrix}$$
where $\sigma_{\text{lat}} = \sigma_{\text{vert}} = 0.05\text{ m/s}$.

In experiment $B_2$, the measurement vector $z$ was replaced by the **true vehicle body velocity vector** derived from ground truth:
$$\mathbf{v}_{\text{true}}^b(t) = (R_b^n(t))^T \mathbf{v}_{\text{gt}}^n(t) = \begin{bmatrix} v_{x, \text{true}}^b(t) \\ v_{y, \text{true}}^b(t) \\ v_{z, \text{true}}^b(t) \end{bmatrix}$$
$$z_{B2}(t) = \begin{bmatrix} v_{y, \text{true}}^b(t) \\ v_{z, \text{true}}^b(t) \end{bmatrix}, \quad R_{B2} = \begin{bmatrix} 0.02^2 & 0 \\ 0 & 0.02^2 \end{bmatrix}$$

---

## 2. Information Asymmetry: What B2 Knows vs What a Real Vehicle Knows

| Dimension | Standard Real Estimator | B2 Diagnostic Oracle | Real-World Feasibility |
|---|---|---|---|
| **Lateral Velocity ($v_y^b$)** | Assumed strictly $0.0\text{ m/s}$ | True instantaneous slip velocity $v_{y, \text{gt}}^b$ | **Unobtainable without side-slip sensors/optical odometry** |
| **Vertical Velocity ($v_z^b$)** | Assumed strictly $0.0\text{ m/s}$ | True instantaneous vertical velocity $v_{z, \text{gt}}^b$ | **Unobtainable without high-rate vertical radar/altimetry** |
| **Source Data** | Commodity Phone IMU + AI forward speed | RTK-GPS / Vehicle ECU ground truth $\mathbf{v}_{\text{gt}}^n$ | **Zero ground truth available during blackout** |
| **Measurement Covariance** | Fixed $R_{\text{lat}} = 0.05^2 = 0.0025\text{ m}^2/\text{s}^2$ | Ultra-tight $R_{B2} = 0.02^2 = 0.0004\text{ m}^2/\text{s}^2$ | **Overconfident if measurement is noisy/modeled** |

---

## 3. Why Rigid NHC ($v_y^b \approx 0$) Fails on Real Vehicles

1. **Centripetal Acceleration & Tire Slip Angles:**
   When a vehicle enters a curve of radius $R$ at speed $v$, the centripetal acceleration $a_{\text{cent}} = v \omega_z$ requires lateral tire forces $F_y = m v \omega_z$. By Pacejka's tire mechanics, lateral force requires a non-zero tire side-slip angle $\alpha$:
   $$\alpha \approx \frac{m v \omega_z}{C_\alpha} \implies v_y^b \approx -v \sin \alpha \ne 0$$
2. **Body Roll Under Lateral Dynamics:**
   Chassis suspension roll under lateral acceleration tilts the phone body frame relative to the wheel contact patch, inducing an apparent lateral velocity $v_y^b = h_{\text{CG}} \dot{\phi}_{\text{roll}}$.
3. **The Damage of Rigid NHC Updates:**
   When the real vehicle has $v_y^b = 0.8\text{ m/s}$ in a curve, forcing $z = 0$ with $\sigma = 0.05\text{ m/s}$ generates an artificial innovation $\delta z = -0.8\text{ m/s}$. The Kalman filter corrects this by violently altering the estimated heading and longitudinal velocity states, creating artificial cross-track and along-track drift.

---

## 4. Architectural Strategy for Causal Phase 10C

Instead of attempting to synthesize the unobtainable true lateral velocity $v_{y, \text{true}}^b$, a **causal deployable estimator must adapt its uncertainty $R_{\text{lat}}$**:
- In **straight-line cruising** ($\omega_z \approx 0, a_y \approx 0$), $v_y^b \approx 0$ is physically accurate $\implies$ enforce tight $R_{\text{lat}} = R_0$ to prevent drift.
- In **cornering & dynamic maneuvers** ($|\omega_z| > 0.05\text{ rad/s}$ or $|a_y| > 0.5\text{ m/s}^2$), tire slip is active $\implies$ **dynamically relax $R_{\text{lat}} \rightarrow R_{\text{max}}$**, preventing the filter from corrupting the navigation state.
