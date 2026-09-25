# 05 — NAVIGATION ENGINE & KALMAN FILTERING
## 15-State Error-State Kalman Filter & Motion Constraints

---

### 1. State Vector Formulation
The core filter is formulated as a 15-state continuous-discrete Error-State Kalman Filter (ESKF):
$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^n \\ \mathbf{v}^n \\ \mathbf{q}_b^n \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$
- $\mathbf{p}^n = [p_x, p_y, p_z]^T$: Position in East-North-Up (ENU) navigation frame.
- $\mathbf{v}^n = [v_x, v_y, v_z]^T$: Velocity in ENU frame.
- $\mathbf{q}_b^n$: Unit quaternion representing body-to-navigation rotation.
- $\mathbf{b}_a = [b_{ax}, b_{ay}, b_{az}]^T$: Accelerometer bias drift states.
- $\mathbf{b}_g = [b_{gx}, b_{gy}, b_{gz}]^T$: Gyroscope bias drift states.

### 2. Multi-Modal Measurement Updates
1. **AI Speed Update:** Forward speed pseudo-measurement from GBDT:
   $$z_{\text{ai}} = \mathbf{e}_x^T (\mathbf{R}_b^n)^T \mathbf{v}^n + \eta_{\text{ai}}$$
2. **Non-Holonomic Constraints (NHC):** Zero lateral and vertical velocity in vehicle frame:
   $$v_y^b \approx 0, \quad v_z^b \approx 0, \quad \mathbf{R}_{\text{nhc}} = \operatorname{diag}(\sigma_{\text{lat}}^2, \sigma_{\text{vert}}^2)$$
3. **Standstill Zero-Velocity Updates (ZUPT):** Energy variance detector clamps velocity to zero when stationary, eliminating stop-and-go drift.
4. **Chi-Square Innovation Gating ($\chi^2$):** Rejects spurious GNSS position fixes during tunnel exit:
   $$\gamma = \mathbf{r}^T \mathbf{S}^{-1} \mathbf{r} \le 11.35 \quad (\alpha=0.01, \text{dof}=3)$$

### 3. C++ Execution Performance
- **Per-Sample Filter Execution Latency:** **$17.85\ \mu\text{s}$ (0.018 ms)**.
- **Maximum Resident Memory (RSS):** **$<2.4\text{ MB}$**.
- **Dynamic Heap Growth:** Zero allocations in the high-rate streaming loop.
