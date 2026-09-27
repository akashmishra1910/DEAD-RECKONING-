# PART 2 MAP MATCHING SPECIFICATION
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Causal 3D Probabilistic Map Matching & Elevation Barrier Specification

**Document Version:** 1.0.0  
**Phase:** Part 2 of 3  
**Status:** Authoritative Technical Specification  
**Baseline:** B1/N8 + M6 Autonomous Dead-Reckoning Core  
**Implementation:** `idr_pipeline/map_matcher_3d.py` (Python), `idr-core/src/map_matcher_3d.cpp` (C++17)

---

## 1. Overview & Operational Problem

In multi-tiered urban environments—such as flyovers, elevated highways, underpasses, and parallel service roads—traditional 2D map matchers suffer catastrophic failure modes:
1. **Flyover / Ground Road Confusion:** When an elevated flyover runs directly above a surface road ($x, y$ coordinates identical within 1–2 meters), 2D matchers snap arbitrarily between levels, introducing massive vertical and heading discontinuities.
2. **Parallel Road Ambiguity:** Frontage service roads parallel to expressways ($8–15\text{ m}$ lateral separation) have identical orientation, causing cross-track snapping errors if cross-track position is forcibly overwritten.
3. **Non-Causal Smoothing Fallacy:** Offline batch algorithms (such as backward Viterbi smoothing) require knowing future vehicle positions, violating real-time navigation constraints.

The **Causal 3D HMM Map Matcher** resolves these challenges by:
- Operating in forward-only recursive mode with zero future lookahead.
- Enforcing a strict **3D Elevation Barrier ($|\Delta h| > 4.0\text{ m}$)** that suppresses impossible cross-level transitions.
- Treating road matches as **soft Kalman measurement updates** rather than hard position overwrites.

---

## 2. Spatial Indexing & Candidate Generation

### 2.1 Uniform Grid Spatial Indexing
To support high-rate execution without expensive global searches, both Python and C++ engines maintain a 2D spatial grid index:
- **Grid Cell Dimensions:** $\Delta X_{\text{cell}} = 60.0\text{ m}$, $\Delta Y_{\text{cell}} = 60.0\text{ m}$.
- **Candidate Query:** Given the current vehicle dead-reckoning position $\mathbf{p}_{\text{dr}} = [x, y, z]^T$, the query inspects all grid cells within radius $R_{\text{search}} = 75.0\text{ m}$:
  $$k_x \in \left[\lfloor (x - R) / \Delta X \rfloor, \, \lfloor (x + R) / \Delta X \rfloor\right]$$
  $$k_y \in \left[\lfloor (y - R) / \Delta Y \rfloor, \, \lfloor (y + R) / \Delta Y \rfloor\right]$$
- **Candidate Bounding:** Returns at most 32 candidate road edges (`MAX_CANDIDATES = 32`).

### 2.2 Point-to-Edge Geometric Projection
For each candidate edge $e$:
1. Subsegment coordinates $\mathbf{p}_1, \mathbf{p}_2 \in \mathbb{R}^3$.
2. Vector $\mathbf{v} = \mathbf{p}_2 - \mathbf{p}_1$, 2D length squared $L_{2D}^2 = v_x^2 + v_y^2$.
3. Projection scalar:
   $$t = \text{clamp}\left(\frac{(x - p_{1,x})v_x + (y - p_{1,y})v_y}{L_{2D}^2}, \, 0.0, \, 1.0\right)$$
4. 3D Snapped point: $\mathbf{p}_{\text{snap}} = \mathbf{p}_1 + t \, \mathbf{v}$.
5. Lateral 2D distance: $d_{2D} = \sqrt{(x - p_{\text{snap},x})^2 + (y - p_{\text{snap},y})^2}$.
6. Vertical elevation difference: $\Delta h = |z - p_{\text{snap},z}|$.
7. Road azimuth: $\psi_{\text{road}} = \text{atan2}(v_x, v_y) \pmod{2\pi}$.
8. Unit road tangent: $\mathbf{t}_{\text{road}} = [v_x / \sqrt{L_{2D}^2}, \, v_y / \sqrt{L_{2D}^2}, \, 0]^T$.

---

## 3. Mathematical Emission Likelihood Formulation

The log emission likelihood $\log P(\mathbf{z}_k | e)$ measures the physical compatibility between the dead-reckoning state $\mathbf{z}_k = [\mathbf{p}_{\text{dr}}, \psi_{\text{dr}}, h_{\text{baro}}]^T$ and road candidate $e$:

$$\log P(\mathbf{z}_k | e) = \log P_{\text{dist}}(d_{2D}) + \log P_{\text{hdg}}(d_\psi) + \log P_{\text{alt}}(\Delta h)$$

### 3.1 Lateral Distance Likelihood
$$\log P_{\text{dist}}(d_{2D}) = -\frac{1}{2}\left(\frac{\min(d_{2D}, 50.0)}{\sigma_d}\right)^2, \quad \sigma_d = 5.0\text{ m}$$

### 3.2 Heading Alignment Likelihood
$$\log P_{\text{hdg}}(d_\psi) = -\frac{1}{2}\left(\frac{\min(d_\psi, 1.0)}{\sigma_\psi}\right)^2, \quad \sigma_\psi = 0.30\text{ rad} \approx 17.2^\circ$$
where $d_\psi = |\text{wrap\_pi}(\psi_{\text{dr}} - \psi_{\text{road}})|$.

### 3.3 3D Elevation Barrier ($|\Delta h| > 4.0\text{ m}$)
The barometric altitude measurement $h_{\text{baro}}$ is compared against road deck elevation $p_{\text{snap},z}$:
$$\log P_{\text{alt}}(\Delta h) = \begin{cases}
-\frac{1}{2}\left(\frac{\Delta h}{\sigma_h}\right)^2 & \text{if } \Delta h \le 4.0\text{ m} \quad (\sigma_h = 1.8\text{ m}) \\
-50.0 - 5.0 \cdot (\Delta h - 4.0) & \text{if } \Delta h > 4.0\text{ m}
\end{cases}$$

#### Why 4.0 m is the Analytically Optimal Barrier Threshold:
- **Vehicle Height & Barometric Noise:** Standard vehicle roof height + sensor mounting elevation variation is $< 1.8\text{ m}$. Barometer noise under highway aerodynamic buffeting is $\sigma_{\text{baro}} \approx 0.8\text{ m}$ ($3\sigma \approx 2.4\text{ m}$).
- **Flyover Clearance Standards:** Indian Road Congress (IRC) and AASHTO civil engineering standards mandate minimum vertical clearance for flyovers and overpasses of $5.5\text{ m}$ (typically $8.5\text{ m}$ deck-to-ground separation).
- **Discrimination:** A threshold of $4.0\text{ m}$ lies cleanly between the maximum sensor noise envelope ($2.4\text{ m}$) and the minimum civil clearance ($5.5\text{ m}$), ensuring that surface vehicles never match to overhead decks.

---

## 4. Topological Transition Model

For transitions between previous matched edge $e_{k-1}$ and current candidate $e_k$:
$$\log P(e_k | e_{k-1}) = \begin{cases}
-\frac{|\Delta d_{\text{network}} - \Delta d_{\text{DR}}|}{\beta} & \text{if } e_k \text{ is topologically connected to } e_{k-1} \\
-15.0 & \text{if } e_k \text{ is disconnected from } e_{k-1}
\end{cases}$$
where $\beta = 5.0\text{ m}$ and $\Delta d_{\text{DR}} = \|\mathbf{p}_{\text{dr}}(t_k) - \mathbf{p}_{\text{dr}}(t_{k-1})\|$.

---

## 5. Causal Forward Viterbi Recursion & Pruning

At each epoch $k$:
1. Active hypotheses $\mathcal{H}_{k-1}$ (at most 8 edges, `MAX_HYPOTHESES = 8`).
2. For each current candidate $e \in \mathcal{C}_k$:
   $$V_k(e) = \log P(\mathbf{z}_k | e) + \max_{h \in \mathcal{H}_{k-1}} \left[ V_{k-1}(h) + \log P(e | h) \right]$$
3. Compute posterior probabilities via softmax:
   $$W(e) = \frac{\exp(V_k(e) - V_{\max})}{\sum_{c} \exp(V_k(c) - V_{\max})}$$
4. Best hypothesis: $e^* = \arg\max_e W(e)$.
5. Score margin between top and runner-up:
   $$\Delta\text{score} = V_k(e^*) - V_k(e_{\text{second}})$$
   If $\Delta\text{score} < 1.5$, the system flags **Ambiguous Road Network** (`is_ambiguous = True`).

---

## 6. Soft Road Constraints Injection into 15-State ESKF

Rather than forcing $\mathbf{p} \leftarrow \mathbf{p}_{\text{snap}}$, the map matcher acts as a sensor supplying bounded Kalman observations:

### 6.1 Road Cross-Track Position Update
Observes vehicle position along the road normal unit vector $\mathbf{n}_{\text{road}} = [-t_y, t_x, 0]^T$:
$$\delta z_{\text{cross}} = \mathbf{n}_{\text{road}}^T (\mathbf{p}_{\text{snap}} - \mathbf{p}_{\text{dr}})$$
Measurement Jacobian:
$$\mathbf{H}_{\text{cross}} = \begin{bmatrix} \mathbf{n}_{\text{road}}^T & \mathbf{0}_{1\times 12} \end{bmatrix} \in \mathbb{R}^{1 \times 15}$$
Measurement Variance:
$$R_{\text{cross}} = \begin{cases} (1.75\text{ m})^2 & \text{if } C_{\text{map}} \ge 0.70 \text{ (`HIGH`)} \\ (3.50\text{ m})^2 & \text{if } 0.35 \le C_{\text{map}} < 0.70 \text{ (`MEDIUM`)} \end{cases}$$
Gated by:
$$\gamma_{\text{cross}} = \frac{\delta z_{\text{cross}}^2}{\mathbf{H}_{\text{cross}} \mathbf{P} \mathbf{H}_{\text{cross}}^T + R_{\text{cross}}} \le 16.0$$

### 6.2 Road Heading Update
Observes road azimuth $\psi_{\text{road}}$:
$$\delta z_{\psi} = \text{wrap\_pi}(\psi_{\text{road}} - \psi_{\text{dr}})$$
Measurement Jacobian:
$$\mathbf{H}_{\psi} = \begin{bmatrix} \mathbf{0}_{1\times 8} & 1.0 & \mathbf{0}_{1\times 6} \end{bmatrix} \in \mathbb{R}^{1 \times 15}$$
Measurement Variance:
$$R_{\text{hdg}} = \begin{cases} (0.087\text{ rad})^2 \approx (5^\circ)^2 & \text{if } C_{\text{map}} \ge 0.70 \\ (0.174\text{ rad})^2 \approx (10^\circ)^2 & \text{if } 0.35 \le C_{\text{map}} < 0.70 \end{cases}$$
Gated by $\gamma_{\psi} \le 9.0$ and vehicle speed $v > 2.0\text{ m/s}$.
