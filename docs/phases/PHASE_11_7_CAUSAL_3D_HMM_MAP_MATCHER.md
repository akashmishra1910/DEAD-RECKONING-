# Phase 11.7 — Causal 3D HMM Map Matching & Map-Aided Dead Reckoning

## 1. Executive Summary & Status
- **Phase**: 11.7 — Causal 3D HMM Map Matching & Map-Aided Dead Reckoning
- **System**: ISRO / SIH PS 26168 AI/ML Based Intelligent Dead Reckoning System
- **Status**: COMPLETE & VERIFIED
- **Frozen Track B Baseline**:
  - $C_0 = 76.34\%$ median drift (12/158 pass)
  - $N_0 = 34.43\%$ median drift (28/158 pass)
  - $N_7 = 35.71\%$ median drift (36/158 pass)
  - *Estimator, GBDT, and ESKF logic remain 100% frozen.*

---

## 2. Architectural Overview

Phase 11.7 integrates real-time causal 3D map matching into `idr-core`, supporting both compiled `.idrmap` v1.0 binary files (`IdrMapReader`) and in-memory topological graphs (`OfflineOSMGraph`).

```text
                  Sensor Ingestion / ESKF
                            │
            (p_x, p_y, p_z, heading, cov, dt)
                            ▼
              ┌───────────────────────────┐
              │    Causal3DMapMatcher     │
              │  (ISO C++17 / Zero-Alloc) │
              └─────────────┬─────────────┘
                            │
         ┌──────────────────┴──────────────────┐
         ▼                                     ▼
┌─────────────────┐                   ┌─────────────────┐
│  .idrmap v1.0   │                   │ OfflineOSMGraph │
│ (IdrMapReader)  │                   │  (In-Memory)    │
└─────────────────┘                   └─────────────────┘
         │                                     │
         └──────────────────┬──────────────────┘
                            ▼
              ┌───────────────────────────┐
              │ Multi-Hypothesis Tracking │
              │      (K <= 8 states)      │
              └─────────────┬─────────────┘
                            │
     ┌──────────────────────┼──────────────────────┐
     ▼                      ▼                      ▼
Cross-Track Emission   Elevation Barrier     Topological Transition
(Gaussian + Heading)  (|Δh| > 4m penalty)     (Shortest path vs Δd)
                            │
                            ▼
              ┌───────────────────────────┐
              │   Posterior Pruning &     │
              │   Ambiguity Detection     │
              └─────────────┬─────────────┘
                            │
                            ▼
                   MapMatchResult / C API
```

---

## 3. Mathematical Formulation

### 3.1 State Representation
A map matching hypothesis $s_{t,i}$ is defined as:
$$s_{t,i} = \left( e_{t,i}, x_{t,i}, z_{t,i}, \theta_{t,i}, S_{t,i} \right)$$
where:
- $e_{t,i}$: Edge / road segment ID
- $x_{t,i}$: Along-track distance along the edge (meters, $x \in [0, L_e]$)
- $z_{t,i}$: Projected 3D coordinate $[p_E, p_N, p_U]^T$
- $\theta_{t,i}$: Road segment heading (radians)
- $S_{t,i}$: Cumulative log-posterior score

### 3.2 Emission Likelihood
The emission log-likelihood $\ln P(z_t \mid s_t)$ combines cross-track spatial error, heading alignment error, and vertical layer consistency:

$$\ln P(z_t \mid s_t) = \ln P_{\text{cross}}(d_{\perp}) + \ln P_{\text{head}}(\Delta \psi) + \ln P_{\text{alt}}(\Delta h)$$

1. **Cross-Track Distance ($d_{\perp}$)**:
   $$\ln P_{\text{cross}}(d_{\perp}) = -\frac{1}{2}\left(\frac{d_{\perp}}{\sigma_d}\right)^2 - \ln(\sqrt{2\pi}\sigma_d)$$
   where $\sigma_d = \max(3.0, \sqrt{\sigma_{\text{nav}}^2 + \sigma_{\text{map}}^2})$ with $\sigma_{\text{map}} = 2.0\text{ m}$.

2. **Heading Discrepancy ($\Delta \psi$)**:
   For bidirectional edges: $\Delta \psi = \min(|\psi_{\text{nav}} - \psi_{\text{road}}|, |\psi_{\text{nav}} - (\psi_{\text{road}} + \pi)|)$.
   For one-way edges: $\Delta \psi = |\psi_{\text{nav}} - \psi_{\text{road}}|$.
   $$\ln P_{\text{head}}(\Delta \psi) = -\frac{1}{2}\left(\frac{\Delta \psi}{\sigma_{\psi}}\right)^2$$
   where $\sigma_{\psi} = 0.35\text{ rad} \approx 20^\circ$.

3. **3D Elevation Barrier ($\Delta h$)**:
   Let $\Delta h = |h_{\text{nav}} - h_{\text{road}}|$.
   $$\ln P_{\text{alt}}(\Delta h) = \begin{cases} -\frac{1}{2}\left(\frac{\Delta h}{\sigma_h}\right)^2, & \Delta h \le 4.0\text{ m} \\ -50.0 - 5.0(\Delta h - 4.0), & \Delta h > 4.0\text{ m} \end{cases}$$
   where $\sigma_h = 2.0\text{ m}$.

### 3.3 Topological Transition Model
For hypothesis transition $s_{t-1, j} \to s_{t, i}$:
1. **Direct Trajectory / Same Edge**:
   $$\Delta d = |x_{t,i} - x_{t-1,j}|, \quad d_{\text{DR}} = v \cdot \Delta t$$
   $$\ln P(s_{t,i} \mid s_{t-1,j}) = -\frac{|\Delta d - d_{\text{DR}}|}{\beta_{\text{trans}}}$$
   where $\beta_{\text{trans}} = 3.0\text{ m}$.

2. **Topologically Connected Edges**:
   Shortest path distance $d_{\text{top}}$ via BFS topological routing over road graph.
   $$\ln P(s_{t,i} \mid s_{t-1,j}) = -\frac{|d_{\text{top}} - d_{\text{DR}}|}{\beta_{\text{trans}}}$$

3. **Disconnected / Impossible Jump**:
   If $d_{\text{top}} = \infty$ or exceeds search depth:
   $$\ln P(s_{t,i} \mid s_{t-1,j}) = -80.0$$

### 3.4 Causal Viterbi Multi-Hypothesis Recursion
For each current candidate $i \in [1, M]$:
$$S_{t, i} = \max_{j \in [1, K]} \left[ S_{t-1, j} + \ln P(s_{t,i} \mid s_{t-1,j}) \right] + \ln P(z_t \mid s_{t,i})$$

1. **Pruning**: Retain at most $K = 8$ hypotheses sorted by score in descending order.
2. **Deterministic Tie-Breaking**:
   $$\text{Comparator}(a, b) \iff (S_a > S_b) \lor (S_a == S_b \land e_a < e_b) \lor (S_a == S_b \land e_a == e_b \land x_a < x_b)$$
3. **Score Normalization**: Subtract $\max_k(S_{t,k})$ across active hypotheses to eliminate numerical underflow across long operating sessions.

### 3.5 Ambiguity & Confidence Classification
- **Score Margin**: $\Delta S = S_{t, 1} - S_{t, 2}$
- **Geometric Separation**: $d_{12} = \|z_{t,1} - z_{t,2}\|_2$
- **Ambiguity Rule**: If $\Delta S < 1.5$ and $d_{12} < 20.0\text{ m}$, then `is_ambiguous = true`.
- **Confidence Levels**:
  - `HIGH`: $d_{\perp} < 5.0\text{ m} \land \Delta \psi < 25^\circ \land \Delta S \ge 2.0 \land \neg \text{is\_ambiguous}$
  - `MEDIUM`: $d_{\perp} < 15.0\text{ m} \land \Delta \psi < 45^\circ$
  - `LOW`: Candidate exists but $d_{\perp} \ge 15.0\text{ m}$ or $\Delta \psi \ge 45^\circ$
  - `INVALID`: No candidates found within spatial search radius ($50.0\text{ m}$).

---

## 4. Verification & Parity Results

### 4.1 C++ Verification Suite (`test_causal_hmm.cpp`)
All 21 comprehensive unit test stages pass:
```text
[ RUN      ] Single obvious road match
[       OK ] Passed
[ RUN      ] Parallel roads separation
[       OK ] Passed
[ RUN      ] Crossing roads resolution
[       OK ] Passed
[ RUN      ] Same-layer parallel ambiguity
[       OK ] Passed
[ RUN      ] Bridge elevation barrier (|Δh| > 4m)
[       OK ] Passed
[ RUN      ] Tunnel identification and matching
[       OK ] Passed
[ RUN      ] Road junction transition
[       OK ] Passed
[ RUN      ] One-way road direction penalty
[       OK ] Passed
[ RUN      ] Disconnected road jump rejection
[       OK ] Passed
[ RUN      ] Ambiguous candidate pair detection
[       OK ] Passed
[ RUN      ] Temporary candidate loss and recovery
[       OK ] Passed
[ RUN      ] GNSS outage transition resilience
[       OK ] Passed
[ RUN      ] GNSS recovery and reacquisition
[       OK ] Passed
[ RUN      ] High navigation uncertainty adaptation
[       OK ] Passed
[ RUN      ] Low-speed movement handling
[       OK ] Passed
[ RUN      ] High-speed movement tracking
[       OK ] Passed
[ RUN      ] Heading boundary wrap (0 <-> 2pi)
[       OK ] Passed
[ RUN      ] Invalid sensor observations rejection
[       OK ] Passed
[ RUN      ] Repeated identical observations stability
[       OK ] Passed
[ RUN      ] Anti-lookahead bitwise invariance proof
[       OK ] Passed
[ RUN      ] Micro-benchmarks (Zero alloc & Latency)
[       OK ] Processed 10,000 steps in 37.26 ms (3.72 us/step, 268,369 Hz)
======================================================
ALL 21 CAUSAL 3D HMM MAP MATCHER TESTS PASSED!
======================================================
```

### 4.2 Python Parity & Regression Suite (`pytest`)
All 186 test cases in the repository pass cleanly:
```text
====================== 186 passed, 8 warnings in 58.27s =======================
```

---

## 5. Performance & Resource Budgets

| Metric | Measured Value | Budget Constraint | Margin |
|---|---|---|---|
| Step Latency (mmap `.idrmap`) | $3.72\text{ \mu s}$ | $< 500\text{ \mu s}$ (10 Hz Android) | **$134\times$ margin** |
| Step Latency (Topological Graph) | $4.15\text{ \mu s}$ | $< 5.0\text{ ms}$ (200 Hz Edge) | **$1200\times$ margin** |
| Dynamic Heap Allocations / step | **0 bytes** | 0 bytes | **Zero heap churn** |
| Max Active Hypotheses | **8** | $\le 16$ | Bounded $O(1)$ memory |
| Anti-lookahead bitwise invariance | **$100\%$ identical** | Exact parity | **Zero future leakage** |

---

## 6. Exported C API Interface

```c
idr_map_matcher_t* idr_map_matcher_create_default(const idr_osm_graph_t* graph);
idr_map_matcher_t* idr_map_matcher_create_with_reader(const idr_map_reader_t* reader);
void idr_map_matcher_destroy(idr_map_matcher_t* matcher);
void idr_map_matcher_reset(idr_map_matcher_t* matcher);

idr_status_t idr_map_matcher_process_step(
    idr_map_matcher_t* matcher,
    const idr_map_match_request_t* request,
    idr_map_match_result_c_t* out_result
);

uint32_t idr_map_matcher_get_hypotheses(
    const idr_map_matcher_t* matcher,
    idr_map_hypothesis_c_t* out_hypotheses,
    uint32_t max_count
);
```

---

## 7. Confirmation of Frozen Baselines
- Track B baselines ($C_0 = 76.34\%$, $N_0 = 34.43\%$, $N_7 = 35.71\%$) remain strictly intact.
- No GBDT features, tree weights, or ESKF update matrices were altered.
- All 158 synthetic outage runs remain reproducible.
