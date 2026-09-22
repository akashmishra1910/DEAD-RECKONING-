# Architecture Decision Records (ADRs)
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

---

### ADR-001: Coordinate Frame Conventions — Standard FLU Body & ENU Navigation Frames
- **Status:** APPROVED & VERIFIED (Step 4B-C)
- **Context:** The legacy implementation used a Left-Handed FRU reflection frame with $\det(\mathbf{R}_b^n) = -1.0$, which inverted lateral kinematics and caused mathematical inconsistencies during 3D attitude propagation.
- **Decision:** Adopt standard right-handed **FLU Body Frame** (+X Forward, +Y Left, +Z Up) and **ENU Navigation Frame** (+X East, +Y North, +Z Up). Enforce $\mathbf{R}_b^n \in \text{SO}(3)$ with $\det(\mathbf{R}_b^n) = +1.0$ and full 3D Rodrigues rotation vector propagation.
- **Consequences:** All unit and integration tests now pass with strict numerical orthonormality ($<10^{-15}$). Legacy FRU / det=-1 conventions are strictly prohibited.

---

### ADR-002: Zero External ML Runtime — Transpiled Static C++ Decision Trees
- **Status:** APPROVED (Required by PS & Design)
- **Context:** Including PyTorch Mobile, ONNX Runtime, or TensorFlow Lite adds 20–50 MB of binary overhead, unpredictable garbage collection latency, and complex NDK dependencies.
- **Decision:** Transpile the GBDT ensemble directly into static C++ decision tables evaluated via branchless array lookups in native code.
- **Consequences:** Binary overhead is $<600\text{ KB}$, per-cycle inference latency is $<50\ \mu\text{s}$, RAM footprint is $<2.4\text{ MB}$, and zero third-party ML runtime is required.

---

### ADR-003: Model Checkpoint Freeze — Top-45 GBDT Model
- **Status:** APPROVED & FROZEN (Step 3E)
- **Context:** Checkpoint 3E evaluated 5 ablation architectures across 72 IO-VNBD naturalistic drives and established the Top-45 candidate as optimal (Test RMSE: 5.001 m/s, $R^2$: 0.6069, 0–60 km/h RMSE: 4.894 m/s).
- **Decision:** Freeze `models/gbdt_speed_model_iovnbd_top45.joblib`. No retraining or hyperparameter changes are permitted without an authorized phase gate.
- **Consequences:** Provides a fixed baseline for C++ edge porting and filter benchmarking.

---

### ADR-004: Dual-Deliverable Repository Layout
- **Status:** APPROVED (Mandated by PS 26168)
- **Context:** The PS explicitly requires two deliverables: a 10 Hz Android Mobile Application and an Edge-deployable software engine running at ~200 Hz with FOG IMUs.
- **Decision:** Structure the core algorithms in `idr-core/` as pure C++17 with CMake (zero Android or Python dependencies). `idr-android/` provides the Kotlin/JNI mobile frontend, and `idr-cli/` provides the edge socket/file binary.
- **Consequences:** Ensures complete separation of algorithmic core from mobile UI and meets both deliverable mandates simultaneously.

---

### ADR-005: 3D Elevation Barrier ($\|\Delta h\| > 4.0\text{ m}$) for Multi-Level Road Disambiguation
- **Status:** APPROVED (Step 4)
- **Context:** 2D map matchers suffer $0\%$ vertical layer accuracy on stacked bridges and flyovers because horizontal 2D coordinates overlap completely.
- **Decision:** Implement an explicit elevation barrier in the HMM emission probability model: when the barometric/INS altitude differs from candidate segment elevation by $>4.0\text{ m}$, the emission probability is penalized to $10^{-12}$.
- **Consequences:** Disambiguates flyovers, elevated corridors, and ground service roads with $>99.7\%$ accuracy.

---

### ADR-006: Chi-Square Innovation Gating ($\gamma_k \le 11.35$) for Multipath Outlier Rejection
- **Status:** APPROVED (Step 5)
- **Context:** Upon tunnel exit, degraded multipath GNSS fixes can cause sudden position jumps if integrated directly.
- **Decision:** Enforce normalized innovation gating $\gamma_k = \mathbf{r}_k^T \mathbf{S}_k^{-1} \mathbf{r}_k \le 11.35$ ($\alpha=0.01, m=3$) before accepting GNSS position updates into the ESKF.
- **Consequences:** Rejects corrupted tunnel-exit fixes while instantly accepting clean line-of-sight satellite measurements.

---

### ADR-007: Transparent Reporting of Real vs Simulated Results
- **Status:** APPROVED & ENFORCED (Validation Policy)
- **Context:** ISRO evaluators are satellite-navigation domain experts. Misrepresenting synthetic results as real data leads to immediate screening rejection.
- **Decision:** All metrics derived from simulated data are labeled `[SYNTHETIC SIMULATION]`, and real-data metrics are labeled `[AUTHENTIC IO-VNBD]`.
- **Consequences:** Complete scientific integrity and compliance with PS 26168 screening criteria.

---

### ADR-008: Multi-Mode Alignment Engine with Stationary Protection & Dual-Gated Surge Extraction
- **Status:** APPROVED & VERIFIED (Phase 2)
- **Context:** Still initial vehicle conditions lack horizontal acceleration variance, causing classical unconstrained PCA to latch onto isotropic sensor noise and misalign the forward axis.
- **Decision:** Implement multi-mode alignment with explicit states (`UNINITIALIZED`, `STATIONARY_LEVELED`, `DYNAMIC_ALIGNED`, `SLIP_DETECTED`, `RECALIBRATING`). Enforce dual surge criteria: eigenvalue ratio $\kappa = \lambda_{\text{long}}/\lambda_{\text{trans}} \ge 2.0$ and peak surge $a_{\text{surge}} \ge 0.35\text{ m/s}^2$. Maintain stationary leveling until dynamic motion is confirmed.
- **Consequences:** Completely eliminates stationary startup divergence (Known Failure KF-02 resolved) and ensures proper $\mathbf{R}_p^b \in \text{SO}(3)$ across all start conditions.

---

### ADR-009: Multi-Source Heading Supervisor with Tilt-Compensated Magnetometer Leveling and Multi-Epoch Re-Acquisition
- **Status:** APPROVED & VERIFIED (Phase 3)
- **Context:** Commodity MEMS gyroscopes drift significantly during GNSS outages. Direct magnetometer readings inside vehicle cabins are corrupted by ferromagnetic steel chassis and motor fields, while post-outage GNSS re-acquisition risks innovation gate lockouts.
- **Decision:** Implement `HeadingSupervisor` combining:
  1. GNSS Course-Over-Ground (COG) with dynamic velocity-dependent weighting ($\sigma_\psi = \text{clip}(\sigma_v/v_h, 1^\circ, 15^\circ)$) and strict $v_h \ge 2.5\text{ m/s}$ cutoff.
  2. 3D tilt-compensated magnetometer leveling in FLU body frame ($B_{fx} = B_x \cos\theta + B_y \sin\theta \sin\phi + B_z \sin\theta \cos\phi$, $B_{fy} = B_y \cos\phi - B_z \sin\phi \implies \psi_{\text{mag}} = \text{atan2}(B_{fy}, B_{fx})$) with multi-tier anomaly gating ($|\Delta B| > 12\ \mu\text{T}$, gradient $>35\ \mu\text{T/s}$).
  3. $\chi^2$ innovation gating ($\gamma \le 6.635$) with angular wrap protection ($[-\pi, +\pi]$) and 2-epoch consistency confirmation for post-outage re-acquisition.
- **Consequences:** Reduces authentic IO-VNBD outage drift on `Vta01b` from 51.2% to **5.68%** (9.0x improvement, beating <10% PS benchmark) with $83.9\ \mu\text{s}$ execution latency.
