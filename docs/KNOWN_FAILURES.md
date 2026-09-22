# Known Failures & Engineering Risk Register
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

---

## 1. Active Algorithmic & Physical Failure Modes

### KF-01: Unconstrained Gyroscope Yaw Drift during Pure Dead Reckoning
- **Symptom:** During 60s and 120s GNSS blackouts on real IO-VNBD smartphone data, median dead reckoning position drift is **47.27%–68.81%**, failing the PS $<10\%$ benchmark.
- **Root Cause:** Commodity smartphone MEMS gyroscopes exhibit in-run bias instabilities ($0.01–0.05\text{ rad/s}$). In the absence of magnetometer heading updates or active road snapping, this integrates into $35^\circ–70^\circ$ of yaw heading error over 60s. Even with accurate longitudinal speed estimation ($\hat{v}_x$), rotating the velocity vector by $50^\circ$ projects forward displacement directly into cross-track positional error:
  $$\Delta x = \int v(t) \sin(\psi_{\text{err}}(t)) dt \implies \text{rapid cross-track drift}$$
- **Impact:** High. Prevents pure DR from passing sponsor thresholds without aiding.
- **Mitigation Plan:**
  1. Implement **Phase 3 (Attitude/Heading Observability)**: Integrate magnetic heading updates with magnetic disturbance rejection ($\|\mathbf{B}_{\text{meas}}\| - \|\mathbf{B}_{\text{ref}}\| \le \tau_{\text{mag}}$) and GNSS Course-Over-Ground (COG) initialization.
  2. Implement **Phase 6 (3D Offline Map-Matching)**: Use high-confidence road tangent constraints to continuously bound heading drift during straight/curved road segments.

---

### KF-02: Mounting Alignment Failure during Initial Stillness
- **Symptom:** On drive `Vtb01`, where the vehicle remained stationary ($v < 0.05\text{ m/s}$) for the first 20 seconds, the longitudinal PCA surge algorithm produced an arbitrary forward axis, leading to massive downstream divergence ($>900\%$ drift).
- **Root Cause:** Principal Component Analysis (PCA) requires dynamic linear acceleration variance along the vehicle driving direction ($a_{\text{long}} > 0.4\text{ m/s}^2$). When the vehicle is stopped, horizontal accelerometer measurements contain only sensor noise, making the longitudinal axis unobservable.
- **Impact:** High for stop-start drive initializations.
- **Mitigation Plan:**
  1. Implement **Phase 2 (Alignment & Initialization Hardening)**: Multi-mode alignment engine with explicit `AlignmentQuality` confidence metrics (`gravity_confidence`, `forward_axis_confidence`, `heading_confidence`).
  2. The system must defer full mounting alignment until an actual acceleration surge or GNSS velocity vector is observed, rather than assuming unverified axes.

---

### KF-03: Absence of Authentic Road Graph for Loughborough / UK Driving Routes
- **Symptom:** In Step 4C, Configuration F (3D Topological Map-Matching) could not be evaluated on real IO-VNBD drives and was marked `NOT AVAILABLE`.
- **Root Cause:** The 3D map matcher was tested against a synthetic multi-level corridor in `test_suite.py`, but an authentic OpenStreetMap (OSM) extract covering Loughborough/Leicestershire (the geographical location of IO-VNBD drives) was not yet integrated into the repository.
- **Impact:** Medium. Blocks Configuration F benchmark on real IO-VNBD data.
- **Mitigation Plan:** Extract and cache the offline topological OSM road network for IO-VNBD routes and Indian benchmark corridors in Phase 6.

---

### KF-04: Metric Discrepancy between Proposal Claims and Real IO-VNBD Evaluation
- **Symptom:** Earlier project proposals reported GBDT speed RMSE of $\sim 1.2\text{ m/s}$ and drift of $\sim 2.01\%$, whereas Step 4C measured GBDT test RMSE of $5.001\text{ m/s}$ and median drift of $53.79\%$ on real IO-VNBD logs.
- **Root Cause:** The earlier $1.2\text{ m/s}$ and $2.01\%$ numbers were evaluated on synthetic simulator data (`generate_realistic_iovnbd_trajectory`), whereas $5.001\text{ m/s}$ and $53.79\%$ reflect the true, unassisted performance on raw smartphone IMUs subjected to real engine harmonics, potholes, and suspension tilt.
- **Impact:** Critical for evaluator honesty.
- **Resolution:** As mandated by the Validation Policy, all synthetic results are explicitly labeled `[SYNTHETIC SIMULATION]`, and all real measurements are tracked transparently as `[AUTHENTIC IO-VNBD]`.

---

### KF-05: Upstream GitHub Git LFS Quota / Missing Blobs
- **Symptom:** `git clone` or `git lfs pull` directly from GitHub failed with HTTP `404` or `403 Forbidden`.
- **Root Cause:** The upstream repository author's GitHub LFS storage/bandwidth quota was exhausted.
- **Resolution:** Bypassed LFS smudge during checkout (`GIT_LFS_SKIP_SMUDGE=1`), verified that clean extracted naturalistic drives exist in `real_iovnbd_data/`, and configured local Git filters so work proceeds without blocking.
