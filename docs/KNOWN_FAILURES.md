# Known Failures & Engineering Risk Register
## ISRO PS 26168: AI-ML Based Intelligent Dead Reckoning System

---

## 1. Active Algorithmic & Physical Failure Modes

### KF-01: Unconstrained Gyroscope Yaw Drift during Pure Dead Reckoning [RESOLVED & VERIFIED in Phase 3]
- **Symptom:** During 60s and 120s GNSS blackouts on real IO-VNBD smartphone data, median dead reckoning position drift was **47.27%–68.81%** in baseline unconstrained mode.
- **Root Cause:** Commodity smartphone MEMS gyroscopes exhibit in-run bias instabilities ($0.01–0.05\text{ rad/s}$). In the absence of heading aiding, this integrates into $35^\circ–70^\circ$ of yaw heading error over 60s. Even with accurate longitudinal speed estimation ($\hat{v}_x$), rotating the velocity vector by $50^\circ$ projects forward displacement directly into cross-track positional error:
  $$\Delta x = \int v(t) \sin(\psi_{\text{err}}(t)) dt \implies \text{rapid cross-track drift}$$
- **Resolution (Implemented in Phase 3):**
  1. Implemented `HeadingSupervisor` in `idr_pipeline/heading_supervisor.py` combining dynamic velocity-weighted GNSS COG aiding and 3D tilt-compensated magnetometer leveling with multi-tier disturbance rejection ($|\Delta B| > 12\ \mu\text{T}$, gradient $>35\ \mu\text{T/s}$).
  2. Enforced $\chi^2$ innovation gating ($\gamma \le 6.635$) with 2-epoch consistency confirmation for post-outage re-acquisition.
  3. Measured authentic IO-VNBD outage performance on `Vta01b`: drift reduced from 51.2% (242.5m) to **5.68% (26.9m)** over 473.9m outage, exceeding the $<10\%$ PS benchmark.
- **Verification Evidence:** `tests/test_phase3_heading_observability.py` (24/24 tests passing). Road snapping in Phase 6 will further bound drift across complex intersections.

---

### KF-02: Mounting Alignment Failure during Initial Stillness [RESOLVED & VERIFIED in Phase 2]
- **Symptom:** On drive `Vtb01`, where the vehicle remained stationary ($v < 0.05\text{ m/s}$) for the first 20 seconds, the legacy longitudinal PCA surge algorithm produced an arbitrary forward axis from sensor noise, leading to massive downstream divergence ($>900\%$ drift).
- **Root Cause:** Principal Component Analysis (PCA) requires dynamic linear acceleration variance along the vehicle driving direction ($a_{\text{long}} > 0.4\text{ m/s}^2$). When the vehicle is stopped, horizontal accelerometer measurements contain only isotropic sensor noise, making the longitudinal axis unobservable.
- **Resolution (Implemented in Phase 2):**
  1. Implemented multi-mode alignment engine with explicit `AlignmentQuality` confidence metrics (`gravity_confidence`, `forward_axis_confidence`, `heading_confidence`).
  2. Enforced strict dual-threshold surge gating: forward alignment requires both horizontal eigenvalue separation $\kappa = \lambda_{\text{long}} / \lambda_{\text{trans}} \ge 2.0$ and peak surge $a_{\text{surge}} \ge 0.35\text{ m/s}^2$.
  3. When excitation is unobservable, the engine remains in `STATIONARY_LEVELED` mode, leveling roll and pitch while safely declaring `forward_axis_confidence = 0.0` and `alignment_valid = False` to prevent unassisted drift.
- **Verification Evidence:** `tests/test_phase2_alignment_initialization.py::test_06` and `test_07` strictly pass.

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

---

### KF-06: Standstill Engine Vibration GBDT Positive Bias & Synergy with ZUPT [RESOLVED & VERIFIED in Phase 4]
- **Symptom:** When the vehicle is stationary ($v_{\text{true}} < 0.5\text{ m/s}$), raw GBDT speed predictions output $+2.08$ to $+3.14\text{ m/s}$ ($7.5–11.3\text{ km/h}$).
- **Root Cause:** Stationary vehicle engines generate continuous mechanical vibrations ($0.1–0.5\text{ m/s}^2$ RMS). Because the GBDT uses vibration energy and SMA features, it maps non-zero stationary noise to the lowest non-zero leaves ($[0, 3.5]\text{ m/s}$).
- **Resolution:**
  1. This behavior proves why an isolated ML model cannot solve dead reckoning alone.
  2. The multi-sensor pipeline's Zero Velocity Update (ZUPT) module detects standstill ($a_{\text{var}} < 0.05\text{ m/s}^2$, $|\omega| < 0.03\text{ rad/s}$) and forcefully clamps $\mathbf{v}^n \to 0$ and $v_{\text{AI}} \to 0$.
  3. Validated in `tests/test_phase4_ai_velocity.py::test_12_dynamic_zupt_synergy`.
- **Verification Evidence:** Full pipeline stop-and-go tests pass with zero standstill velocity accumulation.

---

### KF-07: High-Speed (>60 km/h) Speed Underestimation Due to Leaf Clamping [CHARACTERIZED & BOUNDED in Phase 4]
- **Symptom:** At speeds $>60\text{ km/h}$ ($>16.67\text{ m/s}$), GBDT exhibits a negative bias of $-3.98\text{ m/s}$ (underestimating high speeds).
- **Root Cause:** Urban IO-VNBD dataset distribution is predominantly $0–50\text{ km/h}$ (city driving), and leaf values are clamped to $[0, 35]\text{ m/s}$ to prevent runaway explosion.
- **Resolution:**
  1. When GNSS is available, GNSS velocity vector updates calibrate forward scale.
  2. In Phase 5, learned adaptive measurement noise $R_v(t)$ increases uncertainty at high vibration/speeds so that ESKF IMU forward acceleration integration dominates over conservative GBDT predictions.
- **Verification Evidence:** Documented in `reports/phase4_speed_regimes_breakdown.csv` and `docs/PHASE_4_AI_VELOCITY_VALIDATION_REPORT.md`.
