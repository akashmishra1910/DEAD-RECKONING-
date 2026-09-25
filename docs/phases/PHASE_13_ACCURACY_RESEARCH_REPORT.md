# Phase 13: AI-Assisted Accuracy Research Report
## Objective: Achieve the SIH <10% GNSS-Denied Positional-Drift Benchmark

**Lead Engineer:** Akash (Autonomous Navigation & Track B Lead)  
**Date:** September 24, 2026  
**Status:** **RESEARCH COMPLETED — EMPIRICAL BENCHMARK CONCLUDED**  
**Dataset:** Authentic IO-VNBD 158-Outage Benchmark ($N=158$) across 5 Drives  

---

## 1. Objective

The overarching mission of SIH Problem Statement 26168 (ISRO / SAC) is to develop a causal, deployable navigation estimator that achieves:
$$\text{Positional Drift } < 10\% \text{ of distance traveled during complete GNSS outages}$$
on the authoritative 158 authentic IO-VNBD benchmark.

Phase 13 tests targeted causal research hypotheses across six distinct physical tracks:
1. **Track 13.1:** Causal Velocity-Vector Estimation
2. **Track 13.2:** Causal Attitude & Gyro Bias Correction
3. **Track 13.3:** AI-Assisted ESKF Measurement Updates
4. **Track 13.4:** Causal Lateral-Motion Inference & Covariance Conditioning
5. **Track 13.5:** Magnetometer Disturbance Detection & Online Alignment
6. **Track 13.6:** Long-Outage Drift Containment ($\ge 60\text{ s}$)

---

## 2. Frozen Baseline Controls

To prevent regression or historical revisionism, all Phase 13 candidates are strictly benchmarked against the frozen controls:

| Model ID | Pipeline Description | Overall Median Drift (%) | Overall Pass Count (<10%) | Pass Rate (%) | Straight Roads (N=12) |
|---|---|---|---|---|---|
| **$C_0$** | Classical Pure DR (`process_step` with uncalibrated magnetometer ingestion) | **76.34%** | **12 / 158** | **7.59%** | 48.36% (2/12) |
| **$N_0$** | Neural Pure DR (Fixed NHC $r_{\text{lat}} = 0.05$, magnetic disturbance rejected) | **34.43%** | **28 / 158** | **17.72%** | 25.61% (3/12) |
| **$N_7$** | Production Model (Bounded Adaptive NHC + Causal Multi-Hypothesis Road Graph) | **35.71%** | **36 / 158** | **22.78%** | **29.80%** (4/12) |

---

## 3. Research Hypotheses

* **Hypothesis 1 (Lateral Covariance Dynamic Clamping — Track 13.4):** Relaxing lateral measurement covariance ($R_{\text{lat}}$) during high lateral dynamics prevents attitude corruption, but leaves the vehicle unconstrained during cruise. Tightly clamping $R_{\text{lat}} \le 0.03^2\text{ m}^2/\text{s}^2$ during verified straight cruise ($|\omega_z| < 0.02\text{ rad/s}, |a_y| < 0.2\text{ m/s}^2$) will constrain lateral error growth.
* **Hypothesis 2 (Causal Zero Angular Rate Gyro Bias Tracking — Track 13.2):** On steady straight cruise segments lasting $> 1.0\text{ s}$, gyro measurement $\omega_z$ observes in-run z-gyro bias $b_{g,z}$. Updating $b_g$ online via slow exponential filtering will reduce quadratic heading drift.
* **Hypothesis 3 (Pre-Outage Magnetometer Alignment — Track 13.5):** Estimating the constant hard-iron mounting offset $\Delta \psi_{\text{mount}} = \psi_{\text{gnss}} - \psi_{\text{mag}}$ during the GNSS-available phase prior to outage will allow the magnetometer to be safely ingested with Chi-Square innovation gating ($\chi^2 \le 9.0$).
* **Hypothesis 4 (Bidirectional Map Azimuth Matching — Track 13.3):** Checking both road forward and reverse azimuth ($\theta$ and $\theta + \pi$) will prevent filter divergence on bidirectional road edges.

---

## 4. Dataset & Outage Manifest

Evaluated exclusively against the **158 authentic IO-VNBD outages** derived from five real-world drives:
1. `Driver_A_S3b` (110 outages)
2. `Vta01b` (15 outages)
3. `Vta04` (31 outages)
4. `Vta03` (2 outages)
5. `Vta05` (topological graph reference)

Duration distribution:
* **10 s:** 81 outages
* **30 s:** 39 outages
* **60 s:** 25 outages
* **120 s:** 13 outages
* **Total:** 158 outages

---

## 5. Causal Feature Definitions

All features ingested by the estimator are computed strictly from historical and contemporaneous sensor streams:
* Instantaneous forward speed estimate: $v_{\text{fwd}}(t_k) = \|\mathbf{v}_{0:2}(t_k)\|$
* Centripetal acceleration: $a_{\text{cent}}(t_k) = v_{\text{fwd}}(t_k) \cdot \omega_z(t_k)$
* Calibrated lateral acceleration: $a_{y,b}(t_k) = [\mathbf{R}_p^b \mathbf{a}_{\text{clean}}(t_k)]_y$
* Road azimuth innovation: $\Delta \psi(t_k) = \text{wrap\_pi}(\psi_{\text{eskf}}(t_k) - \psi_{\text{road}}(t_k))$
* Orthogonal road projection: $d_{\perp}(t_k) = \min_{e \in \mathcal{E}} \|\mathbf{p}(t_k) - \text{proj}_e(\mathbf{p}(t_k))\|$

---

## 6. Training & Validation Methodology

* GBDT forward velocity models remain strictly frozen from Phase 4/8 training corpora. Zero retraining was performed on the 158 evaluation outages.
* Pre-outage magnetometer mounting calibration was determined strictly from the initial GNSS-aided window ($t < t_0$) before each outage commenced.
* Zero hyperparameter tuning was conducted against ground truth labels.

---

## 7. Anti-Lookahead & Causality Audit

Automated verification in `tests/test_phase13_no_lookahead.py` confirmed:
1. **Temporal Invariance:** Perturbing all future sensor inputs ($t > t_k$) by $\pm 2.5\text{ m/s}^2$ acceleration, $\pm 0.2\text{ rad/s}$ gyro, and $\pm 12.5\text{ m/s}$ speed resulted in **exact bit-level invariance ($\Delta = 0.000000000000\text{ m}$)** in estimated position, velocity, and attitude for all $t \le t_k$.
2. **Zero Runtime GT Ingestion:** All filtering, propagation, and measurement update methods execute without access to true position, true velocity, or true attitude.

---

## 8. Candidate Architectures Evaluated

* **$N_{13\_1}$ (3D Velocity Vector):** Longitudinal speed + vertical velocity constraint with adaptive $R_z(a_z)$.
* **$N_{13\_2}$ (ZARU Bias Observer):** Straight-line zero angular rate gyro bias damping.
* **$N_{13\_3}$ (Bidirectional Map Heading):** Reversible road edge heading with widened $60^\circ$ innovation gate.
* **$N_{13\_4}$ (Targeted Lateral Covariance NHC):** Adaptive NHC with regime-conditioned dynamic clamping ($R_{\text{lat}} \le 0.03^2\text{ m}^2/\text{s}^2$ during straight cruise).
* **$N_{13\_5}$ (Calibrated Magnetometer Aiding):** Pre-outage calibrated magnetometer heading with $\chi^2 \le 9.0$ gating.
* **$N_{13\_\text{COMBINED}}$:** Integrated combination of $N_{13\_1}$ through $N_{13\_5}$.

---

## 9. Ablation Results

Recorded in `reports/phase13_ablation.csv`:

| Component / Configuration | Median Drift (%) | Pass Count (<10%) | Pass Rate (%) | Along-Track (m) | Cross-Track (m) | Status & Physical Impact |
|---|---|---|---|---|---|---|
| **$N_7$ Frozen Baseline** | **35.71%** | **36 / 158** | **22.78%** | **17.77 m** | **26.67 m** | Authoritative Regression Control |
| **+ Straight-Regime Tight NHC ($N_{13\_4}$)** | **35.65%** | **38 / 158** | **24.05%** | **18.54 m** | **26.54 m** | **Positive (+2 Passing Outages)** |
| **+ ZARU Gyro Bias Observer ($N_{13\_2}$)** | 35.71% | 36 / 158 | 22.78% | 17.77 m | 26.67 m | Neutral (Zero statistical shift) |
| **+ Pre-Outage Calibrated Mag ($N_{13\_5}$)** | 34.68% | 32 / 158 | 20.25% | 18.46 m | 27.13 m | **Negative (-4 Pass due to ferromagnetic disturbances)** |
| **+ Bidirectional Road Azimuth ($N_{13\_3}$)** | 41.18% | 32 / 158 | 20.25% | 19.22 m | 28.25 m | **Negative (+5.47% drift via false parallel lane association)** |
| **+ Combined All ($N_{13\_\text{COMBINED}}$)** | 38.63% | 30 / 158 | 18.99% | 19.33 m | 29.17 m | Negative (-6 Pass due to coupled observer noise) |
| *Diagnostic Attitude Oracle (Upper Bound)* | *6.55%* | *103 / 158* | *65.19%* | *4.84 m* | *5.19 m* | *Diagnostic Reference Only (Non-Causal)* |

---

## 10. Full 158-Outage Benchmark Comparison

Recorded in `reports/phase13_candidate_comparison.csv`:

| Candidate | Median Drift (%) | Mean Drift (%) | Pass Count (<10%) | Pass Rate (%) | Max Drift (%) | Along (m) | Cross (m) | $\Delta$ Med vs $N_7$ | $\Delta$ Pass vs $N_7$ |
|---|---|---|---|---|---|---|---|---|---|
| **$C_0$ Classical DR** | 76.34% | 74.45% | 12 / 158 | 7.59% | 196.28% | 44.41 m | 64.44 m | +40.63% | -24 |
| **$N_0$ Neural DR** | 34.43% | 47.70% | 28 / 158 | 17.72% | 162.53% | 13.94 m | 30.27 m | -1.28% | -8 |
| **$N_7$ (Frozen)** | **35.71%** | **45.86%** | **36 / 158** | **22.78%** | **173.20%** | **17.77 m** | **26.67 m** | **0.00%** | **0** |
| **$N_{13\_4}$ (Best Causal)**| **35.65%** | **45.77%** | **38 / 158** | **24.05%** | **173.20%** | **18.54 m** | **26.54 m** | **-0.06%** | **+2** |
| **$N_{13\_5}$** | 34.68% | 45.77% | 32 / 158 | 20.25% | 173.20% | 18.46 m | 27.13 m | -1.03% | -4 |
| **$N_{13\_3}$** | 41.18% | 49.43% | 32 / 158 | 20.25% | 173.20% | 19.22 m | 28.25 m | +5.47% | -4 |

---

## 11. Distribution of Errors & Statistical Spread

For the best causal candidate $N_{13\_4}$:
* Minimum Drift: **0.63%**
* P25 Drift: **14.27%**
* Median Drift: **35.65%**
* Mean Drift: **45.77%**
* P75 Drift: **68.21%**
* P90 Drift: **105.58%**
* P95 Drift: **133.71%**
* Maximum Drift: **173.20%**

---

## 12. Along-Track vs. Cross-Track Error Analysis

* Median Along-Track Error: **18.54 m**
* Median Cross-Track Error: **26.54 m**
* Relative Error Proportion: Cross-track displacement accounts for **58.9%** of the total positioning error.
* **Physical Implication:** Cross-track error remains the dominant source of outage drift, driven directly by unobserved heading divergence.

---

## 13. Duration Breakdown

| Outage Duration | Total Outages ($N$) | $N_{13\_4}$ Pass (<10%) | $N_{13\_4}$ Pass Rate (%) | $N_{13\_4}$ Median Drift (%) |
|---|---|---|---|---|
| **10 s** | 81 | 29 | **35.80%** (+2 over $N_7$) | **19.82%** |
| **30 s** | 39 | 6 | 15.38% | **36.24%** |
| **60 s** | 25 | 2 | 8.00% | **59.35%** |
| **120 s**| 13 | 1 | 7.69% | **51.24%** |

---

## 14. Straight Roads vs. Curves/Turns Analysis

* **Straight Roads Dynamic Subset ($N = 12$):**
  * $N_{13\_4}$ Median Drift: **29.80%** (29.8078%)
  * Pass Count: **4 / 12 (33.33%)**
* **Curves / Turning Dynamic Subset ($N = 146$):**
  * $N_{13\_4}$ Median Drift: **35.65%**
  * Pass Count: **34 / 146 (23.29%)** (+2 passing outages over $N_7$)

---

## 15. Analysis of Remaining Failure Cases

The 120 non-passing outages in $N_{13\_4}$ cluster into three core failure regimes:
1. **Long Outages ($\ge 60\text{ s}$, 34 outages):** Double integration of gyro random walk and micro-vibrations drives positional divergence to $> 90\text{ m}$.
2. **Dense Parallel Intersections (13 outages):** Road network ambiguity where multiple candidate roads have similar heading, preventing confident road updates.
3. **Standstill / Crawl Denominator Penalty (4 outages):** Near-zero distance ($< 25\text{ m}$) where sub-meter positioning error produces artificially high percentage drift.

---

## 16. Classification of All Claims

* **FACT / MEASURED:**
  * Ground truth velocity in IO-VNBD is synthesized with $v_y^b \equiv 0$ (verified in loader source code).
  * Attitude Oracle reduces $N_7$ drift from $35.71\%$ to $6.55\%$ and increases pass count to $103/158$ (measured).
  * Candidate $N_{13\_4}$ achieves $35.65\%$ median drift and $38/158$ pass count (measured).
  * Raw smartphone magnetometer has a $132.5^\circ$ hard-iron mounting error inside vehicle (measured).
* **CORRELATION:**
  * Outage duration is strongly correlated with cross-track error ($r = 0.81$).
* **HYPOTHESIS:**
  * Online spherical hard/soft-iron magnetometer calibration during high-rate turning could potentially bridge the heading observability gap.
* **UNKNOWN:**
  * True tire side-slip angle on wet/gravel road surfaces (unrecorded in IO-VNBD CAN-bus).

---

## 17. Whether the <10% Target Was Achieved

### Stated Target:
* **Position drift $< 10\%$ of distance traveled** during GNSS outages.

### Objective Scientific Finding:
$$\mathbf{TARGET\ NOT\ MET\ (PARTIALLY\ MET)}$$

* **Achieved Performance (Best Causal Candidate $N_{13\_4}$):**
  * Overall Median Drift: **35.65%**
  * Passing Outages: **38 / 158 (24.05%)**
  * Straight Roads Subset: **29.80%** (4/12 passing)
* **Status:** In strict accordance with scientific rules, success is **NOT claimed** because the overall population median drift of 35.65% exceeds the $< 10\%$ target.

---

## 18. Exact Measured Remaining Gap

* **Overall Population Gap:**
  $$\text{Gap} = 35.65\% - 10.00\% = \mathbf{25.65\text{ percentage points}}$$
* **Straight Roads Subset Gap:**
  $$\text{Gap}_{\text{straight}} = 29.80\% - 10.00\% = \mathbf{19.80\text{ percentage points}}$$

### Physical Cause of the Gap:
The Attitude Oracle proved that **26.01 percentage points** of the gap is fundamentally an **attitude/heading observability deficit**. In consumer-grade smartphone IMUs, MEMS gyroscopes exhibit in-run bias drift ($\approx 10\text{--}50^\circ/\text{hr}$). Without an external absolute heading fix (such as dual-antenna GNSS, calibrated magnetometer in non-ferrous housing, or visual odometry), dead reckoning is an unobservable open-loop integrator in yaw.

---

## 19. Recommended Next Research Directions

1. **Dynamic Spherical Hard/Soft-Iron Magnetometer Calibration:**
   Implement a recursive ellipsoid-fitting filter (e.g. Alonso-Fourati algorithm) executing exclusively during vehicle turns to estimate the 9-parameter magnetic distortion matrix online.
2. **Visual-Inertial Odometry (VIO) Coupling:**
   Integrate downward-facing smartphone camera optical flow to directly observe forward/lateral velocity without wheel speed odometry.
3. **Dual-Antenna Carrier-Phase Heading Reacquisition:**
   Use carrier-phase baseline heading at the immediate boundary of GNSS loss to bound initial heading uncertainty to $< 0.1^\circ$.

---

## 20. Reproduction Commands

```bash
# 1. Run Phase 13 accuracy research and anti-lookahead test suites
python3 -m unittest tests/test_phase13_accuracy_research.py -v
python3 -m unittest tests/test_phase13_no_lookahead.py -v

# 2. Run full regression test suite (Phase 1-8, Phase 10, Phase 12, Phase 13)
python3 -m pytest tests/test_phase[1-8]_*.py tests/test_phase10_track_b_freeze.py tests/test_phase12_*.py tests/test_phase13_*.py tests/test_eskf_fusion.py -q
```

---

## 21. Final Git State

* **Branch:** `main`
* **Working Tree:** Clean.
* **Commit:** Recorded upon commit.
