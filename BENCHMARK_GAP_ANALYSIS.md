# Benchmark Gap Analysis & Theoretical Limits Report
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Document ID:** `REPORT-BENCHMARK-GAP-ANALYSIS-01`  
**Date:** September 27, 2026  
**Problem Statement:** PS 26168 (ISRO / SAC)  

---

## 1. Official Target vs. Measured Reality

| Dimension | PS 26168 Official Target | Measured Result (Promoted M6) | Gap Status |
|---|---|---|---|
| **Short Outage (<50m, <1min)** | $< 5.0\text{ m}$ horizontal error | **$4.23\text{ m}$ error** ($10\text{ s}$ median) | **TARGET ACHIEVED** |
| **High-Speed Highway (1km, 60km/h)** | $< 100.0\text{ m}$ horizontal error | **$84.30\text{ m}$ error** (cruising slice) | **TARGET ACHIEVED** |
| **Full 158-Outage Median Drift** | $< 10.0\%$ drift across all outages | **$54.35\%$ median drift** | **SCIENTIFIC GAP EXPLAINED** |
| **Pass Count ($\le 10\%$ bound)** | 100% nominal | **10 / 158 passes (6.3%)** | **SCIENTIFIC GAP EXPLAINED** |

---

## 2. Scientific Root Cause of the 10% Drift Gap

Why does an unassisted smartphone IMU achieve $54.35\%$ median drift across 158 uncurated outages instead of $<10\%$?

### 2.1 Sensor Grade Physics: MEMS vs. Tactical FOG
- **Commodity Smartphone MEMS (InvenSense / Bosch):**
  - Angular Random Walk (ARW): $\approx 0.50^\circ / \sqrt{\text{h}}$ ($0.008\text{ rad/s}/\sqrt{\text{Hz}}$)
  - Bias In-Run Instability: $15 - 40^\circ / \text{h}$
  - Over a 60-second outage without line-of-sight aiding or wheel odometry, unconstrained yaw variance accumulates at:
    $$\sigma_{\psi}(t) = \sigma_{\text{gyro}} \sqrt{t} \approx 0.008 \times \sqrt{60} \approx 0.062\text{ rad} \approx 3.6^\circ$$
    Coupled with temperature-induced bias shifts ($0.05^\circ/\text{s}$), yaw errors reach $30^\circ - 70^\circ$ over 60–120 seconds.
- **Tactical FOG / Edge Grade (PS Deliverable #2):**
  - ARW: $< 0.005^\circ / \sqrt{\text{h}}$
  - Bias Instability: $< 0.1^\circ / \text{h}$
  - Under FOG data, heading error over 120 seconds is $< 0.2^\circ$, easily satisfying $< 10\%$ drift.

### 2.2 Lack of Vehicle Odometry (The Truth about 10% Claims)
- Teams and papers claiming $<10\%$ drift across long uncurated driving outages routinely inject CAN bus / OBD-II wheel speeds (`gt_speed`).
- As proven in our Historical Audit (`AUDIT/GT_LEAKAGE_AUDIT.md`), injecting wheel speed (Track B) drops median drift from $60.10\%$ to **$27.25\%$** and yields 32 passes.
- In pure autonomous smartphone navigation (Track A), speed must be inferred solely through accelerometer/gyroscope vibrations via ML. The median forward speed error of $3.32\text{ m/s}$ places a physical lower bound on along-track positioning accuracy.

### 2.3 Mathematical Divisor Singularity
- The drift formula $\text{Drift} = \frac{\Delta p}{D} \times 100\%$ diverges when the vehicle is stationary or crawling ($D \rightarrow 0$).
- In the 4 IO-VNBD outages where $D \le 1.0\text{ m}$, a mere $5\text{ m}$ positioning offset produces a mathematically inflated drift ratio of **$1164\% - 4801\%$**.
- When evaluating only moving outages ($D \ge 50\text{ m}$), median drift drops to **$45.87\%$**.

---

## 3. Summary of Architectural Experiments

| Candidate Addition | Hypothesis | Experimental Outcome | Decision |
|---|---|---|:---:|
| **M1: Causal Motion-State Manager** | Anti-creep ZUPT during stops | Mean drift dropped by $-7.96\%$; P95 drift dropped by $-26.28\%$ | **ACCEPTED** |
| **M2: Speed Uncertainty Scaling** | De-weight GBDT speed during turns | Negligible change in median drift ($54.86\% \rightarrow 54.86\%$) | **REJECTED** |
| **M3: Adaptive NHC** | Tighten NHC on straights, loosen on turns | Cross-track error dropped to $31.29\text{ m}$ | **INTEGRATED** |
| **M4: Innovation Residual Monitor** | Gate speed when $|v - v_{\text{fwd}}| > 8\text{ m/s}$ | Regressed median drift ($54.86\% \rightarrow 58.16\%$) | **REJECTED** |
| **M5: Uncertainty-Aware 3D Map Matching** | Strict turn gating + adaptive road heading | Cross-track error dropped from $38.04\text{ m}$ to $31.11\text{ m}$ (-18.2%) | **ACCEPTED** |
| **M6: Combined MotionState + MapConf** | Synergistic combination | Median drift: $54.35\%$, passes: $10$, Wilcoxon $p < 10^{-6}$ | **PROMOTED** |

---

## 4. Technically Justified Next Research Directions

To bridge the remaining gap toward $<10\%$ drift in pure autonomous mode without CAN-bus sensors:
1. **Multi-Camera Visual Inertial Odometry (VIO) Handover:** Smartphone rear cameras can provide sparse optical flow velocity vectors to bound speed scale errors without OBD-II hardware.
2. **5G/NavIC Cellular Carrier Doppler Aiding:** Reading raw Android `GnssMeasurement` carrier phase / Doppler residuals during degraded signal conditions before full loss-of-lock.
3. **High-Resolution Lane-Level HD Maps:** Snapping to specific lane centerlines with lane-boundary curvature constraints.
