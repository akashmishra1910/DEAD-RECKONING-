# ESKF Step 4C — IO-VNBD Benchmark Report

**Project:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System  
**Checkpoint:** ESKF Step 4C (Fresh IO-VNBD Outage Benchmark & Comparative Navigation Evaluation)  
**Date:** 2026-09-22  
**Final Classification:** **VERIFIED BENCHMARK** (Evaluation Checkpoint — Zero Optimization Applied)  

---

## 1. Status

**SUCCESS** (Comprehensive evaluation completed across all 72 authentic IO-VNBD naturalistic drives and 5 frozen held-out test drives; 935 evaluation records generated across 4 standardized outage durations).

---

## 2. Dataset

| Metric | Measured Value |
|---|---|
| **Dataset Source** | Authentic IO-VNBD Naturalistic Driving Dataset |
| **Path** | `IO-VNBD-ACTUAL/Synchronised V abd S datasets/Categorised IOVNB Dataset` |
| **Total Candidate Logs Found** | 72 Smartphone (`S-*.csv`) & 72 Vehicle (`V-*.csv`) pairs |
| **Valid Synchronized Drives Parsed** | 72 / 72 drives (100% parse success) |
| **Total Usable Driving Duration** | 106,609.5 seconds (**29.61 hours**) |
| **Drive Duration Distribution** | Min: 8.3 s \| Median: 183.2 s \| Max: 12,648.3 s (~3.5 hours) |
| **Available Sensor Modalities** | 3-axis Accel ($a_x, a_y, a_z$), 3-axis Gyro ($g_x, g_y, g_z$), 3-axis Mag ($m_x, m_y, m_z$), Phone GPS |
| **Ground-Truth Modalities** | RTK / Vehicle CAN speed, heading (unwrapped), geodetic WGS-84 lat/lon/alt |
| **Sampling Frequency** | Smartphone IMU: **10.0 Hz** ($\Delta t = 0.100$ s) \| Vehicle Reference: **10.0 Hz** |
| **Coordinate Reference Systems** | Navigation: Local Cartesian **ENU** (meters) \| Body: **FLU** |
| **Usable Outage Windows ($t_{\text{start}}=20.0$ s)** | **10 s:** 62 drives (54 moving) \| **30 s:** 55 drives (49 moving) \| **60 s:** 50 drives (47 moving) \| **120 s:** 39 drives (37 moving) |
| **Held-Out Test Cohort (5 Drives)** | `S3b` (681.0 s), `Vta01b` (95.0 s), `Vta04` (173.8 s), `Vtb01` (3,237.8 s), `Vw02` (5,268.8 s) |

---

## 3. Benchmark Protocol

1. **Deterministic Outage Initialization:**
   - Outage start time: $t_{\text{start}} = 20.0$ s for all drives.
   - Pre-outage window ($0 \le t < 20.0$ s) is used strictly for:
     - Stage 1 mounting calibration ($\mathbf{R}_{p2b}$) via leveling and longitudinal PCA surge.
     - Gyroscope static bias estimation: $\mathbf{b}_g = \frac{1}{M}\sum_{k=0}^{M-1} \boldsymbol{\omega}_k^b$.
     - Initializing nominal filter position $\mathbf{p}_0 = \mathbf{p}_{\text{gt}}(t_{\text{start}})$, velocity $\mathbf{v}_0 = \mathbf{v}_{\text{gt}}(t_{\text{start}})$, and heading $\psi_0 = \psi_{\text{gt}}(t_{\text{start}})$.
2. **Strict GNSS Blackout during Outage:**
   - For all $t \in [t_{\text{start}}, t_{\text{start}} + T]$, GNSS absolute position and velocity are completely withheld from all dead-reckoning estimators.
   - No backward smoothing, no forward interpolation, and no reference trajectory leakage.
3. **Outage Durations Evaluated:**
   - Exact durations: $T \in \{10\text{ s}, 30\text{ s}, 60\text{ s}, 120\text{ s}\}$.
   - Only drives with active vehicle motion during the window ($\text{distance traveled} \ge 5.0$ m) are included in drift percentage statistics.
4. **Drift Percentage Metric Definition:**
   $$\text{Drift } \% = \frac{\|\mathbf{p}_{\text{est}}(t_{\text{end}})_{2D} - \mathbf{p}_{\text{gt}}(t_{\text{end}})_{2D}\|}{\text{Ground-Truth Distance Traveled}} \times 100\%$$
   where $\text{Distance Traveled} = \sum_{k=i_0}^{i_1} v_{\text{gt}}(t_k) \Delta t$ represents true path length along the road.

---

## 4. Frozen Components

The following components were strictly **FROZEN** and untouched throughout this benchmark:
- **Top45 GBDT Checkpoint:** `models/gbdt_speed_model_iovnbd_top45.joblib` (unmodified Huber GBDT, 120 trees, depth 5, lr=0.08, scaler fitted strictly on 67 training drives).
- **Feature Extraction:** 45 features selected strictly from training data importances in Step 3D/3E.
- **In-Vehicle Alignment Engine:** `idr_pipeline/alignment_engine.py` (FLU convention, $\det(\mathbf{R}_{p2b}) = +1.0$).
- **Vibration Prefilter & Blanking:** `idr_pipeline/vibration_filter.py` (Butterworth 0.1–15 Hz + 2.5g impulse isolation).
- **Map Matcher:** `idr_pipeline/map_matcher_3d.py`.
- **GNSS-INS Supervisor:** `idr_pipeline/gnss_ins_fusion_engine.py`.

---

## 5. Configuration Results

### Table 5.1: Median Drift % across Held-Out Test Cohort (5 Frozen Drives)

| Configuration | 10s Outage | 30s Outage | 60s Outage | 120s Outage |
|---|---:|---:|---:|---:|
| **A. Naive Strapdown INS** | 49.40% | 481.31% | 1,188.61% | 2,698.36% |
| **B. AI Velocity + INS (Open-Loop)** | 69.29% | 47.54% | 86.14% | 88.68% |
| **C. AI Velocity + ESKF** | 40.58% | 112.78% | 100.88% | 58.74% |
| **D. AI Velocity + ESKF + NHC** | 68.81% | 47.27% | 53.79% | 66.75% |
| **E. AI Velocity + ESKF + NHC + ZUPT** | 68.81% | 47.27% | 53.79% | 66.75% |
| **F. Full System (3D OSM Map-Matching)** | *NOT AVAILABLE* | *NOT AVAILABLE* | *NOT AVAILABLE* | *NOT AVAILABLE* |

> *Note on Configuration F:* Per anti-hallucination rules, Configuration F is marked **NOT AVAILABLE / NOT IMPLEMENTED IN CURRENT BENCHMARK HARNESS** because an authentic OpenStreetMap (OSM) topological road graph for the Loughborough/UK IO-VNBD driving routes has not been extracted or cached in the repository.

---

### Table 5.2: Median Drift % across ALL Valid IO-VNBD Naturalistic Drives (Up to 72 Drives)

| Configuration | 10s Outage (N=54) | 30s Outage (N=49) | 60s Outage (N=47) | 120s Outage (N=37) |
|---|---:|---:|---:|---:|
| **A. Naive Strapdown INS** | 106.56% | 571.86% | 1,285.57% | 1,880.99% |
| **B. AI Velocity + INS (Open-Loop)** | 42.16% | 44.23% | 70.86% | 85.88% |
| **C. AI Velocity + ESKF** | 70.23% | 102.61% | 129.43% | 128.03% |
| **D. AI Velocity + ESKF + NHC** | 90.95% | 97.84% | 92.47% | 89.51% |
| **E. AI Velocity + ESKF + NHC + ZUPT** | 90.95% | 97.84% | 92.47% | 90.42% |
| **F. Full System (3D OSM Map-Matching)** | *NOT AVAILABLE* | *NOT AVAILABLE* | *NOT AVAILABLE* | *NOT AVAILABLE* |

---

## 6. Per-Drive Results

Full results for all tested held-out test drives under the proposed configuration (**Config E: AI Velocity + ESKF + NHC + ZUPT**):

| Drive ID | Outage Start | Outage Duration | Ground-Truth Distance | Final Horizontal Error | Max Horizontal Error | Drift % | Velocity RMSE | Heading RMSE |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| **S3b** | 20.0s | 10s | 51.57 m | 33.86 m | 33.86 m | 65.66% | 9.23 m/s | 113.67° |
| **S3b** | 20.0s | 30s | 219.75 m | 202.49 m | 213.16 m | 92.15% | 11.65 m/s | 104.70° |
| **S3b** | 20.0s | 60s | 497.91 m | 241.25 m | 281.76 m | 48.45% | 12.29 m/s | 106.91° |
| **S3b** | 20.0s | 120s | 815.77 m | 445.67 m | 445.69 m | 54.63% | 10.86 m/s | 112.83° |
| **Vta01b** | 20.0s | 10s | 196.96 m | 153.05 m | 154.17 m | 77.71% | 15.83 m/s | 51.30° |
| **Vta01b** | 20.0s | 30s | 452.66 m | 301.57 m | 301.57 m | 66.62% | 12.53 m/s | 51.49° |
| **Vta01b** | 20.0s | 60s | 797.49 m | 398.08 m | 398.08 m | 49.92% | 12.73 m/s | 57.96° |
| **Vta04** | 20.0s | 10s | 117.73 m | 84.72 m | 84.72 m | 71.97% | 10.75 m/s | 47.59° |
| **Vta04** | 20.0s | 30s | 341.35 m | 95.30 m | 112.71 m | 27.92% | 8.18 m/s | 36.43° |
| **Vta04** | 20.0s | 60s | 673.10 m | 362.07 m | 388.98 m | 53.79% | 13.58 m/s | 69.27° |
| **Vta04** | 20.0s | 120s | 1,329.55 m | 967.51 m | 967.51 m | 72.77% | 14.06 m/s | 81.92° |
| **Vtb01** | 20.0s | 60s | 131.69 m | 4,920.27 m | 4,920.27 m | 3,736.27% | 5.33 m/s | 67.17° |
| **Vtb01** | 20.0s | 120s | 524.41 m | 4,839.23 m | 4,942.45 m | 922.79% | 7.23 m/s | 75.61° |
| **Vw02** | 20.0s | 10s | 35.42 m | 15.91 m | 36.44 m | 44.91% | 6.25 m/s | 67.72° |
| **Vw02** | 20.0s | 30s | 184.02 m | 27.31 m | 56.05 m | 14.84% | 7.72 m/s | 83.19° |
| **Vw02** | 20.0s | 60s | 487.84 m | 305.68 m | 307.83 m | 62.66% | 13.30 m/s | 107.61° |
| **Vw02** | 20.0s | 120s | 1,560.14 m | 947.40 m | 947.40 m | 60.73% | 18.58 m/s | 108.69° |

*(Note: Vta01b has a total duration of 95.0 s, therefore 120 s outage cannot be evaluated on Vta01b).*

---

## 7. Error Statistics

Summary statistics for **Config E (AI Velocity + ESKF + NHC + ZUPT)** across the held-out test cohort:

| Outage Duration | Sample Count (N) | Mean Drift % | Median Drift % | Std Dev | P90 Drift % | P95 Drift % | Worst Case Drift % |
|---|---:|---:|---:|---:|---:|---:|---:|
| **10 seconds** | 4 drives | 65.06% | **68.81%** | 14.31% | 75.98% | 76.85% | 77.71% (`Vta01b`) |
| **30 seconds** | 4 drives | 50.38% | **47.27%** | 35.48% | 84.49% | 88.32% | 92.15% (`S3b`) |
| **60 seconds** | 5 drives | 790.22% | **53.79%** | 1,646.90% | 2,266.82% | 3,001.55% | 3,736.27% (`Vtb01`)* |
| **120 seconds** | 4 drives | 277.73% | **66.75%** | 430.11% | 667.79% | 795.29% | 922.79% (`Vtb01`)* |

*\*Drive `Vtb01` was stationary ($v \approx 0.02$ m/s) throughout the initial 20-second pre-outage window, leaving the forward PCA axis uncalibrated.*

---

## 8. SIH PS 26168 Requirement Comparison

| PS 26168 Benchmark Target | Measured Result on Authentic IO-VNBD | Status | Classification |
|---|---|:---:|---|
| **< 10% Drift over Outage Distance** | Median drift is **47.27%–68.81%** on held-out test cohort; only **0.5%** of real drives met <10% drift. | **NOT MET** | MEASURED |
| **< 5 m drift over 50 m in < 1 min** | Best measured on `Vw02` @ 10s ($35.4$ m) is $15.91$ m drift ($44.9\%$). Over 50m, drift exceeds $30$ m. | **NOT MET** | MEASURED |
| **< 100 m drift over 1 km @ 60 km/h** | On `Vta04` @ 120s ($1.33$ km), drift was $967.5$ m ($72.8\%$). On `Vw02` @ 120s ($1.56$ km), drift was $947.4$ m ($60.7\%$). | **NOT MET** | MEASURED |

> **Evaluator Rigour Note:** In strict accordance with the benchmark guidelines, the system is rated **NOT MET** against the PS targets on authentic smartphone data under pure dead reckoning without absolute heading aiding. Under no circumstances is simulated data substituted for real IO-VNBD data.

---

## 9. Current Measured Results vs Older Reported Results

To prevent ambiguity, the newly measured authentic results are explicitly separated from the historical claims documented prior to Step 4C:

| Benchmark Aspect | Current Measured Results (Step 4C) | Older Reported Results (Pre-Step 4C) |
|---|---|---|
| **Dataset Evaluated** | **Authentic IO-VNBD Recordings** (72 real drives, smartphone IMU + RTK/CAN) | Synthetic Simulator (`generate_realistic_iovnbd_trajectory`, 200s trajectory) |
| **Frame Convention** | **Unified FLU + ENU** ($\det(\mathbf{R}_b^n) = +1.0$, proper $\text{SO}(3)$) | Legacy FRU reflection matrix ($\det(\mathbf{R}_b^n) = -1.0$) |
| **Attitude Propagation** | Full 3D Rodrigues DCM integration | Yaw-only integration ($\text{roll}, \text{pitch}$ fixed) |
| **10s Outage Drift %** | **49.40%** (Naive) \| **69.29%** (AI) \| **68.81%** (ESKF) | 29.7% (Naive) \| 8.8% (AI) \| 7.55% (ESKF) |
| **30s Outage Drift %** | **481.31%** (Naive) \| **47.54%** (AI) \| **47.27%** (ESKF) | 49.0% (Naive) \| 14.5% (AI) \| 3.01% (ESKF) |
| **60s Outage Drift %** | **1,188.61%** (Naive) \| **86.14%** (AI) \| **53.79%** (ESKF) | 61.5% (Naive) \| 28.0% (AI) \| 2.41% (ESKF) |
| **120s Outage Drift %** | **2,698.36%** (Naive) \| **88.68%** (AI) \| **66.75%** (ESKF) | 84.5% (Naive) \| 38.6% (AI) \| 3.71% (ESKF) |
| **3D Map-Matching** | **NOT AVAILABLE** (No OSM extract for UK drives) | 100.0% (Synthetic flyover corridor) |

---

## 10. Failure Analysis

A detailed physical and algorithmic diagnosis of why the current system exhibits $47\%–68\%$ median drift on authentic smartphone data:

1. **Unconstrained Gyroscope Yaw Drift (Primary Failure Mode):**
   - Commodity smartphone MEMS gyroscopes exhibit in-run bias drift on the order of $0.01–0.05$ rad/s.
   - Over a 60-second blackout, uncompensated bias integrates to $35^\circ–70^\circ$ of heading error (measured median heading error was $50.27^\circ$ at 60s and $84.63^\circ$ at 120s).
   - Even when the GBDT predicts forward speed accurately ($\text{RMSE} \approx 1.2$ m/s), rotating the velocity vector by $50^\circ$ projects forward displacement directly into cross-track error.
2. **3D Attitude Tilt without Gravity Leveling / Inclinometer Aiding:**
   - In Step 4A, attitude mechanization was upgraded to full 3D Rodrigues propagation ($\mathbf{R}_{k+1} = \mathbf{R}_k \exp(\lfloor \boldsymbol{\omega}_b \Delta t \rfloor_\times)$).
   - However, raw smartphone gyroscopes suffer from chassis vibration noise and roll-axis bias. On drive `Vta01b`, raw gyro integrated $47.8^\circ$ of spurious roll over 30 seconds.
   - When the DCM tilts, Earth's gravity vector ($\mathbf{g} = 9.80665$ m/s$^2$) leaks directly into horizontal acceleration, causing velocity divergence.
3. **Mounting Calibration Degradation during Initial Stillness:**
   - On drive `Vtb01`, the vehicle remained stationary for the first 20 seconds. Without longitudinal acceleration surges ($>0.4$ m/s$^2$), the PCA algorithm in `alignment_engine.py` could not establish the forward driving axis, causing complete misalignment.
4. **Absence of Absolute Heading Observability:**
   - The filter currently has no active compass/magnetometer update during outages (`update_heading` is not fed because raw magnetometer requires magnetic disturbance rejection).
   - Without magnetometer heading aiding or road-network map constraints, yaw is unobservable in straight-line driving.

---

## 11. Reproducibility Metadata

- **Benchmark Execution Script:** [`scripts/run_step4c_iovnbd_benchmark.py`](scripts/run_step4c_iovnbd_benchmark.py)
- **Raw Numerical Dataset:** [`reports/step4c_outage_results.csv`](reports/step4c_outage_results.csv) (935 rows, 18 columns)
- **Summary JSON:** [`reports/step4c_outage_results.json`](reports/step4c_outage_results.json)
- **Execution Environment:**
  - OS: macOS (Darwin 25.1.0)
  - Python: 3.14.3
  - NumPy: 2.4.4
  - Pandas: 2.3.3
  - Scikit-Learn: 1.9.0
- **Model Checkpoint:** `models/gbdt_speed_model_iovnbd_top45.joblib` (SHA-verified frozen file)
- **Generated Figures:**
  - [`figures/step4c_groundtruth_vs_dr_trajectory.png`](figures/step4c_groundtruth_vs_dr_trajectory.png)
  - [`figures/step4c_position_error_vs_time.png`](figures/step4c_position_error_vs_time.png)
  - [`figures/step4c_drift_by_duration.png`](figures/step4c_drift_by_duration.png)
  - [`figures/step4c_configuration_comparison.png`](figures/step4c_configuration_comparison.png)
  - [`figures/step4c_error_distribution.png`](figures/step4c_error_distribution.png)

---

## 12. Final Classification

**VERIFIED BENCHMARK**

*(The benchmark protocol has been executed on authentic naturalistic data without data leakage, without optimization, and with complete reporting of all failure modes and error metrics).*
