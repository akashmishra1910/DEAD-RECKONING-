# M0–M7 Controlled Architectural Traceability & Scientific Audit Report
## SIH 2026 / ISRO-SAC Problem Statement PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Benchmark Dataset:** Authentic IO-VNBD 158-Outage Benchmark (5 Drives, 158 Natural & Injected Outages)  
**Execution Date:** September 27, 2026  
**Evaluation Protocol:** Regime B — Pure Autonomous Dead Reckoning (Strictly Zero Ground-Truth Speed in Outage)  
**Speed Estimator:** Pre-trained Top-45 Feature GBDT (`gbdt_speed_model_iovnbd_top45.joblib`, SHA256: `dc05b8de795732233082c3c687553d983d358bd4581514c5ffc4bdd9d58ad242`)  
**Runner:** `scripts/run_controlled_architectural_ablations.py`  
**Configuration Contract:** `CONFIG/EXPERIMENT_CONTRACT_M0_M7.yaml`  
**Historical Ledger:** `reports/historical_ablation_results_m0_m7.csv`  
**Authoritative Ledger:** `BENCHMARK_ABLATION_RESULTS.csv`  

---

## 1. Executive Summary & Purpose

The purpose of the M0–M7 controlled ablation suite is to isolate, characterize, and trace every algorithmic component built on top of the golden autonomous baseline (**B1/N8**). In previous historical research rounds, certain experimental configurations lacked strict parameter isolation—specifically, configuration **M5** (Uncertainty-Aware Map Matching) had inadvertently inherited `motion_state: True` in historical runner scripts, causing its performance to mirror M6 identically.

This report documents:
1. The exact configuration matrix and parameter switches for experiments **M0** through **M7**.
2. The rigorous resolution and verification of the M5 vs. M6 isolation.
3. The empirical findings across all 158 authentic outages under strict causal zero-GT conditions.
4. Scientific trade-offs regarding heading observability, cross-track drift, along-track drift, and pass rates.

---

## 2. Formal Experiment Configuration Matrix

| Experiment ID | Experiment Name | Motion-State Manager | Speed Confidence Scaling | Adaptive NHC Covariance | Innovation Residual Monitor | Map Confidence Heading Gating |
|---|---|:---:|:---:|:---:|:---:|:---:|
| **M0** | B1/N8 Golden Baseline | ❌ | ❌ | ❌ | ❌ | ❌ |
| **M1** | B1/N8 + Motion-State Manager | ✅ | ❌ | ❌ | ❌ | ❌ |
| **M2** | B1/N8 + Speed Uncertainty | ❌ | ✅ | ❌ | ❌ | ❌ |
| **M3** | B1/N8 + Adaptive NHC | ✅ | ❌ | ✅ | ❌ | ❌ |
| **M4** | B1/N8 + Innovation Monitor | ❌ | ❌ | ❌ | ✅ | ❌ |
| **M5** | B1/N8 + Uncertainty-Aware Map (Isolated) | ❌ | ❌ | ❌ | ❌ | ✅ |
| **M6** | B1/N8 + MotionState + MapConf | ✅ | ❌ | ❌ | ❌ | ✅ |
| **M7** | B1/N8 + Full Integrated Policy | ✅ | ✅ | ✅ | ✅ | ✅ |

---

## 3. Resolution of Historical M5 vs. M6 Traceability Conflation

### 3.1 The Anomaly in Historical Data
In `reports/historical_ablation_results_m0_m7.csv`, the recorded metrics were:
- **Historical M5:** Median Drift = `54.35%`, Mean Drift = `115.12%`, Pass Count ($\le 10\%$) = `10`
- **Historical M6:** Median Drift = `54.35%`, Mean Drift = `115.12%`, Pass Count ($\le 10\%$) = `10`

Code audit of `scripts/run_controlled_architectural_ablations.py` revealed that M5 had been defined as:
```python
# Historical code defect in M5 definition:
"M5": {
    "name": "B1/N8 + Uncertainty-Aware Map",
    "motion_state": True,     # <-- DEFECT: Should have been False to isolate map confidence
    "speed_confidence": False,
    "adaptive_nhc": False,
    "innovation_monitor": False,
    "map_confidence": True,
}
```
Because `motion_state: True` was enabled, M5 was executing both the motion-state standstill ZUPT/anti-creep AND the map-confidence gating, making M5 an unintentional clone of M6.

### 3.2 The Traceability Fix
In accordance with `CONFIG/EXPERIMENT_CONTRACT_M0_M7.yaml`, M5 was corrected to isolate map confidence gating completely:
```python
"M5": {
    "name": "B1/N8 + Uncertainty-Aware Map",
    "motion_state": False,    # <-- FIXED: Strict isolation
    "speed_confidence": False,
    "adaptive_nhc": False,
    "innovation_monitor": False,
    "map_confidence": True,
}
```
Additionally, during turn conditions when `motion_state` is disabled, the map-confidence gating dynamically conditions on the raw gyro turning rate $|\omega_z| > 0.15\text{ rad/s}$ ($8.6^\circ/\text{s}$), ensuring causal standalone operation.

### 3.3 The Definitive Empirical Proof
When re-evaluated across all 158 authentic outages with the corrected runner:
- **Isolated M5 (Map-Confidence Alone):**  
  - Median Drift: `54.86%` (matches M0 baseline `54.86%`)  
  - Mean Drift: `123.13%` (vs M0 baseline `123.14%`)  
  - Passes ($\le 10\%$): `9` (matches M0 baseline `9`)  
  - Median Cross-Track Error: `38.04 m` (identical to M0 baseline `38.04 m`)  
  - Median Along-Track Error: `37.10 m` (identical to M0 baseline `37.10 m`)  
  - Median Yaw Error: `36.32°` (identical to M0 baseline `36.32°`)  
- **Combined M6 (Motion-State + Map-Confidence):**  
  - Median Drift: `54.35%`  
  - Mean Drift: `115.12%`  
  - Passes ($\le 10\%$): `10`  
  - Median Cross-Track Error: `31.11 m` (**6.93 m reduction** vs M0/M5)  
  - Median Yaw Error: `37.51°`  

**Conclusion:** The improvement previously observed in M5 was entirely driven by the **Motion-State Manager** (`motion_state: True`), which prevents false dead-reckoning integration during vehicle standstills. Map-matching confidence gating on its own without motion classification has negligible effect on open-loop unmapped trajectory segments, but synergizes with motion state in M6 to produce a $18.2\%$ reduction in cross-track error ($38.04\text{ m} \rightarrow 31.11\text{ m}$).

---

## 4. Complete M0–M7 Benchmark Results

Evaluated on all 158 outages across drives `2019-11-20-14-11-09`, `2019-11-20-14-38-00`, `2019-11-20-15-18-50`, `2019-11-20-15-47-28`, and `2019-11-20-16-09-08`.

| Experiment | Pipeline Configuration | Median Drift (%) | Mean Drift (%) | P25 (%) | P75 (%) | P90 (%) | P95 (%) | Passes ($\le 10\%$) | Along-Track (m) | Cross-Track (m) | Yaw Err (deg) | Runtime (s) |
|:---:|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **M0** | B1/N8 Golden Baseline | **54.86** | 123.14 | 27.91 | 124.40 | 183.53 | 229.92 | 9 / 158 | 37.10 | 38.04 | **36.32°** | 19.84 |
| **M1** | + Motion-State Manager | **54.35** | 115.18 | 28.11 | 118.67 | 171.37 | 203.64 | 10 / 158 | 38.03 | 32.69 | 37.51° | 21.11 |
| **M2** | + Speed Uncertainty | **54.86** | 123.14 | 27.91 | 124.40 | 183.53 | 229.92 | 9 / 158 | 37.10 | 38.04 | 36.32° | 19.88 |
| **M3** | + Adaptive NHC | **54.35** | 115.03 | 27.64 | 118.37 | 171.30 | 203.64 | 9 / 158 | 38.00 | 31.29 | 37.58° | 21.10 |
| **M4** | + Innovation Monitor | 58.16 | 120.26 | 27.66 | 123.33 | 179.29 | 219.17 | 9 / 158 | 36.52 | 32.76 | 37.49° | 20.17 |
| **M5** | + Uncertainty-Aware Map (Isolated) | **54.86** | 123.13 | 27.87 | 124.40 | 183.53 | 230.20 | 9 / 158 | 37.10 | 38.04 | **36.32°** | 20.17 |
| **M6** | + MotionState + MapConf | **54.35** | 115.12 | 28.07 | 118.67 | 171.37 | 203.64 | 10 / 158 | 38.17 | **31.11** | 37.51° | 21.48 |
| **M7** | + Full Integrated Policy | 54.96 | **108.38** | 28.08 | **115.25** | **164.15** | **203.58** | **12 / 158** | **35.81** | **30.90** | 42.98° | 20.95 |

---

## 5. Architectural & Scientific Insights

### 5.1 The Motion-State Manager is the Primary Stabilizer (M1)
The introduction of the causal 6-state motion detector (detecting `STOPPED`, `TURNING`, `ACCEL`, `BRAKE`, `STRAIGHT_CRUISING`) with anti-creep standstill clamp reduces mean drift by **7.96 percentage points** ($123.14\% \rightarrow 115.18\%$) and reduces 95th percentile outlier drift by **26.28 percentage points** ($229.92\% \rightarrow 203.64\%$). This occurs because in stop-and-go driving, small residual accelerometer bias integration accumulates quadratic position error. Clamping velocity to zero whenever variance thresholds $\sigma_a^2 < 0.05\text{ m}^2/\text{s}^4$ and $|\omega_z| < 0.03\text{ rad/s}$ are satisfied completely halts artificial drift during red lights and traffic stops.

### 5.2 The Innovation Monitor Gating Trade-Off (M4)
In M4, gating speed updates when innovation residual exceeded $8.0\text{ m/s}$ caused median drift to degrade from $54.86\%$ to $58.16\%$. During rapid accelerations, the GBDT speed estimator occasionally experiences transient lag. If the filter rejects the GBDT measurement during this window, the ESKF falls back to open-loop inertial double integration, which rapidly diverges due to unmodelled accelerometer scale factors. Hence, aggressive hard innovation gating on forward speed is harmful to the autonomous filter.

### 5.3 The Heading Observability Trade-Off in M7
M7 achieves the highest pass count (12/158 passes $\le 10\%$), the lowest along-track error ($35.81\text{ m}$), and the lowest mean drift ($108.38\%$). However, its median yaw error degrades from $36.32^\circ$ to $42.98^\circ$ ($+6.66^\circ$).
This occurs because scaling up $R_{vx}$ and relaxing lateral NHC during turns reduces the filter's centripetal acceleration constraint ($a_y \approx v \cdot \omega_z$), which is the primary source of yaw gyro bias observability during vehicle maneuvers.
For this reason, **M1 / M6** provides the most balanced production compromise: superior cross-track drift ($31.11\text{ m}$), improved pass rate (10 passes), without compromising heading stability ($37.51^\circ$).
