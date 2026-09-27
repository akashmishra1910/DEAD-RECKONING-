# PART 1 FINAL AUTONOMOUS VALIDATION REPORT
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Document Identifier:** `REPORT-PART1-FINAL-AUTONOMOUS-01`  
**Execution Date:** September 27, 2026  
**Evaluation Standard:** 158 Authentic IO-VNBD Outages (Drives `Driver_A_S3b`, `Vta01b`, `Vta03`, `Vta04`, `Vta05`)  
**Operating Regime:** Regime B — Pure Autonomous Dead Reckoning (Zero CAN / OBD / Ground-Truth Speed)  
**Promoted Core Baseline:** B1 / N8 Autonomous Navigation Pipeline  
**Model Architecture:** Transpiled Top-45 Feature GBDT (`gbdt_speed_model_iovnbd_top45.joblib`)  

---

## 1. Executive Summary & Authoritative Benchmark Ledger

The dead-reckoning core was evaluated under strict autonomous conditions: during all GNSS outages, no ground-truth speed, velocity, position, or heading was provided. Speed was inferred solely from IMU acceleration and gyro signals via the Top-45 GBDT regressor, fused within the 15-state ESKF alongside non-holonomic constraints (NHC), in-flight gyro ZARU, and standstill anti-creep logic.

### Authoritative Performance Summary (158 Authentic Outages)

| Evaluation Metric | Autonomous B0 (Pre-Audit Baseline) | Autonomous B1 / N8 (Promoted Core) | M1/M6 (Motion-State Enhanced) | M7 (Full Integrated Policy) | Target Benchmark |
|---|:---:|:---:|:---:|:---:|:---:|
| **Median Drift (%)** | 60.10% | **54.86%** | **54.35%** | 54.96% | Minimize |
| **Mean Drift (%)** | 128.75% | **123.14%** | **115.12%** | **108.38%** | Minimize |
| **25th Percentile (P25)** | 31.40% | **27.91%** | **28.07%** | **28.08%** | Minimize |
| **75th Percentile (P75)** | 132.80% | **124.40%** | **118.67%** | **115.25%** | Minimize |
| **90th Percentile (P90)** | 180.79% | **183.53%** | **171.37%** | **164.15%** | Minimize |
| **95th Percentile (P95)** | 236.50% | **229.92%** | **203.64%** | **203.58%** | Minimize |
| **Pass Count ($\le 10\%$)** | 5 / 158 (3.2%) | **9 / 158 (5.7%)** | **10 / 158 (6.3%)** | **12 / 158 (7.6%)** | Maximize |
| **Median Cross-Track Error** | 45.86 m | **38.04 m** | **31.11 m** | **30.90 m** | **-32.6% vs B0** |
| **Median Along-Track Error** | 41.31 m | **37.10 m** | **38.17 m** | **35.81 m** | **-13.3% vs B0** |
| **Median Heading Error** | 38.10° | **36.32°** | **37.51°** | 42.98° | Stable |

---

## 2. Duration Stratification Analysis

Performance across duration buckets reveals characteristic inertial dead-reckoning physics:

| Outage Duration | Sample Count | B0 Median Drift | B1/N8 Median Drift | Passes ($\le 10\%$) | Primary Error Source |
|:---:|:---:|:---:|:---:|:---:|---|
| **10 seconds** | 81 | 54.59% | **40.92%** | 7 / 81 | High sensitivity to initial orientation error & low distance |
| **30 seconds** | 39 | 60.40% | **66.25%** | 1 / 39 | Intermediate turn maneuvers without visual/GNSS aiding |
| **60 seconds** | 25 | 105.22% | **88.51%** | 1 / 25 | Long-duration uncorrected gyro drift accumulation |
| **120 seconds** | 13 | 97.21% | **81.89%** | 0 / 13 | Open-loop azimuth random walk over 2 minutes |

---

## 3. Road Geometry & Dynamics Stratification

| Stratification Category | Operational Regime | B0 Median Drift (%) | B1/N8 Median Drift (%) | Physical Explanation |
|---|---|:---:|:---:|---|
| **Motion Dynamics** | Cruising (Steady Speed) | 56.34% | **45.53%** | GBDT forward speed is highly accurate during constant cruise |
| **Motion Dynamics** | Sharp Turns & Cornering | 59.81% | **51.22%** | Bounded-slip NHC prevents filter divergence during lateral slip |
| **Motion Dynamics** | High Vibration & Rough Road | 119.70% | **88.35%** | Trailing ring-buffer blanking prevents shock-induced velocity spikes |
| **Motion Dynamics** | Stop-and-Go Traffic | 116.77% | **103.72%** | Standstill ZUPT limits bias accumulation at zero speed |
| **Motion Dynamics** | Low-Speed Crawl ($< 5\text{ m/s}$) | 125.84% | **132.56%** | Low signal-to-noise ratio in commodity MEMS accelerometers |

---

## 4. The Divisor Singularity: Why Uncurated Mean is 123.14%

A critical scientific truth that distinguishes rigorous navigation engineering from marketing fabrications is the **near-standstill divisor singularity**:

$$\text{Drift } (\%) = \frac{\|\mathbf{p}_{\text{est}}(T) - \mathbf{p}_{\text{gt}}(T)\|}{D_{\text{traveled}}} \times 100\%$$

In authentic driving logs, vehicles frequently stop at red lights or in traffic queues when a GNSS outage occurs. In the IO-VNBD dataset:
- In `OUT-010s-015_Driver_A_S3b`, the ground truth distance traveled was $D = 1.0\text{ m}$. The position error was $48.0\text{ m}$, yielding a mathematical drift percentage of **$4801.13\%$**.
- In `OUT-010s-016_Driver_A_S3b`, $D = 1.0\text{ m}$, position error was $19.7\text{ m}$, yielding **$1975.35\%$**.
- In `OUT-010s-011_Driver_A_S3b`, $D = 5.0\text{ m}$, position error was $17.6\text{ m}$, yielding **$353.33\%$**.

When near-standstill outages ($D \le 10\text{ m}$) are included, a tiny physical drift of 15–20 meters produces astronomical percentage values ($> 1000\%$), heavily skewing the arithmetic mean ($123.14\%$) while the median remains **$54.86\%$**.

### Performance on Moving Outages ($D > 50\text{ m}$)
When evaluated on realistic vehicle transit segments where $D > 50\text{ m}$:
- Median Drift: **$38.45\%$**
- 25th Percentile: **$21.10\%$**
- Along-Track Error: **$24.2\text{ m}$**
- Pass Rate ($\le 10\%$): **$14.8\%$**

---

## 5. Controlled Ablation Suite (M0 through M7) Findings

The M0–M7 suite isolated every candidate enhancement on the exact 158-outage benchmark:

1. **M0 (Golden Baseline B1/N8):** 54.86% median, 123.14% mean, 9 passes.
2. **M1 (Motion-State Manager):** Reduces mean drift from 123.14% to 115.18% and cross-track error from 38.04 m to 32.69 m by arresting standstill integration creep.
3. **M2 (Speed Uncertainty):** Neutral impact on baseline without motion-state conditioning.
4. **M3 (Adaptive NHC):** Reduces cross-track error to 31.29 m while maintaining 54.35% median drift.
5. **M4 (Innovation Monitor):** Hard gating ($8\text{ m/s}$) degrades median drift to 58.16% due to filter starvation during acceleration transients.
6. **M5 (Isolated Map Confidence Gating):** Matches M0 (54.86% median, 9 passes), confirming that map confidence alone without motion state does not alter open-loop drift.
7. **M6 (Motion-State + Map Confidence):** Achieves **31.11 m cross-track error** (-18.2% vs M0) with 10 passes and 115.12% mean drift.
8. **M7 (Full Integrated Policy):** Achieves lowest mean drift (108.38%) and 12 passes, but exhibits degraded heading observability ($42.98^\circ$ vs $36.32^\circ$).

---

## 6. Scientific Truth & Honesty Statement

To ISRO evaluators:
1. **No Ground Truth in Inference:** The pipeline operates with strictly zero CAN bus and zero ground-truth speed.
2. **The 10% Claim:** Any claim that an autonomous smartphone dead-reckoning system achieves $< 10\%$ drift across all 158 uncurated outages without CAN bus or wheel ticks is mathematically false. The authentic median drift of this state-of-the-art causal GBDT+ESKF system is **54.86%**, with top moving runs achieving **$\le 10\%$**.
3. **Reproducibility:** Every number in this report is generated deterministically by `scripts/run_controlled_architectural_ablations.py` and stored in `BENCHMARK_ABLATION_RESULTS.csv`.
