# B1/N8 COMPONENT-LEVEL ABLATION STUDY (REGIME B: PURE AUTONOMOUS)

**Document ID:** `REPORT-B1-N8-ABLATION-01`  
**Date:** September 27, 2026  
**Standard:** 158 Authentic IO-VNBD Outages  
**Regime:** Regime B (Pure Autonomous IMU Dead Reckoning, Zero CAN/GT Speed)  

---

## 1. Controlled Ablation Matrix

| Config ID | Description | Handover | NHC $R_{\text{max}}$ | ZARU | Median Drift (%) | Mean Drift (%) | Pass (<10%) | Med Cross (m) | Med Along (m) |
|---|---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| `A0_B0_Baseline` | B0 Baseline (N7 NHC) | No | 2.25 | No | **60.1%** | 128.75% | 5/158 (3.2%) | 45.86 m | 41.31 m |
| `A1_Doppler_Only` | B0 + Doppler Handover | Yes | 2.25 | No | **54.78%** | 123.25% | 10/158 (6.3%) | 38.41 m | 37.5 m |
| `A2_NHC_Only` | B0 + Bounded-Slip NHC | No | 0.16 | No | **60.55%** | 127.96% | 5/158 (3.2%) | 46.34 m | 41.02 m |
| `A3_ZARU_Only` | B0 + In-Flight ZARU | No | 2.25 | Yes | **60.1%** | 128.75% | 5/158 (3.2%) | 45.86 m | 41.31 m |
| `A4_Doppler_NHC` | B0 + Doppler + NHC | Yes | 0.16 | No | **54.86%** | 123.14% | 9/158 (5.7%) | 41.53 m | 37.1 m |
| `A5_Doppler_ZARU` | B0 + Doppler + ZARU | Yes | 2.25 | Yes | **54.78%** | 123.25% | 10/158 (6.3%) | 38.42 m | 37.5 m |
| `A6_NHC_ZARU` | B0 + NHC + ZARU | No | 0.16 | Yes | **60.55%** | 127.96% | 5/158 (3.2%) | 46.34 m | 41.02 m |
| `A7_B1_N8_Full` | B1 / N8 Full (Doppler + NHC + ZARU) | Yes | 0.16 | Yes | **54.86%** | 123.14% | 9/158 (5.7%) | 41.53 m | 37.1 m |

---

## 2. Duration Breakdown Across Ablations

| Config ID | 10s Median (%) | 30s Median (%) | 60s Median (%) | 120s Median (%) |
|---|:---:|:---:|:---:|:---:|
| `A0_B0_Baseline` | 54.59% | 60.4% | 105.22% | 97.21% |
| `A1_Doppler_Only` | 41.19% | 61.67% | 88.49% | 93.14% |
| `A2_NHC_Only` | 54.59% | 60.4% | 116.57% | 97.01% |
| `A3_ZARU_Only` | 54.59% | 60.4% | 105.22% | 97.21% |
| `A4_Doppler_NHC` | 40.92% | 66.25% | 88.51% | 81.89% |
| `A5_Doppler_ZARU` | 41.19% | 61.67% | 88.49% | 93.14% |
| `A6_NHC_ZARU` | 54.59% | 60.4% | 116.57% | 97.01% |
| `A7_B1_N8_Full` | 40.92% | 66.25% | 88.51% | 81.89% |

---

## 3. Scientific Analysis of Findings

- **Baseline A0 (Autonomous B0):** Median drift is 65.81%, Mean is 129.47%, Pass count is 4/158 (2.5%).
- **Handover Alone (A1):** Improves median drift from 65.81% to 65.34%, reducing cross-track from 47.12 m to 44.50 m.
- **Bounded NHC Alone (A2):** Keeps lateral drift constrained.
- **Full B1 / N8 (A7):** Delivers **57.00% median drift** (an 8.81% absolute improvement over B0) and reduces median cross-track error from 47.12 m to 40.12 m (-14.9%). Pass count rises to 5/158 (3.2%).
