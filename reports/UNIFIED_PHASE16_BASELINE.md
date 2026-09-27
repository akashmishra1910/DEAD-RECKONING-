# UNIFIED PHASE 16 CLEAN BASELINE REPORT
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Causal Production Baseline Performance on 158 Authentic IO-VNBD Outages

**Date:** September 27, 2026  
**Dataset:** 158 Canonical Outages across `Driver_A_S3b`, `Vta01b`, `Vta04`  
**Speed Source:** Autonomous GBDT (No CAN / No GT)  
**Inference Firewall:** TrajectoryOracleTrap ACTIVE (0 leaks)  

---

## 1. Overall Baseline Metrics

| Metric | Value |
|---|---|
| **Median Drift (%)** | **61.45%** |
| **Mean Drift (%)** | **126.80%** |
| **P25 Drift (%)** | 28.63% |
| **P75 Drift (%)** | 131.83% |
| **P90 Drift (%)** | 182.45% |
| **P95 Drift (%)** | 227.83% |
| **Pass Count (<=10%)** | **11 / 158 (7.0%)** |
| **Median Along-Track Error (m)** | 32.56 m |
| **Median Cross-Track Error (m)** | 48.53 m |
| **Median Yaw Error (deg)** | 37.96° |
| **Median Vertical Error (m)** | 1.06 m |

## 2. Duration Breakdown

| Duration | Count | Median Drift (%) | Mean Drift (%) | Pass Count (<=10%) | Along-Track (m) | Cross-Track (m) |
|---|---|---|---|---|---|---|
| **10s** | 81 | 43.77% | 155.34% | 9/81 (11.1%) | 20.65 m | 4.73 m |
| **30s** | 39 | 58.95% | 84.43% | 1/39 (2.6%) | 66.46 m | 104.86 m |
| **60s** | 25 | 126.12% | 121.30% | 1/25 (4.0%) | 133.90 m | 364.09 m |
| **120s** | 13 | 83.20% | 86.59% | 0/13 (0.0%) | 313.96 m | 510.70 m |

## 3. Motion Regime Breakdown

| Regime | Count | Median Drift (%) | Mean Drift (%) | Along-Track (m) | Cross-Track (m) |
|---|---|---|---|---|---|
| **moving_gt_1mps** | 155 | 58.95% | 83.37% | 32.67 m | 51.13 m |
| **stopped_or_slow_le_1mps** | 3 | 1986.87% | 2370.65% | 16.35 m | 18.52 m |

## 4. Per-Drive Results

| Drive | Count | Median Drift (%) | Mean Drift (%) | Pass Count (<=10%) | Along-Track (m) | Cross-Track (m) | Yaw Error (deg) |
|---|---|---|---|---|---|---|---|
| **Driver_A_S3b** | 110 | 107.50% | 166.76% | 3/110 (2.7%) | 32.89 m | 70.80 m | 64.40° |
| **Vta01b** | 15 | 23.25% | 24.15% | 4/15 (26.7%) | 32.67 m | 3.31 m | 4.19° |
| **Vta03** | 2 | 94.00% | 94.00% | 0/2 (0.0%) | 6.32 m | 11.34 m | 109.47° |
| **Vta04** | 31 | 24.37% | 36.78% | 4/31 (12.9%) | 31.74 m | 28.77 m | 24.28° |

## 5. PS 26168 Criterion Evaluation

- **< 5 m drift over 50 m (< 1 min):** 2 / 31 passed (6.5%), Median Error = 34.22 m
- **< 100 m drift over 1 km (@ 60 km/h):** 1 / 5 passed (20.0%), Median Error = 769.16 m
