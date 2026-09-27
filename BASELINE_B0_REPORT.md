# BASELINE B0 REPORT: FROZEN PRODUCTION BENCHMARK

**Document ID:** `REPORT-BASELINE-B0-01`  
**Date:** September 27, 2026  
**Status:** FROZEN REFERENCE BASELINE  
**Author:** Autonomous Navigation Engineering Team  
**Problem Statement:** ISRO PS 26168 — AI-ML Based Intelligent Dead Reckoning System

---

## 1. Baseline Definition

The **B0 Baseline** represents the authoritative, reproducible performance of the production navigation pipeline across all **158 authentic GNSS outages** extracted from the official IO-VNBD dataset.

- **Baseline ID:** `B0`
- **Architectural Specification:** N7 Production Model (Bounded Adaptive NHC + Causal Multi-Hypothesis Map Fusion).
- **Speed Regressor:** Top-45 Huber GBDT model ([`models/production/gbdt_speed_model_iovnbd_top45.joblib`](file:///Users/akashmishra/SIH%20MOdel/models/production/gbdt_speed_model_iovnbd_top45.joblib)).
- **Filter Mechanization:** 15-State Error-State Kalman Filter (ESKF).
- **Authoritative Data Source:** [`reports/final_track_b_outage_results.csv`](file:///Users/akashmishra/SIH%20MOdel/reports/final_track_b_outage_results.csv), [`reports/phase12_benchmark_comparison.csv`](file:///Users/akashmishra/SIH%20MOdel/reports/phase12_benchmark_comparison.csv).

---

## 2. Definitive Summary Metrics

| Metric | B0 Frozen Baseline Value | Unit | Evaluation Standard |
|---|:---:|:---:|---|
| **Total Evaluated Outages** | **158** | Outages | Complete naturalistic dataset |
| **Median Drift** | **35.71%** | % of distance | 50th percentile across all outages |
| **Mean Drift** | **45.86%** | % of distance | Arithmetic mean |
| **90th Percentile (P90)** | **105.58%** | % of distance | 90th percentile |
| **95th Percentile (P95)** | **133.71%** | % of distance | 95th percentile |
| **Worst Outage Drift** | **173.20%** | % of distance | Maximum observed drift |
| **Pass Count (<10% Drift)** | **36 / 158** | Outages | Exact $\le 10.00\%$ criterion |
| **Pass Rate (<10% Drift)** | **22.78%** | % | Overall success rate |

---

## 3. Duration-Stratified Performance Breakdown

| Duration Bucket | Outage Count | B0 Median Drift (%) | B0 Mean Drift (%) | Pass Count (<10%) | Pass Rate (%) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Short (10 s)** | 81 | **20.10%** | 35.88% | 27 / 81 | 33.33% |
| **Medium (30 s)** | 39 | **36.24%** | 46.12% | 6 / 39 | 15.38% |
| **Extended (60 s)** | 25 | **59.35%** | 62.43% | 2 / 25 | 8.00% |
| **Severe (120 s)** | 13 | **51.24%** | 56.09% | 1 / 13 | 7.69% |

---

## 4. Dynamic Road Geometry Breakdown

| Slice | Outage Count | B0 Median Drift (%) | B0 Mean Drift (%) | Pass Count (<10%) | Pass Rate (%) |
|---|:---:|:---:|:---:|:---:|:---:|
| **Straight Segments** | 12 | **29.81%** | 34.61% | 4 / 12 | 33.33% |
| **Curved / Dynamic Segments** | 146 | **35.71%** | 46.79% | 32 / 146 | 21.92% |

---

## 5. Worst 10 Outages Under Baseline B0

The 10 most challenging outages in the baseline population:

| Rank | Outage ID | Drive Name | Duration (s) | Distance (m) | Drift (%) | Geometry |
|:---:|---|---|:---:|:---:|:---:|:---:|
| 1 | `OUT-060s-014_Driver_A_S3b` | Driver_A_S3b | 60.0 | 44.91 | **173.20%** | Curved |
| 2 | `OUT-060s-001_Driver_A_S3b` | Driver_A_S3b | 60.0 | 179.35 | **156.73%** | Curved |
| 3 | `OUT-010s-060_Vta04` | Vta04 | 10.0 | 7.91 | **155.79%** | Curved |
| 4 | `OUT-010s-017_Driver_A_S3b` | Driver_A_S3b | 10.0 | 31.24 | **154.35%** | Curved |
| 5 | `OUT-030s-006_Driver_A_S3b` | Driver_A_S3b | 30.0 | 60.28 | **144.55%** | Curved |
| 6 | `OUT-060s-004_Driver_A_S3b` | Driver_A_S3b | 60.0 | 165.71 | **140.82%** | Curved |
| 7 | `OUT-060s-003_Driver_A_S3b` | Driver_A_S3b | 60.0 | 142.34 | **140.23%** | Curved |
| 8 | `OUT-060s-011_Driver_A_S3b` | Driver_A_S3b | 60.0 | 114.28 | **134.46%** | Curved |
| 9 | `OUT-030s-002_Driver_A_S3b` | Driver_A_S3b | 30.0 | 85.91 | **133.58%** | Curved |
| 10 | `OUT-060s-017_Driver_A_S3b` | Driver_A_S3b | 60.0 | 97.45 | **127.21%** | Curved |

---

## 6. Reproducibility & Integrity Assurance

- All numbers in this report are mathematically frozen and sourced directly from verified test logs.
- This baseline is stored in [`BASELINE_B0_RESULTS.json`](file:///Users/akashmishra/SIH%20MOdel/BASELINE_B0_RESULTS.json).
- Any candidate architecture (B1, B2, ...) will be objectively compared against this exact baseline using [`scripts/evaluate_candidate.py`](file:///Users/akashmishra/SIH%20MOdel/scripts/evaluate_candidate.py).
