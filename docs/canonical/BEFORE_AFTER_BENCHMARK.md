# BEFORE & AFTER BENCHMARK & METRICS COMPARISON
## SIH 2026 PS 26168 — AI-ML Based Intelligent Dead Reckoning System

**Date:** September 27, 2026  
**Evaluation:** Comprehensive Before vs. After Correction Benchmark  
**Status:** VALIDATED ON AUTHENTIC 158-OUTAGE IO-VNBD DATASET & HARDWARE

---

## 1. System Performance Comparison Table

| Metric | Before Fix (Broken / Baseline) | After Fix (Production Corrected) | Measured Change & Impact |
|---|---|---|---|
| **158-Outage Median Drift (%)** | 35.71% (N7 baseline) | **27.94%** | **-7.77 percentage points** (21.8% relative error reduction) |
| **Pass Count (<10% PS Target)** | 36 / 158 (22.78%) | **48 / 158 (30.38%)** | **+12 passed outages (+33.3% relative expansion)** |
| **Median Cross-Track Error** | 24.62 m | **18.83 m** | **-5.79 m lateral precision improvement** |
| **Median Along-Track Error** | 19.34 m | **15.01 m** | **-4.33 m longitudinal precision improvement** |
| **Standstill Drift (120s idle)** | 22.40 m | **0.18 m** | **-99.20% drift reduction during stops** |
| **Speed RMSE (60s DR)** | >12.40 m/s (open-loop) | **1.14 m/s** (GBDT aided) | **-90.8% speed error reduction** |
| **GNSS Satellite In-View Count** | Flashing 0 / Resetting | **Realtime Total in View & in Fix** | **100% Fixed (Continuous provenance)** |
| **NavIC Satellite In-View Count** | Flashing 0 / Resetting | **Realtime NavIC Count (e.g. 4–7 sats)** | **100% Fixed (Constitutional provenance)** |
| **Live Cursor Route Alignment** | Diverged off-road in DR | **Smooth road-aligned trajectory** | **Unified 3D velocity vector constraint** |
| **GNSS $\to$ DR Recovery Step** | Potential jump | **Continuous Chi² innovation gate** | **Smooth transition without visual discontinuity** |
| **Edge Throughput (C++)** | 56,026 Hz | **55,480 Hz** | **>5,500× real-time 10 Hz budget** |
| **Per-Cycle Latency (C++)** | 17.85 μs | **18.02 μs** | **Negligible impact (<0.2 μs overhead)** |
| **Memory Footprint (RSS)** | <2.4 MB | **<2.4 MB** | **Zero heap allocations in step execution** |
| **Test Suite Passing** | 266 / 266 passing | **266 Python + All Android Unit Tests** | **100% passing across all layers** |

---

## 2. Outage Duration Breakdown (158 Authentic IO-VNBD Outages)

| Outage Duration | Count | Passed (<10%) | Pass Rate (%) | Median Drift (%) | Median Cross-Track (m) | Median Along-Track (m) |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **10 seconds** | 81 | **35** | **43.21%** | **14.30%** | **4.79 m** | **4.21 m** |
| **30 seconds** | 39 | **10** | **25.64%** | **37.34%** | **37.13 m** | **44.17 m** |
| **60 seconds** | 25 | **3** | **12.00%** | **60.81%** | **113.65 m** | **82.37 m** |
| **120 seconds** | 13 | **0** | **0.00%** | **63.02%** | **270.27 m** | **304.16 m** |
| **Total Population** | **158** | **48** | **30.38%** | **27.94%** | **18.83 m** | **15.01 m** |

---

## 3. PS Acceptance Discipline & Remaining Gap Statement

- **Authoritative PS Benchmark Target:** $<10\%$ positional drift across all GNSS-denied intervals.
- **Achieved Production Performance:** **27.94% overall median drift**, **48 / 158 passes (30.38%)**.
- **Disclosed Remaining Gap:** **17.94 percentage points**.
- **Physical Root Cause:** Smartphone MEMS consumer-grade sensors without wheel-speed odometry (OBD-II/CAN) or tactical FOG hardware experience physics-based gyro integration drift across long 60s/120s outages with consecutive 90° turns.
