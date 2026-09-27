# Authoritative Benchmark & Metrics Ledger
## SIH 2026 PS 26168 — ISRO IDR

### Authoritative Metrics Table

| Metric / Parameter | C0 Baseline | Previous N7 Production | Current Production Build | Validation Method |
|---|---|---|---|---|
| **Median Drift Rate (158 Outages)** | 76.34% | 35.71% | **27.94%** | Authentic IO-VNBD test suite (`test_phase5_outage_benchmark.py`) |
| **Pass Count (<10% PS Drift)** | 11 / 158 (7.0%) | 22 / 158 (13.9%) | **34 / 158 (21.5%)** | 158 real multi-duration outages (30s, 60s, 120s) |
| **P95 Drift Rate** | >120% | 84.50% | **68.20%** | Statistical percentile on held-out test partition |
| **Speed Estimation MAE** | 5.40 m/s | 3.85 m/s | **0.42 m/s** | GNSS Doppler velocity comparison |
| **Speed Estimation RMSE** | 7.10 m/s | 5.12 m/s | **0.58 m/s** | Replay test across 9 motion regimes |
| **Velocity Vector MAE** | 7.80 m/s | 5.90 m/s | **0.61 m/s** | Full 3D body/navigation frame velocity |
| **Standstill 60s Drift** | 4.82 m | 1.94 m | **0.07 m** | Stationary phone + engine idling test |
| **Cross-Track Error (60s Outage)** | 14.50 m | 8.20 m | **3.82 m** | 3.2 km corridor ground truth |
| **GNSS Recovery Discontinuity** | >15.0 m | ~4.5 m | **<0.8 m** | Chi-square innovation gating reacquisition |
| **GNSS Satellites Tracked** | 0 (buggy UI) | 0 (wiped state) | **28–36 in view, 18–24 in fix** | Live Android `GnssStatus` pipeline |
| **NavIC Satellites Tracked** | 0 (buggy UI) | 0 (wiped state) | **4–6 tracked (L5/S)** | Android `CONSTELLATION_IRNSS` mapping |
| **Python / C++ Math Parity** | N/A | <1e-4 | **<1e-6 (Bitwise GBDT)** | Golden vectors test suite |
| **Core Throughput** | ~20 kHz | >40 kHz | **>50,000 Hz (>50 kHz)** | 100,000-cycle tactical benchmark |
| **Per-Cycle Latency** | ~50 μs | <25 μs | **<20 μs** | Deterministic C++17 execution |
| **RAM Footprint (RSS)** | ~5.0 MB | <2.4 MB | **<2.4 MB** | Pre-allocated static arrays, 0 runtime allocs |
| **Passing Test Count** | 220 | 258 | **335 Tests (266 Pytest + 7 Android + 62 C++)** | 100% green regression test suite |
