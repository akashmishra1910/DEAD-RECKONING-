# SUBMISSION CLAIMS LEDGER — SIH26168
## Authoritative Source of Truth for Presentation & Submission Claims

**Document ID:** `DOC-CANONICAL-CLAIMS-LEDGER-06`  
**Version:** 2.0.0 (Submission Freeze)  
**Date:** September 26, 2026  
**Status Standard:** Rule Zero — Explicit Presentation Clearance & Required Qualifiers  

---

## 1. Presentation Claims Matrix

| Claim ID | Potential PPT Claim | Claim Category | Numerical Value | Evaluated Dataset | Evaluated Population | Source File | Safe to Present? | Mandatory Qualifier for Slide / Speaker |
|:---:|---|---|---|---|---|---|:---:|---|
| **CLM-PPT-01** | "15-State Error-State Kalman Filter Formulation" | A. VERIFIED PRODUCTION | 15 States ($p, v, q, b_a, b_g$) | IO-VNBD / Synthetic | All drives | `idr-core/src/eskf.cpp` | **YES** | None required. |
| **CLM-PPT-02** | "AI-Based Forward Speed Estimation from Smartphone IMU" | A. VERIFIED PRODUCTION | $R^2 = 0.884$, RMSE $1.21\text{ m/s}$ | IO-VNBD Real Drives | 10,019 samples | `idr-core/src/gbdt_speed.cpp` | **YES** | State: "Transpiled branchless GBDT; no OBD-II/CAN bus required." |
| **CLM-PPT-03** | "Evaluated across 158 Real GNSS Outages on IO-VNBD" | B. VERIFIED RESEARCH | 158 Outages | IO-VNBD Real Drives | Complete dataset | `reports/final_track_b_benchmark.csv` | **YES** | State: "Mandated ISRO dataset." |
| **CLM-PPT-04** | "2.01% Drift on 60-Second GNSS-Denied Corridor" | A. VERIFIED PRODUCTION | 2.01% of distance | IO-VNBD / Field Data | 60s Outage subset | `reports/final_track_b_outage_results.csv` | **YES** | State: "Measured on realistic 60s outage corridor." |
| **CLM-PPT-05** | "2.88% Drift on 120-Second GNSS-Denied Corridor" | A. VERIFIED PRODUCTION | 2.88% of distance | IO-VNBD / Field Data | 120s Outage subset | `reports/final_track_b_outage_results.csv` | **YES** | State: "Measured on extended 120s outage corridor." |
| **CLM-PPT-06** | "3D Multi-Layer Flyover Layer Disambiguation" | A. VERIFIED PRODUCTION | 99.7% – 100.0% | Multi-layer test maps | Elevation barrier tests | `reports/map_matching_3d_evaluation.json` | **YES** | State: "3D HMM with $|\Delta h| > 4.0\text{ m}$ elevation barrier." |
| **CLM-PPT-07** | "Tactical Edge Engine Throughput" | A. VERIFIED PRODUCTION | $>50,000\text{ Hz}$ ($56,026\text{ Hz}$) | Desktop/Edge Linux | 100,000 cycles | `reports/idr_cli_output.csv` | **YES** | State: "Benchmarked on streaming IMU logs; $>280\times$ the 200 Hz PS requirement." |
| **CLM-PPT-08** | "Sub-Millisecond Per-Sample Latency" | A. VERIFIED PRODUCTION | $17.85\ \mu\text{s}$ (0.018 ms) | Core C++ Engine | Continuous stream | `reports/idr_cli_output.csv` | **YES** | State: "Average per-sample filter step latency." |
| **CLM-PPT-09** | "Ultra-Low Memory Footprint" | A. VERIFIED PRODUCTION | $<2.4\text{ MB}$ RSS | C++ Core Binary | Continuous stream | Memory Profiler report | **YES** | State: "Zero dynamic heap allocation during streaming." |
| **CLM-PPT-10** | "ISRO NavIC / IRNSS Constellation Tracking" | A. VERIFIED PRODUCTION | Real-time PRN / SNR | Android Hardware | Live Skyview | `android/app/.../GnssNavicScreen.kt` | **YES** | State: "Reads Android `GnssStatus` for `CONSTELLATION_IRNSS`." |
| **CLM-PPT-11** | "Best Research Median Drift: 27.25%" | B. VERIFIED RESEARCH | 27.25% Median | IO-VNBD Real Drives | Full 158 outages | `reports/phase16_final_benchmark.csv` | **YES** | **MUST STATE:** "Achieved in Phase 16 Python research; C++ production baseline is 35.71%." |
| **CLM-PPT-12** | "Best Research Pass Count: 49/158 Outages" | B. VERIFIED RESEARCH | 49 / 158 Passes (31.0%) | IO-VNBD Real Drives | Full 158 outages | `reports/phase17_final_benchmark.csv` | **YES** | **MUST STATE:** "Achieved in Phase 17 Python research; C++ production baseline is 36/158." |
| **CLM-PPT-13** | "Dataset-Wide PS <10% Bound Achieved" | E. NOT VERIFIED | $<10\%$ overall | Full dataset | Full 158 outages | N/A | **NO (FORBIDDEN)** | **DO NOT CLAIM.** State that realistic corridors pass ($1.62\%-2.88\%$), but full 158-outage median is $35.71\%$ in C++ and $27.25\%$ in Python. |
| **CLM-PPT-14** | "Physical FOG Hardware Validated" | F. PENDING | N/A | Physical FOG sensor | Physical bench test | N/A | **NO (FORBIDDEN)** | **DO NOT CLAIM.** State: "High-rate FOG streaming validated in software; physical hardware integration pending." |
