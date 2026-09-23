# Claims & Verification Ledger
## ISRO / SIH PS 26168: AI-ML Based Intelligent Dead Reckoning System

This document maps every numerical claim presented in documentation, reports, and benchmarks to the exact test, artifact, or run log that produced it.

---

## Phase 9: Portable C++17 Edge Core & Production Engine (`idr-core`)

| Claim ID | Metric Description | Claimed Value | Measured Value | Verification Artifact / Source | Determination |
|:---:|---|:---:|:---:|---|:---:|
| **CLM-09-01** | C++ GBDT Speed Model Parity | $0.000000\text{ m/s}$ error | $0.000000\text{ m/s}$ | `idr-core/build/tests/test_gbdt_parity` (5,101 vectors) | **PASSED (BIT-FOR-BIT)** |
| **CLM-09-02** | SO(3) Euler $\leftrightarrow$ DCM Roundtrip | $< 10^{-12}$ | $1.11 \times 10^{-16}$ | `idr-core/build/tests/test_math_golden_parity` | **PASSED** |
| **CLM-09-03** | Rodrigues Vector Propagation | $< 10^{-12}$ | $8.88 \times 10^{-16}$ | `idr-core/build/tests/test_math_golden_parity` | **PASSED** |
| **CLM-09-04** | 45-Feature Extraction Parity | $< 10^{-5}$ | $2.33 \times 10^{-9}$ | `idr-core/build/tests/test_feature_vibration` | **PASSED** |
| **CLM-09-05** | In-Vehicle Alignment Error | $< 10^{-12}\text{ rad}$ | $3.55 \times 10^{-15}\text{ rad}$ | `idr-core/build/tests/test_alignment` | **PASSED** |
| **CLM-09-06** | 15-State ESKF Propagation Parity | $< 10^{-12}\text{ m}$ | $4.44 \times 10^{-15}\text{ m}$ | `idr-core/build/tests/test_eskf` | **PASSED** |
| **CLM-09-07** | 158-Outage Max Python $\leftrightarrow$ C++ Delta | $< 0.050\text{ m}$ | $1.184180 \times 10^{-11}\text{ m}$ | `reports/phase9_python_cpp_equivalence.csv` | **PASSED (11.8 picometers)** |
| **CLM-09-08** | 158-Outage Mean Python $\leftrightarrow$ C++ Delta | $< 0.010\text{ m}$ | $1.014235 \times 10^{-12}\text{ m}$ | `reports/phase9_python_cpp_equivalence.csv` | **PASSED (1.0 picometer)** |
| **CLM-09-09** | 158-Outage Drift % Delta | $< 0.010\%$ | $1.979709 \times 10^{-12}\%$ | `reports/phase9_python_cpp_equivalence.csv` | **PASSED** |
| **CLM-09-10** | Tactical High-Rate Throughput | $\ge 200\text{ Hz}$ | **$56,026.7\text{ Hz}$** | `idr-cli --benchmark` soak test (100k cycles) | **PASSED (280x req)** |
| **CLM-09-11** | Tactical High-Rate Mean Latency | $< 0.20\text{ ms}$ | **$17.85\ \mu\text{s}$ ($0.018\text{ ms}$)** | `idr-cli --benchmark` soak test | **PASSED (11x faster)** |
| **CLM-09-12** | Maximum Resident Set Size (RSS) | $< 2.5\text{ MB}$ | **$2,310,144\text{ B}$ ($2.20\text{ MB}$)** | `/usr/bin/time -l ./idr-cli --benchmark` | **PASSED** |
| **CLM-09-13** | Peak Memory Footprint | $< 2.5\text{ MB}$ | **$1,507,664\text{ B}$ ($1.44\text{ MB}$)** | `/usr/bin/time -l ./idr-cli --benchmark` | **PASSED** |
| **CLM-09-14** | Per-Sample Loop Heap Allocations | 0 bytes | 0 bytes | Zero dynamic container growth in tactical loop | **PASSED** |
| **CLM-09-15** | Python Reference Regression Suite | 100% (155/155) | 100% (155/155) | `pytest` | **PASSED** |
| **CLM-09-16** | Native C++ CTest Suite | 100% (10/10) | 100% (10/10) | `ctest --test-dir idr-core/build` | **PASSED** |
| **CLM-09-17** | Third-Party ML Runtime Elimination | Zero deps | 0 external ML libs | Transpiled branchless static C++ tables | **PASSED** |
| **CLM-09-18** | Two-Wheeler Scope Exclusion | Out of scope | 0 code / 0 branches | Codebase audit | **PASSED** |

---

## Phase 0–8 Reference System Benchmarks

| Metric | Reference Value | Dataset / Conditions |
|---|:---:|---|
| **Huber GBDT Speed RMSE** | $3.63\text{ m/s}$ | Held-out authentic IO-VNBD drives (`Driver_A_S3b`, `Vta01b`) |
| **Drift @ 60s Outage (M7 Full)** | $2.88\%$ | 158 authentic outages, IO-VNBD |
| **Drift @ 120s Outage (M7 Full)** | $3.45\%$ | Extended authentic outages, IO-VNBD |
| **PS Benchmark Threshold 1** | $< 5\text{ m}$ over $50\text{ m}$ | PASSED across all authentic test corridors |
| **PS Benchmark Threshold 2** | $< 100\text{ m}$ over $1\text{ km}$ @ $60\text{ km/h}$ | PASSED across all authentic test corridors |
| **Flyover Layer Disambiguation** | $99.7\text{--}100\%$ | 3D HMM with elevation barrier $\|\Delta h\| > 4\text{ m}$ |
