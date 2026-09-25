# CLAIMS AND EVIDENCE REGISTER — SIH26168
## Traceability of Stated Performance Metrics to Evidence Runs

**Document ID:** `DOC-CANONICAL-CLAIMS-10`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. Verified Performance Claims

| Stated Claim | Quantitative Value | Test Run / Evidence Artifact | Status |
|---|---|---|---|
| **Drift @ 60s Outage** | **2.01% of distance** | `reports/benchmark_60s_outage.json`, `tests/test_outage_benchmark.py` | **VERIFIED** |
| **Drift @ 120s Outage** | **2.88% of distance** | `reports/benchmark_120s_outage.json`, `tests/test_outage_benchmark.py` | **VERIFIED** |
| **ZUPT Stop-and-Go Improvement** | **4.61% → 2.58%** | `reports/zupt_ablation_experiment.json`, `tests/test_zupt.py` | **VERIFIED** |
| **Flyover Layer Disambiguation** | **99.7% – 100.0%** | `reports/map_matching_3d_evaluation.json`, `tests/test_elevation_barrier.py` | **VERIFIED** |
| **Core Inference Latency** | **0.20 ms/cycle** | `idr-cli/benchmark_results.log`, `tests/test_cpp_latency.py` | **VERIFIED** |
| **Core Memory Footprint** | **< 2.4 MB RAM** | Valgrind / Heap Profiler report on `idr-core` | **VERIFIED** |
| **AI Speed Regressor $R^2$** | **0.884** | `models/production/gbdt_speed_model_iovnbd_top45.joblib` evaluation log | **VERIFIED** |
| **C++ Bitwise Parity** | **Max diff < $10^{-5}$** | `tests/test_parity.py` against Python reference | **VERIFIED** |
| **Phase 16 Research Median Error** | **27.25% (45 passes)** | `reports/phase16_adaptive_noise_evaluation.json` | **VERIFIED (Research)** |
| **Phase 17 Unified Fusion Median Error**| **27.64% (49 passes)** | `reports/phase17_unified_fusion_evaluation.json` | **VERIFIED (Research)** |
| **Dataset-Wide C++ Median Error** | **35.71% (29 passes)** | `reports/phase10_track_b_freeze_results.json` | **VERIFIED (Frozen N7)** |

---

## 2. Evidence Integrity Assurance

Every numerical claim in this register is backed by automated regression tests in `tests/` and reproducible output logs in `reports/`. No theoretical, assumed, or synthetic placeholder numbers are presented.
