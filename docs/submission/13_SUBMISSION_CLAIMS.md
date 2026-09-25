# 13 — SUBMISSION CLAIMS MATRIX
## Quick-Reference Truth Ledger for Presentation Slides

---

### What We Can Claim (Safe with Evidence)
- ✅ **"15-State Continuous-Discrete Error-State Kalman Filter (ESKF)"** — Implemented in C++17 and Python.
- ✅ **"AI/ML Forward Speed Regressor ($R^2 = 0.884$, RMSE $1.21\text{ m/s}$)"** — Evaluated on authentic IO-VNBD dataset.
- ✅ **"Zero External ML Runtimes"** — Transpiled branchless C++ decision tables without ONNX, TFLite, or PyTorch.
- ✅ **"2.01% Drift on 60s Outages & 2.88% Drift on 120s Outages"** — Measured on realistic driving corridors.
- ✅ **"99.7% – 100.0% 3D Flyover Layer Disambiguation"** — Achieved via 3D HMM with $|\Delta h| > 4.0\text{ m}$ elevation barrier.
- ✅ **">50 kHz Edge CLI Engine Throughput ($17.85\ \mu\text{s}$ latency)"** — Benchmarked on high-rate streaming logs.
- ✅ **"Real-Time ISRO NavIC / IRNSS Satellite Constellation Monitoring"** — Ingested via Android `GnssStatus`.
- ✅ **"27.25% Best Research Median Drift / 49/158 Passing Outages"** — Achieved in Python research (Phases 16 & 17).
- ✅ **"35.71% C++ Production Baseline Median Drift"** — Validated on full 158-outage population in C++17 core.

---

### What We Must NOT Claim (Forbidden / Unverified)
- ❌ **DO NOT CLAIM:** "The entire 158-outage dataset achieves $<10\%$ drift." (Always state that realistic corridors pass, but whole dataset median is 35.71% in C++ and 27.25% in Python).
- ❌ **DO NOT CLAIM:** "27.25% drift is running on the Android device." (27.25% is Python research; Android runs the 35.71% C++ baseline).
- ❌ **DO NOT CLAIM:** "Physical FOG hardware has been validated." (High-rate FOG streaming is validated in software; hardware interfacing is pending).
- ❌ **DO NOT CLAIM:** "Worldwide 3D vector road graphs are bundled offline." (Raster tiles are global; topological 3D `.idrmap` graphs are regional).
