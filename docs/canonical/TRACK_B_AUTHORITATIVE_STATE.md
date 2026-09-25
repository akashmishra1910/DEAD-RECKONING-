# TRACK B: AUTHORITATIVE STATE — ALGORITHMIC RESEARCH & EVALUATION
## Machine Learning, ESKF Optimizations, and IO-VNBD 158-Outage Benchmarks

**Document ID:** `DOC-CANONICAL-TRACK-B-04`  
**Version:** 2.0.0  
**Date:** September 25, 2026  
**Status:** VALIDATED RESEARCH PIPELINE  

---

## 1. Scope and Objective

Track B contains the analytical algorithms, machine learning models, training scripts, feature engineering pipelines, and rigorous evaluation harnesses developed to satisfy the scientific requirements of SIH 2026 PS 26168.

All Track B Python reference code lives in `idr_pipeline/` and is validated against the 158 GNSS outages of the **IO-VNBD** real vehicle dataset.

---

## 2. Research Progression & Milestones

The research progression is tracked across sequential phases:

```
Phase 0-4: Dataset Ingestion & GBDT Speed Model Formulation
    │
Phase 5-8: Classical 15-State ESKF Formulation (N0 Baseline)
    │
Phase 9-10: C++ Port & Frozen N7 Production Baseline (N7 Ref)
    │
Phase 13-15: Feature Importance & Window Optimization
    │
Phase 16: Adaptive Noise Formulation (27.25% Median Drift)
    │
Phase 17: Unified AI Fusion Engine (27.64% Median Drift, 49/158 Passes)
```

---

## 3. Dataset Characteristics (Mandated IO-VNBD)

- **Dataset:** Real vehicle recordings from [IO-VNBD](https://github.com/onyekpeu/IO-VNBD).
- **Total Duration:** Multi-hour driving sequences across complex urban and highway routes.
- **Sensors:** 100 Hz Phone IMU (accelerometer, gyroscope, magnetometer), Barometer, OBD-II speed ground truth, RTK-GNSS ground truth.
- **Outage Evaluation Suite:** 158 injected and natural GNSS denial intervals ranging from 10s to 180s duration across diverse dynamics (straight cruise, stop-and-go, sharp turns, flyovers).

---

## 4. Model Architecture & Transpilation

### 4.1 AI Speed Regressor
- **Architecture:** Gradient Boosted Decision Trees (GBDT).
- **Ensemble Hyperparameters:** 120 trees, max depth 5, learning rate $\eta = 0.08$.
- **Window Length:** 1.0 s sliding window (100 samples at 100 Hz).
- **Features Extracted (Top 45):**
  - Statistical moments: mean, variance, skewness, kurtosis of tri-axial acceleration and angular rate.
  - Signal Magnitude Area (SMA) and vibration energy envelopes.
  - Centripetal kinematic indicator: $a_y \approx v \cdot \omega_z$.
  - High-frequency pothole/engine vibration bandpower ratios.
- **C++ Transpilation:** Serialized to static arrays of thresholds and leaf values, compiled directly into binary code without ONNX or TFLite dependencies.

### 4.2 Learned Adaptive Fusion (Phase 16 / 17 Innovation)
In Phase 16 and 17, an adaptive measurement noise model was developed:
$$R_{\text{ai}}(t) = R_0 \cdot \exp\left( \alpha \cdot \text{vib\_energy}(t) + \beta \cdot |\omega_z(t)| \right)$$
- Increases filter trust during smooth steady-state cruise.
- Reduces filter trust during sharp maneuvers and severe road vibration.
- Results in an improvement from $35.71\%$ median error down to $27.25\%$ median error on the 158-outage benchmark.

---

## 5. Parity & Separation Policy

1. **Parity Requirement:** Any research model or filter modification from `idr_pipeline/` must pass a floating-point parity test against the C++ core on replayed logs before being merged into Track A production binaries.
2. **Current Production State:** Track A production C++ is validated against the **Frozen N7 Baseline**. Python Phase 16/17 models remain in `idr_pipeline/` as validated research candidates.
