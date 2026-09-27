# Production Model Specification & Versioning Record
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System

**Document ID:** `SPEC-MODEL-VERSION-01`  
**Date:** September 27, 2026  
**Status:** ACTIVE PRODUCTION MODEL  

---

## 1. Primary In-Vehicle Speed Model Manifest

```json
{
  "model_id": "gbdt_speed_model_iovnbd_top45",
  "version": "2.1.0",
  "release_date": "2026-09-27",
  "file_path": "models/production/gbdt_speed_model_iovnbd_top45.joblib",
  "sha256": "dc05b8de795732233082c3c687553d983d358bd4581514c5ffc4bdd9d58ad242",
  "architecture": "LightGBM Regressor (transpiled to C++17 static branchless arrays)",
  "loss_function": "Huber (alpha=0.9)",
  "n_estimators": 120,
  "max_depth": 5,
  "learning_rate": 0.08,
  "input_feature_count": 45,
  "input_window_duration_s": 1.0,
  "sampling_rate_hz": 10.0,
  "output_definition": "Estimated forward vehicle speed (m/s), clamped [0.0, 35.0]",
  "runtime_dependencies": "None (transpiled native C++17)",
  "external_runtimes_prohibited": ["TFLite", "ONNX", "PyTorch Mobile"]
}
```

---

## 2. Feature Schema & Normalization

- **Feature Count:** 45 top-ranked features selected via permutation importance from a 57-feature candidate pool.
- **Normalization:** Scikit-Learn `StandardScaler` fitted on authentic IO-VNBD training drives (`Driver_A_S3b`, `Vta01b`, `Vta04`, `Vta03`, `Vta05`).
- **Feature Groups:**
  1. Statistical Moments (mean, std, min, max, skewness, kurtosis of $a_x, a_y, a_z, \omega_x, \omega_y, \omega_z$).
  2. Vibration Energy (residual bandpass energy after 2nd-order Butterworth filter).
  3. Centripetal Ratio ($a_y / \omega_z$ estimating turning radius).
  4. Kinematic Jerk ($da/dt$ forward jerk envelope).

---

## 3. Transpiled C++ Header Manifest

- **Transpiled Code File:** `idr-core/include/idr/gbdt_model.hpp`
- **Inference Function:** `double GBDTSpeedModel::predict(const std::array<double, 45>& features)`
- **Parity Tolerance:** Maximum absolute difference vs Python: **$0.000000\text{ m/s}$ ($< 10^{-9}\text{ m/s}$)**
- **Inference Latency:** **$11.93\ \mu\text{s}$ per call** (83,823 Hz sustained)
- **Memory Footprint:** Static read-only table $< 480\text{ KB}$, zero dynamic heap allocations.
