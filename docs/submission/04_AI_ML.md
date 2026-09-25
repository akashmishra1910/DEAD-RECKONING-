# 04 — AI / MACHINE LEARNING PIPELINE
## Vibration-Aware Speed Regression & Learned Adaptive Noise

---

### 1. GBDT Speed Regressor Architecture
- **Model Type:** Gradient Boosted Decision Trees with Huber Loss ($\delta = 1.35$).
- **Ensemble Dimensions:** 120 trees, maximum tree depth 5, learning rate $\eta = 0.08$.
- **Window Formulation:** 1.0-second sliding temporal window (100 IMU samples at 100 Hz).
- **Physical Feature Set (Top-45 Ranked):**
  1. Statistical moments (mean, variance, skewness, kurtosis) across tri-axial accelerations and angular rates.
  2. Signal Magnitude Area (SMA) and total vibration energy envelopes.
  3. Centripetal Kinematic Indicator ($a_y \approx v \cdot \omega_z$).
  4. Spectral energy bandpower ratios (engine idling, road roughness, suspension harmonics).
  5. Time-derivative kinematic jerk moments.
- **Transpilation Strategy:** Converted to C++ arrays of floating-point thresholds and leaf assignments, enabling inference in $<0.02\text{ ms}$ per sample without dynamic memory allocation.

### 2. Validation Metrics on Mandated IO-VNBD Dataset
- **Coefficient of Determination ($R^2$):** **0.884**
- **Root Mean Squared Error (RMSE):** **1.21 m/s (4.3 km/h)**
- **Mean Absolute Error (MAE):** **0.87 m/s**
- **Clamping Safety Bounds:** Physical velocity leaf clamp $[0.0, 35.0]\text{ m/s}$ ($0 - 126\text{ km/h}$).

### 3. Learned Adaptive Noise ($R_{\text{ai}}$) Formulation (Research Innovation)
To account for varying road conditions, an adaptive measurement noise model was developed:
$$R_{\text{ai}}(t) = R_0 \cdot \exp\left( \alpha \cdot \text{vib\_energy}(t) + \beta \cdot |\omega_z(t)| \right)$$
- **Effect:** Scales filter confidence automatically—trusting AI speed during steady highway cruise while de-weighting it during intense pothole vibration or sharp turning maneuvers.
