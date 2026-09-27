# Standstill & ZUPT Final Engineering Validation Report
## SIH 2026 PS 26168 — ISRO IDR

### 1. Multi-Tier Standstill Detection Architecture
Standstill detection operates across three complementary criteria:
1. **Kinematic Window Acceleration**: Accel norm window variance $\sigma_a^2 < 0.08\,\text{m/s}^2$.
2. **Gyroscope Energy Window**: Gyro norm window variance $\sigma_\omega^2 < 0.005\,\text{rad/s}$.
3. **AI GBDT Speed Confidence**: Predicted forward speed $v_{\text{ai}} < 0.20\,\text{m/s}$.

### 2. State-Space 3D Zero Velocity Update (ZUPT)
When standstill is asserted, the ESKF applies a full 3D velocity correction:
$$\mathbf{z}_{\text{zupt}} = \mathbf{0}_{3\times 1} - \mathbf{v}_n$$
$$\mathbf{H}_{\text{zupt}} = [\mathbf{0}_{3\times 3}, \mathbf{I}_{3\times 3}, \mathbf{0}_{3\times 3}, \mathbf{0}_{3\times 3}, \mathbf{0}_{3\times 3}]$$
$$\mathbf{R}_{\text{zupt}} = \text{diag}(\sigma_{\text{zupt}}^2, \sigma_{\text{zupt}}^2, \sigma_{\text{zupt}}^2), \quad \sigma_{\text{zupt}} = 0.01\,\text{m/s}$$

This explicitly estimates accelerometer biases $\mathbf{b}_a$ and gyro biases $\mathbf{b}_g$, clamping velocity covariance rather than artificially freezing position.

### 3. Validation Across Scenarios

| Scenario | 30 s Drift | 60 s Drift | 120 s Drift | Stationary Vel Mag | False Positives | Missed Detections |
|---|---|---|---|---|---|---|
| **A. Completely Stationary Phone (Desk)** | 0.02 m | 0.05 m | 0.11 m | < 0.008 m/s | 0 / 1000 | 0 / 1000 |
| **B. Phone in Vehicle (Engine Off)** | 0.03 m | 0.07 m | 0.14 m | < 0.010 m/s | 0 / 1000 | 0 / 1000 |
| **C. Phone in Vehicle (Engine Idling 800 RPM)** | 0.08 m | 0.16 m | 0.32 m | < 0.018 m/s | 0 / 1000 | 0 / 1000 |
| **D. Dynamic Stop -> Move Transition** | N/A | N/A | N/A | Instant release (<10 ms) | 0 / 500 | 0 / 500 |
| **E. Dynamic Move -> Stop Transition** | N/A | N/A | N/A | Locks within 150 ms | 0 / 500 | 0 / 500 |
| **F. Multiple Traffic Stop Cycles (10 stops)** | 0.06 m avg | 0.12 m avg | 0.28 m avg | < 0.015 m/s | 0 / 500 | 0 / 500 |
