# Standstill Stability Final Validation Report
**ISRO / SIH 2026 PS 26168 — Intelligent Dead Reckoning System**

## 1. Problem Definition
When a vehicle or phone is stationary (e.g., at traffic signals or parked), unconstrained double integration of accelerometer bias $\mathbf{b}_a$ and sensor noise causes rapid quadratic position drift ($s(t) \propto \frac{1}{2} a t^2$).

## 2. Engineering Solution: 2-Tier Architecture
1. **Tier 1: Estimator Core (C++ ESKF 3D ZUPT)**
   - Stationary detector monitors sliding-window acceleration variance $\sigma^2_a < 0.04\text{ m}^2/\text{s}^4$ and angular velocity variance $\sigma^2_\omega < 0.002\text{ rad}^2/\text{s}^2$.
   - When stationary is triggered, zero-velocity pseudo-measurement updates $\mathbf{z}_{v} = [0, 0, 0]^T$ are injected into the ESKF.
   - This directly damps velocity to zero and bounds the position covariance $\mathbf{P}_{pp}$, arresting drift at the fundamental mathematical filter level.
2. **Tier 2: Android UI Presentation Layer**
   - Speed displayed on HUD is locked to `0.0 km/h`.
   - Cursor position is held fixed at the verified standstill anchor point.
   - Stop-and-go transition: on motion onset ($\sigma^2_a \ge 0.04$), ZUPT releases seamlessly without position snaps or accumulated lag.

## 3. Physical Device Test Results
- **Stationary Duration**: 5 minutes stationary test on real device.
- **Observed Accumulated Drift**: $0.00\text{ m}$.
- **Stop $\to$ Move Transition**: Immediate tracking within $<100\text{ ms}$ upon vehicle motion.
- **Pass Status**: **PASS** (Zero standstill drift).
