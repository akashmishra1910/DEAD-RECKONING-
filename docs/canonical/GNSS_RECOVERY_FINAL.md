# GNSS Recovery & Reacquisition Final Report
**ISRO / SIH 2026 PS 26168 — Intelligent Dead Reckoning System**

## 1. Executive Summary
This document defines and validates the causal GNSS $\to$ DR $\to$ GNSS state transitions in both the C++ 15-state ESKF engine and the Android mobile navigation client.

## 2. Reacquisition Mechanics
When GNSS signal is lost:
1. The supervisor switches filter state to `DEAD_RECKONING`.
2. AI speed regressor + Non-Holonomic Constraints (NHC) + ZUPT drive state propagation and error covariance updates.
3. Live drift bounds expand proportional to elapsed outage duration and integrated IMU uncertainty.

When GNSS fix is restored:
1. **Chi-Square Innovation Gating**:
   $$\gamma = \mathbf{r}^T \mathbf{S}^{-1} \mathbf{r} \le 11.35 \quad (\alpha=0.01, m=3)$$
   Rejects multipath transients immediately upon tunnel exit or re-emergence.
2. **Causal Filter Correction**:
   The measurement innovation $\mathbf{r} = \mathbf{z}_{\text{gnss}} - \hat{\mathbf{p}}$ updates the full 15-state vector $\mathbf{x} = [\delta\mathbf{p}, \delta\mathbf{v}, \delta\mathbf{\theta}, \mathbf{b}_a, \mathbf{b}_g]^T$ via the Kalman gain $\mathbf{K} = \mathbf{P}\mathbf{H}^T\mathbf{S}^{-1}$.
3. **Internal State Re-anchoring**:
   The underlying C++ filter state is physically corrected to the GNSS fix rather than merely snapping the UI cursor.
4. **Subsequent Dead Reckoning**:
   When a subsequent outage occurs, dead reckoning integrates forward from the corrected filter state rather than carrying forward pre-reacquisition accumulated drift.

## 3. Physical Device Validation
- Tested with multiple simulated and physical outage sequences.
- Transition from `GNSS OK` $\to$ `GNSS DENIED — DEAD RECKONING` $\to$ `GNSS REACQUIRED` executes smoothly.
- Zero discontinuity, zero state explosion, and immediate re-convergence of error covariance.
