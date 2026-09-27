# PART 2 INTEGRITY SPECIFICATION
## SIH 2026 / ISRO-SAC PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Navigation Integrity Management, Innovation Monitoring & Covariance Consistency

**Document Version:** 1.0.0  
**Phase:** Part 2 of 3  
**Status:** Authoritative Technical Specification  
**Baseline:** B1/N8 + M6 Autonomous Dead-Reckoning Core  
**Implementation:** `idr_pipeline/integrity_manager.py` (Python), `idr-core/src/integrity_manager.cpp` (C++17)

---

## 1. Executive Summary & Purpose

In dead-reckoning and multi-sensor fusion systems, catastrophic divergence typically occurs not from continuous white noise, but from **unmodeled outliers, sensor anomalies, and false map snaps**:
1. Multipath reflections on tunnel exits causing sudden 50–100 m position spikes.
2. Speed bumps or potholes injecting transient acceleration shocks into inertial integration.
3. Map mismatches snapping a vehicle to an erroneous parallel road or overhead flyover.
4. Covariance matrix ill-conditioning or unconstrained drift during long outages.

The **`IntegrityManager`** acts as a centralized supervisor monitoring all Kalman innovations, temporal statistics, covariance growth, and sensor health before any measurement update is permitted to perturb the 15-state ESKF.

---

## 2. Multi-Channel Innovation Monitoring (NIS)

For any observation channel $j$, the measurement innovation is:
$$\mathbf{r}_j = \mathbf{z}_j - \mathbf{h}_j(\hat{\mathbf{x}})$$
with innovation covariance:
$$\mathbf{S}_j = \mathbf{H}_j \mathbf{P} \mathbf{H}_j^T + \mathbf{R}_j$$
The Normalized Innovation Squared (NIS) is:
$$\text{NIS}_j = \mathbf{r}_j^T \mathbf{S}_j^{-1} \mathbf{r}_j$$
Under nominal Gaussian assumptions, $\text{NIS}_j$ follows a central $\chi^2$ distribution with degrees of freedom (DOF) $m$ equal to the measurement dimension.

### 2.1 The Multi-Channel Chi-Square Matrix

| Channel | State Observed | DOF ($m$) | Confidence Level ($1 - \alpha$) | Chi-Square Gate ($\chi^2$) | Operational Action if Exceeded |
|---|---|:---:|:---:|:---:|---|
| **GNSS Position** | $\delta\mathbf{p}^n \in \mathbb{R}^3$ | 3 | $99.0\%$ ($\alpha = 0.01$) | **11.345** | Reject fix; prevent multipath step corruption |
| **GNSS Velocity** | $\delta\mathbf{v}^n \in \mathbb{R}^3$ | 3 | $97.0\%$ ($\alpha = 0.03$) | **9.000** | Reject fix; preserve pre-outage Doppler momentum |
| **AI Forward Speed** | $v_x^b \in \mathbb{R}^1$ | 1 | $99.0\%$ ($\alpha = 0.01$) | **6.635** | De-weight speed covariance $R_{vx} \leftarrow 2.0 \cdot R_{vx}$ |
| **Road Heading** | $\delta\psi \in \mathbb{R}^1$ | 1 | $97.3\%$ ($\alpha = 0.027$) | **9.000** | Reject road heading; prevent erroneous yaw pull |
| **Road Cross-Track** | $\delta p_{\text{cross}} \in \mathbb{R}^1$ | 1 | $99.99\%$ ($\alpha = 10^{-4}$) | **16.000** | Reject road position; prevent off-network pull |
| **Lateral NHC** | $v_y^b \approx 0$ | 1 | $97.3\%$ | **9.000** | Relax lateral constraint during cornering |
| **Vertical NHC** | $v_z^b \approx 0$ | 1 | $97.3\%$ | **9.000** | Relax vertical constraint during curb strikes |

---

## 3. Temporal Hysteresis & Rolling Statistics

A major flaw in naive Chi-Square gating is susceptibility to isolated sensor spikes: rejecting a single noisy sample is proper, but immediately terminating aiding leads to filter starvation. Conversely, accepting corrupted measurements because of an instantaneous dip in NIS leads to filter divergence.

The `IntegrityManager` implements **temporal hysteresis**:
1. **Trailing Rolling Buffer:** A sliding window of the last $N = 10$ epochs tracks rolling NIS:
   $$\overline{\text{NIS}}_j = \frac{1}{10}\sum_{i=0}^9 \text{NIS}_{j, k-i}$$
2. **Consecutive Rejection Counter ($N_{\text{consec}}$):**
   - If $\text{NIS}_j > \chi^2_{\text{gate}}$, $N_{\text{consec}} \leftarrow N_{\text{consec}} + 1$.
   - If $\text{NIS}_j \le \chi^2_{\text{gate}}$, $N_{\text{consec}} \leftarrow 0$.
3. **Threshold for Fault Declaration:**
   - Isolated spike ($N_{\text{consec}} < 5$): Measurement rejected, but channel marked healthy.
   - Persistent failure ($N_{\text{consec}} \ge 5$): Channel declared **UNCERTAIN**, triggering state machine transition.

---

## 4. Covariance Consistency & Divergence Protection

In long GNSS outages without absolute positioning, the error-state covariance matrix $\mathbf{P}$ naturally accumulates uncertainty due to inertial sensor noise. However, numerical instability or ill-conditioned updates can cause unbounded covariance explosion.

### 4.1 Monitored Invariants:
1. **Total Position Covariance Trace:**
   $$\text{Tr}(\mathbf{P}_{pp}) = P_{0,0} + P_{1,1} + P_{2,2} \le 1000.0\text{ m}^2$$
   If $\text{Tr}(\mathbf{P}_{pp}) > 1000.0\text{ m}^2$, position uncertainty exceeds $1\sigma \approx 31.6\text{ m}$.
   If $\text{Tr}(\mathbf{P}_{pp}) > 2000.0\text{ m}^2$, filter is marked **`FAULT`**.
2. **Yaw Standard Deviation:**
   $$\sigma_\psi = \sqrt{\max(0, P_{8,8})} \times \frac{180}{\pi} \le 45.0^\circ$$
   If heading uncertainty exceeds $45^\circ$, map matching is suspended to prevent false snaps.

---

## 5. Integrity State Machine

```
              +-------------------+
              |      HEALTHY      |  (All channels nominal, GNSS fix accepted)
              +-------------------+
                        |  GNSS Outage Entry
                        v
              +-------------------+
              |     DEGRADED      |  (Autonomous DR, C_map >= 0.35)
              +-------------------+
                 /        |        \
    C_map < 0.35/   Vib > 25/    GNSS\ Reacquired
               v          v           v
     +--------------+ +--------------+ +---------------+
     | MAP_UNCERTAIN| | IMU_UNCERTAIN| |  RECOVERING   |
     +--------------+ +--------------+ +---------------+
               \          |           /
                \         |          /  Tr(P) > 2000 m^2
                 v        v         v
              +-------------------+
              |       FAULT       |
              +-------------------+
```

### State Definitions & Operational Rules:
- **`HEALTHY`:** GNSS fixes accepted within $\chi^2 \le 11.35$. Covariance bounded. Full navigation output.
- **`DEGRADED`:** GNSS unavailable. Dead reckoning active with verified road constraints ($C_{\text{map}} \ge 0.35$).
- **`MAP_UNCERTAIN`:** Off-network, high parallel road ambiguity, or 3 consecutive road innovation rejections. **Map updates disabled; inertial propagation preserved.**
- **`IMU_UNCERTAIN`:** High vibration energy ($E_{\text{vib}} > 25\text{ m/s}^2$) or persistent AI speed residual failure. AI speed de-weighted; NHC and ZUPT strengthened.
- **`RECOVERING`:** Emerging from GNSS outage. Requires 3 consecutive valid epochs before transitioning back to `HEALTHY`.
- **`FAULT`:** Unrecoverable divergence. Fail-safe flag set in telemetry.
