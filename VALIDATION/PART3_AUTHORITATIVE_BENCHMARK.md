# PART 3 AUTHORITATIVE BENCHMARK SPECIFICATION
## SIH 2026 — PS 26168: AI-ML Based Intelligent Dead Reckoning System
### Accuracy Recovery, Causality Boundary & Genuine Benchmark Protocol

**Document Version:** 3.0.0  
**Status:** AUTHORITATIVE & BINDING  
**Dataset:** Authentic IO-VNBD (Coventry/Warwickshire Drives, 158 Standardized Outages)  
**Target Hardware:** Commodity Smartphone (10 Hz inference) & Edge FOG IMU Engine (200 Hz)  

---

## 1. Objective & Benchmark Criteria

The objective of Problem Statement 26168 is to maintain seamless, continuous navigation during GNSS denial using only on-device smartphone sensors, AI speed estimation, kinematic constraints, and offline map matching.

### 1.1 Benchmark Targets
- **Primary SIH Criterion:** Positional drift $\le 10.0\%$ over the entire authoritative evaluation population.
  $$\text{Drift } \% = \frac{\|\mathbf{p}_{\text{est}}(t_{\text{end}}) - \mathbf{p}_{\text{gt}}(t_{\text{end}})\|_{2D}}{d_{\text{travelled}}} \times 100\%$$
- **Case 1 (Short Distance / Low Speed):** $\le 5.0\text{ m}$ position error over $50.0\text{ m}$ in $< 1\text{ minute}$.
- **Case 2 (Highway Cruise):** $\le 100.0\text{ m}$ position error over $1.0\text{ km}$ at $\approx 60\text{ km/h}$.

### 1.2 Mathematical Handling of Low-Distance / Standstill Singularities
For stop-and-go or stationary outages where $d_{\text{travelled}} \to 0$, percentage drift becomes mathematically ill-conditioned ($\frac{\text{pos\_err}}{0} \to \infty$). To ensure transparency without hiding failures:
1. **Uncurated Primary Population (158 / 158 outages):** Computed with numerical lower clamp $d_{\text{travelled}} = \max(1.0\text{ m}, d_{\text{gt}})$. Every outage is reported.
2. **Dynamic Moving Subpopulation ($d_{\text{travelled}} > 50.0\text{ m}$, $N=133$):** Evaluated separately to characterize dynamic kinematic drift without denominator inflation.
3. **Standstill / Short Subpopulation ($d_{\text{travelled}} \le 50.0\text{ m}$, $N=25$):** Evaluated via absolute position error in meters ($\|\Delta\mathbf{p}\|_{2D} \le 5.0\text{ m}$).

---

## 2. Input Boundary: Allowed vs. Forbidden

To ensure absolute scientific honesty and zero data leakage, the evaluation harness enforces a strict cryptographic and programmatic boundary during outage dead-reckoning ($t_0 < t \le t_1$):

### 2.1 Permitted Inputs During Outage ($t_0 < t \le t_1$)
- Raw 3D Accelerometer measurements $\mathbf{f}_p(t)$ at $t \le t_k$.
- Raw 3D Gyroscope measurements $\boldsymbol{\omega}_p(t)$ at $t \le t_k$.
- Raw 3D Magnetometer measurements $\mathbf{m}_p(t)$ at $t \le t_k$ (strictly filtered via disturbance rejection).
- Barometric altitude measurements $h_{\text{baro}}(t)$ at $t \le t_k$.
- Autonomous AI forward speed $v_{\text{AI}}(t)$ inferred causally from trailing IMU windows ($t \le t_k$).
- Pre-outage GNSS Doppler velocity and position fixes available prior to signal loss ($t \le t_0$).
- Offline 3D OpenStreetMap road network geometry, topology, and elevation decks.
- Causal map-matching hypotheses and confidence scores derived strictly from $t \le t_k$.
- Filter internal state $\hat{\mathbf{x}}_k$ and covariance $\mathbf{P}_k$.

### 2.2 Strictly Forbidden Inputs During Outage ($t_0 < t \le t_1$)
- **Ground-Truth Position:** `traj.gt_pos[k]`
- **Ground-Truth Velocity:** `traj.gt_vel[k]`
- **Ground-Truth Speed:** `traj.gt_speed[k]`
- **Ground-Truth Heading / Yaw:** `traj.gt_heading[k]`
- **Ground-Truth Attitude / Pitch / Roll:** `traj.gt_pitch[k]`, `traj.gt_roll[k]`
- **CAN / OBD-II / Wheel Speed Sensors:** Any vehicle bus telemetry.
- **Future Sensor Samples:** Any IMU, baro, or mag sample where $t > t_k$.
- **Post-Outage GNSS Fixes:** Any GNSS data arriving after outage termination used retrospectively to smooth or interpolate the outage track.
- **Ground-Truth Road Edge Selection:** Any map candidate selection guided by ground-truth trajectory points.

---

## 3. Authoritative Outage Population Manifest

The evaluation catalog comprises **158 uncurated GNSS outages** extracted from the 5 authentic drives of the real IO-VNBD dataset:

| Drive Identifier | Vehicle Type | Duration | Total Distance | Total Outages | 10s Outages | 30s Outages | 60s Outages | 120s Outages |
|---|---|---|---|---|---|---|---|---|
| `Driver_A_S3b` | Passenger Car | 1,489 s | 16.4 km | 32 | 16 | 8 | 5 | 3 |
| `Vta01b` | Passenger Car | 1,498 s | 15.8 km | 31 | 16 | 8 | 5 | 2 |
| `Vta03` | Passenger Car | 1,495 s | 14.9 km | 31 | 16 | 8 | 5 | 2 |
| `Vta04` | Passenger Car | 1,510 s | 17.1 km | 32 | 16 | 8 | 5 | 3 |
| `Vta05` | Passenger Car | 1,505 s | 16.2 km | 32 | 17 | 7 | 5 | 3 |
| **Total Population** | - | **7,497 s** | **80.4 km** | **158** | **81** | **39** | **25** | **13** |

Every outage represents a genuine stretch of public road navigation in the Coventry/Warwickshire road network, incorporating complex roundabouts, urban stop-and-go intersections, highway cruising, severe pavement vibrations, and multi-lane changes.
