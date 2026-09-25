# 08 — EDGE DEPLOYMENT & EXTERNAL IMU ENGINE
## Standalone High-Rate C++ CLI Runner & Dashboard

---

### 1. The Edge Deliverable Mandate
ISRO PS 26168 explicitly requires the dead reckoning engine to function as a decoupled software module capable of consuming data from external IMUs (including tactical-grade and Fiber-Optic Gyro / FOG units) running at high frequencies (~200 Hz).

### 2. Standalone CLI Runner (`idr-cli`)
- **Binary:** Self-contained executable built directly from `idr-core` using CMake.
- **Input Modes:** Supports streaming CSV logs, binary sensor packets, and UDP/TCP network sockets.
- **Sample-Rate Agnostic:** Accepts $\Delta t$ per sample explicitly, operating dynamically across 10 Hz, 100 Hz, 200 Hz, and up to 1000 Hz sensor feeds.
- **Output:** Outputs timestamped ENU position, velocity, orientation quaternion, covariance diagonals, and map-matched coordinates.

### 3. Edge Execution Performance
- **Measured Throughput:** **$56,026\text{ Hz}$** ($>280\times$ the required 200 Hz rate).
- **Mean Per-Sample Latency:** **$17.85\ \mu\text{s}$ (0.018 ms)**.
- **Memory Footprint:** **$<2.4\text{ MB}$ RSS** (Peak heap: $1.44\text{ MB}$).
- **External IMU Validation:** Validated against I2NAV dataset and simulated multi-axis industrial IMU streams (`fa0340c`, `d545c83`).

### 4. Edge Engine Test Dashboard
- **Web-Based Visualization:** Local WebSocket dashboard (`tools/edge_dashboard/`) providing live attitude gimbal graphics, trajectory tracks, error budgets, and simulated GNSS-denial controls.
