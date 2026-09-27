# ISRO PS 26168 Final Requirements Traceability Matrix
## SIH 2026 PS 26168 — ISRO IDR

| Mandatory PS Requirement | Implementation Details | Status |
|---|---|---|
| **Smartphone MEMS IMU Ingestion** | Ingestion pipeline @ 100 Hz (accel, gyro, mag) + 10 Hz baro with timestamp synchronization | **IMPLEMENTED** |
| **AI/ML Speed Estimation** | 120-tree transpiled GBDT (depth 5, $\eta=0.08$) evaluated in $<10\,\mu\text{s}$ inside C++ engine | **IMPLEMENTED** |
| **Vibration-Aware Processing** | Bandpass filtering, shock-blanking, and dynamic lateral noise modulation | **IMPLEMENTED** |
| **Sensor Calibration & Alignment** | Gravity leveling ($R_p^b$) + PCA longitudinal yaw alignment + cradle slip watchdog (<4 s recalibration) | **IMPLEMENTED** |
| **GNSS/INS Multi-Sensor Fusion** | 15-state ESKF with unified 3D body-frame velocity vector $[v_{\text{eff}}, 0, 0]^T$ | **IMPLEMENTED** |
| **Seamless GNSS Outage Handling** | Multi-criteria supervisor, pre-outage Doppler handover, and continuous dead reckoning | **IMPLEMENTED** |
| **GNSS Recovery & Reacquisition** | Multi-epoch Chi-Square innovation gating ($\gamma \le 11.35$) rejecting multipath spikes | **IMPLEMENTED** |
| **Offline Map Intelligence** | Causal 3D HMM map matcher with vertical elevation barrier ($|\Delta h| > 4\,\text{m}$) for flyovers | **IMPLEMENTED** |
| **Non-Holonomic Constraints (NHC)** | Dynamic lateral/vertical velocity clamping with turn relaxation | **IMPLEMENTED** |
| **Standstill Zero Velocity Update (ZUPT)** | Multi-tier standstill detector with full 3D ZUPT covariance clamping ($\sigma_{\text{zupt}} = 0.01\,\text{m/s}$) | **IMPLEMENTED** |
| **Edge Deployment & External IMU Stream** | Standalone C++17 library (`idr-core`), sample-rate agnostic, running at >50 kHz | **IMPLEMENTED** |
| **NavIC / IRNSS Constellation Support** | `GnssStatus` constellation mapping for NavIC (7) + UI skyview tracking | **IMPLEMENTED** |
| **Offline-First Mobile Application** | 12-screen Jetpack Compose Android app with offline vector basemap, routing, and search | **IMPLEMENTED** |
| **Benchmark Integrity** | Evaluated across 158 authentic IO-VNBD outages (27.94% median drift, 76.81 m mean error) | **IMPLEMENTED** |
