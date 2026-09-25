# 11 — PS 26168 TRACEABILITY SUMMARY
## Concise Compliance Summary for Submission

---

| PS Item | Requirement Description | Verification Status | Implementation & Evidence |
|---|---|:---:|---|
| **PS-01** | AI speed estimation from IMU | **VERIFIED** | Transpiled GBDT ($R^2=0.884$) in `idr-core/src/gbdt_speed.cpp`. |
| **PS-02** | 15-state ESKF formulation | **VERIFIED** | Error-state Kalman filter in `idr-core/src/eskf.cpp`. |
| **PS-03** | Non-holonomic constraints (NHC) | **VERIFIED** | Body-frame velocity suppression ($v_y^b \approx 0, v_z^b \approx 0$) in C++ core. |
| **PS-04** | Standstill detection (ZUPT) | **VERIFIED** | Energy variance standstill clamping in C++ core. |
| **PS-05** | Portable C++ core engine | **VERIFIED** | Pure C++17 library with CMake; zero external ML runtimes. |
| **PS-06** | Edge engine @ ~200 Hz with external IMU | **VERIFIED** | `idr-cli` streaming at $56,026\text{ Hz}$ throughput ($17.85\ \mu\text{s}$ latency). |
| **PS-07** | 10 Hz Android application | **VERIFIED** | 12-Screen Jetpack Compose app with JNI bindings. |
| **PS-08** | Offline 3D multi-layer map matching | **VERIFIED** | 3D HMM with $|\Delta h| > 4.0\text{ m}$ barrier ($99.7\%-100\%$ layer acc). |
| **PS-09** | GNSS deficit & multipath supervisor | **VERIFIED** | $\chi^2 \le 11.35$ innovation gate in C++ core. |
| **PS-10** | ISRO NavIC constellation tracking | **VERIFIED** | Android `GnssStatus` tracking `CONSTELLATION_IRNSS` in real-time. |
| **PS-11** | Magnetometer yaw aiding & disturbance check | **VERIFIED** | Magnetic norm residual check in C++ core. |
| **PS-12** | Mandatory IO-VNBD dataset evaluation | **VERIFIED** | 158 authentic outages evaluated on mandated IO-VNBD dataset. |
| **PS-13** | Two-wheeler lean-compensated mode | **VERIFIED** | Roll-compensated NHC algorithm in C++ core. |
| **PS-14** | Dataset-wide <10% drift bound | **GAP DISCLOSED** | Realistic corridors pass ($1.62\%-2.88\%$); dataset-wide median is $35.71\%$ in C++ and $27.25\%$ in Python. |
| **PS-15** | Physical FOG hardware validation | **PENDING** | Validated on high-rate software streams; physical FOG hardware connection pending. |
