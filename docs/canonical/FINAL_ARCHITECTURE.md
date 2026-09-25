# FINAL SYSTEM ARCHITECTURE — SIH26168
## End-to-End Dead Reckoning & Navigation System Design

**Document ID:** `DOC-CANONICAL-ARCH-11`  
**Version:** 2.0.0  
**Date:** September 25, 2026  

---

## 1. System Pipeline Block Diagram

```mermaid
flowchart TD
    subgraph INGESTION ["1. Hardware Ingestion Layer"]
        A1["Phone IMU (100 Hz)\nAccel, Gyro, Mag, Baro"]
        A2["External IMU Stream (~200 Hz)\nFOG / Industrial MEMS"]
        A3["GNSS Receiver (1 Hz)\nNavIC / GPS Fixes & Raw GnssStatus"]
    end

    subgraph PREPROCESSING ["2. Alignment & Preprocessing"]
        B1["Gravity Leveling (Roll/Pitch)\n+ PCA Longitudinal (Yaw)"]
        B2["Cradle Watchdog\nSlip & Angular-Rate Variance Detection"]
        B3["1.0s Sliding Window Feature Extractor\n(Top-45 Statistical Moments & Vibration Bands)"]
    end

    subgraph ESTIMATION ["3. Core Navigation Engine (idr-core)"]
        C1["Branchless Transpiled GBDT\nForward Speed Regressor"]
        C2["15-State Continuous-Discrete ESKF\n[p, v, q, b_a, b_g]"]
        C3["Non-Holonomic Constraints (NHC)\nv_y ≈ 0, v_z ≈ 0"]
        C4["Standstill ZUPT Detector\nEnergy Variance < Threshold"]
        C5["GNSS Deficit Supervisor\nChi-Square Innovation Gate (γ ≤ 11.35)"]
    end

    subgraph MAPMATCH ["4. Multi-Layer Map Matching"]
        D1["Binary .idrmap Graph Reader\n(Ground, Flyover, Tunnel)"]
        D2["Causal 3D HMM Viterbi Trellis\nElevation Barrier |Δh| > 4.0m"]
    end

    subgraph PRESENTATION ["5. Presentation & Delivery"]
        E1["Android UI (10 Hz StateFlow)\n12-Screen Global Geographic Product"]
        E2["Edge CLI Binary (idr-cli)\nStreaming Socket / File Runner"]
    end

    A1 --> B1
    A2 --> B1
    B1 --> B2 --> B3
    B3 --> C1
    C1 --> C2
    A1 --> C3
    A1 --> C4
    A3 --> C5
    C5 --> C2
    C2 --> D2
    D1 --> D2
    D2 --> E1
    D2 --> E2
```

---

## 2. Component Interfaces & Data Contracts

1. **Sensor Ingestion:** Standardized `ImuSample` ($t, \mathbf{a}, \boldsymbol{\omega}$), `MagSample` ($t, \mathbf{m}$), and `GnssSample` ($t, \text{lat}, \text{lon}, \text{alt}, \sigma$).
2. **Navigation State Output:** Standardized `NavigationTelemetry` emitting position (ENU & WGS84), velocity, attitude quaternion, covariance diagonals, mode (`INITIALIZING`, `GNSS_FIXED`, `DEAD_RECKONING`, `MAP_AIDED`), and diagnostic health indicators.
3. **Map Storage (`.idrmap`):** Compact binary format encoding node coordinates, road segment polylines, topological connectivity, street names, speed limits, and discrete z-levels (`Ground`, `Flyover`, `Underpass`).
