# 03 — SOLUTION ARCHITECTURE & SYSTEM DESIGN
## End-to-End Five-Stage Intelligent Dead Reckoning Pipeline

---

### Pipeline Overview Diagram

```mermaid
flowchart TD
    subgraph SENSORS ["1. Hardware Ingestion Layer"]
        A1["Phone IMU (100 Hz)\n3-axis Accel + 3-axis Gyro"]
        A2["External IMU Stream (~200 Hz)\nFOG / Industrial MEMS"]
        A3["Magnetometer (20-50 Hz) & Barometer (10 Hz)"]
        A4["GNSS Receiver (1 Hz)\nNavIC / GPS Fixes & Raw GnssStatus"]
    end

    subgraph STAGE1 ["Stage 1: Alignment & Preprocessing"]
        B1["Gravity Leveling (Roll/Pitch) + PCA Yaw Mount Calibration"]
        B2["Cradle Watchdog (Angular-Rate Variance Check)"]
        B3["1.0s Sliding Window Feature Extraction (45 Features)"]
    end

    subgraph STAGE2 ["Stage 2: AI Speed Estimation"]
        C1["Branchless Transpiled GBDT Regressor\n(120 Trees, Depth 5, η=0.08)"]
        C2["Vibration & Engine Idling Blanking"]
    end

    subgraph STAGE3 ["Stage 3: 15-State ESKF Fusion"]
        D1["Strapdown Inertial Propagation (dt)"]
        D2["AI Speed Pseudo-Measurement Update"]
        D3["Non-Holonomic Constraints (v_y^b ≈ 0, v_z^b ≈ 0)"]
        D4["Standstill Zero-Velocity Update (ZUPT)"]
    end

    subgraph STAGE4 ["Stage 4: Causal 3D Map Matching"]
        E1["Binary .idrmap Road Graph Reader"]
        E2["3D HMM Trellis with |Δh| > 4.0m Elevation Barrier"]
    end

    subgraph STAGE5 ["Stage 5: Deficit Supervisor & Delivery"]
        F1["Chi-Square Innovation Gate (γ ≤ 11.35)"]
        F2["6-State FSM Navigation Supervisor"]
        F3["Android 10 Hz UI / Edge CLI Stream"]
    end

    A1 & A2 --> B1 --> B2 --> B3 --> C1 --> C2 --> D2
    A1 & A2 --> D1 --> D2
    A1 --> D3 & D4
    A3 --> D1
    A4 --> F1 --> F2 --> D1
    D1 & D2 & D3 & D4 --> E2
    E1 --> E2 --> F3
```

### Architectural Highlights
- **Zero External ML Runtimes:** GBDT regression tree logic is transpiled into pure C++ conditional branch tables, eliminating ONNX, TFLite, and PyTorch runtime overheads.
- **Strict Decoupling:** `idr-core` is sample-rate agnostic and depends solely on standard C++17 library primitives.
