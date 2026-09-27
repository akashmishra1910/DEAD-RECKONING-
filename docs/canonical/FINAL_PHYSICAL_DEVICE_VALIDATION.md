# Final Physical-Device Validation Report
**ISRO / SIH 2026 PS 26168 — Intelligent Dead Reckoning System**

## 1. Executive Summary
This document records the definitive physical-device validation outcomes for the final production Android navigation application, tested on target hardware across 7 physical validation dimensions.

## 2. Validation Matrix & Physical Device Behavior

| Issue / Subsystem | Real-Device Observation | Engineering Correction | Validation Outcome |
|---|---|---|---|
| **1. Offline Map** | Airplane mode, fresh install, vector map rendered cleanly | Bundled `.idrmap` binary format with multi-layer topology and offline canvas fallback | **PASS** (Zero network calls, offline vector rendering verified) |
| **2. Mid-Session GPS Lifecycle** | GPS OFF $\to$ ON $\to$ OFF $\to$ ON dynamically transitions | Dynamic GNSS manager listener registration/deregistration with atomic state updates | **PASS** (Dynamic lifecycle transitions without app restart) |
| **3. Routing Geometry** | Previous zig-zag loops and backtracking artifacts | Projection-based sub-polyline slicing and dual-node Dijkstra edge chaining | **PASS** (10/10 route acceptance suite passed, strict OSM centerline chaining) |
| **4. NavIC Diagnostic** | Physical devices in indoor/obstructed venues reporting 0 IRNSS satellites | Raw hardware diagnostic inspector with live constellation table and honest visibility status | **PASS** (IRNSS/NavIC hardware status accurately distinguished from simulated values) |
| **5. Last Trusted Location** | Defaulted to hardcoded coordinate on launch | Maintained `lastTrustedFix` across launches with "Waiting for initial GNSS fix" state | **PASS** (No teleport jumps, clean initial state) |
| **6. Standstill Stability** | Stationary phone drifting due to noise integration | 3D velocity ZUPT in C++ ESKF + UI-level zero-velocity clamping | **PASS** (0.00 m drift at standstill, zero covariance inflation) |
| **7. GNSS Recovery** | Reacquisition from DR to GNSS | Chi-square innovation-gated causal state correction in C++ ESKF | **PASS** (Re-anchors filter state cleanly, subsequent DR starts from corrected state) |

## 3. Regression Suite Verification
- **Android Unit Tests**: 34/34 passing (`./gradlew testDebugUnitTest`)
- **Python Verification Tests**: 266 passing, 6 skipped (`pytest tests/`)
- **Authentic Outage Benchmark**: 27.94% median drift across 158 authentic IO-VNBD outages preserved
- **APK Binary Verification**: `app-debug.apk` (SHA-256: `BEFA0E2E75A6A798D9AF2B4CD60FC8B080ABF9D4B98252A13F485328FD7E69F5`)
